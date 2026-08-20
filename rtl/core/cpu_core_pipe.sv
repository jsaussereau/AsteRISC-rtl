/**********************************************************************\
*                               AsteRISC                               *
************************************************************************
*
* Copyright (C) 2022 Jonathan Saussereau
*
* This file is part of AsteRISC.
* AsteRISC is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* AsteRISC is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with AsteRISC. If not, see <https://www.gnu.org/licenses/>.
*
*/

//! CPU core (pipeline version)

`ifndef __CPU_CORE_PIPE__
`define __CPU_CORE_PIPE__

`ifdef VIVADO
  `include "../soc/soc_config.sv"
  `include "packages/pck_control.sv"
  `include "packages/pck_isa.sv"
  `include "packages/pck_isa_i.sv"
  `include "packages/pck_mem_bus.sv"
  `include "packages/pck_pipe.sv"
`else
  `include "soc/soc_config.sv"
  `include "core/packages/pck_control.sv"
  `include "core/packages/pck_isa.sv"
  `include "core/packages/pck_isa_i.sv"
  `include "core/packages/pck_mem_bus.sv"
  `include "core/packages/pck_pipe.sv"
`endif

module cpu_core_pipe 
  import pck_control::*;
  import pck_isa::*;
  import pck_pipe::*;
#(
  parameter p_reset_vector = 32'hf0000000,//! address of the 1st instruction

  // extensions:
  parameter p_ext_rve      = 0,           //! use RV32E extension (reduces the integer register count to 16)
  parameter p_ext_rvc      = 1,           //! use RV32C extension (compressed instructions)
  parameter p_ext_rvm      = 1,           //! use RV32M extension (multiplication and division)
  parameter p_ext_rvzicsr  = 1,           //! use RV32Zicsr extension (control and status registers)
  parameter p_ext_custom   = 1,           //! use custom instructions

  parameter p_counters     = 0,           //! use counters (mcycle, minstret)

  parameter p_mul_fast     = 0,           //! fast mul
  parameter p_mul_1_cycle  = 0,           //! one cycle mul

  // ALU micro-architecture (see `cpu_alu`): these do not move a register
  // barrier, they change what stands between two barriers.
  parameter p_alu_share_adder = 0,        //! sub/slt/sltu and the branch comparators reuse the main adder
  parameter p_alu_shift_bits  = 32,       //! bits shifted per cycle: 32 = barrel shifter, 1/2/4/8/16 = sequential

  // pipeline settings: number of barriers of each pipeline group.
  // 0 collapses the group (its slot becomes a combinational alias of the slot
  // upstream), 1 is a plain pipeline stage, N > 1 spreads the group logic over
  // N cycles at unchanged throughput -- several EX stages for a pipelined
  // multiplier, several fetch stages to shorten the instruction memory path.
  parameter p_stage_IF     = 1,           //! fetch barriers (>= 1)
  parameter p_stage_IC     = 0,           //! decompression barriers
  parameter p_stage_ID     = 1,           //! decode barriers
  parameter p_stage_RF     = 0,           //! register file barriers (0 or 1)
  parameter p_stage_EX     = 1,           //! execute barriers
  parameter p_stage_MA     = 1,           //! memory access barriers (>= 1)
  parameter p_stage_WB     = 1,           //! write back barriers

  // pipeline control settings:
  parameter p_early_jal    = 1,           //! resolve `jal` from the decoder instead of the RF slot
  parameter p_redirect_buf = 1,           //! register the redirection target before the fetch stage

  //! slot at which a resolved control transfer is acted upon (see `cpu_hazard`):
  //!   0 = the RF slot, 1 = the EX slot (one more bubble per redirection, but
  //!   the whole redirection fan-out leaves the RF critical path)
  parameter p_branch_stage = 0,           //! branch resolution slot

  // operand forwarding: each group of in-flight writes may or may not be
  // bypassed towards the RF slot. Clearing one shortens the bypass mux in front
  // of the ALU and pays for it with load-use style stalls (see `cpu_hazard`).
  parameter p_fwd_ex       = 1,           //! bypass from the EX barriers
  parameter p_fwd_ma       = 1,           //! bypass from the MA barriers
  parameter p_fwd_wb       = 1,           //! bypass from the WB barriers
  parameter p_fwd_pw       = 1,           //! bypass from the pending register file write

  //! branch prediction scheme (shared with the multi-cycle core):
  //!   0 = off, 1 = static (backward taken / forward not taken).
  //! Higher values are reserved for the dynamic predictors to come.
  parameter p_branch_pred  = 0,           //! branch prediction scheme

  // non pipeline settings:
  parameter p_fetch_buf    = 0,           //! add buffers to fetch stage output
  parameter p_decode_buf   = 0,           //! add buffers to decode stage outputs
  parameter p_rf_sp        = 0,           //! register file is a single port ram
  parameter p_rf_read_buf  = 0,           //! register file has synchronous read
  parameter p_mem_buf      = 0,           //! add buffers to mem stage inputs
  parameter p_wb_buf       = 0,           //! add buffers to write back stage inputs
  parameter p_branch_buf   = 0,           //! add buffers to alu comp outputs (+1 cycle for conditionnal branches)
  parameter p_wait_for_ack = 0            //! wait for data bus acknowledgement
)(
  // global
  input  logic         i_clk,             //! global clock: triggers with a rising edge
  input  logic         i_rst,             //! global reset: **active high**, synchronous
  input  logic         i_sleep,           //! active high sleep control
  input  logic [63: 0] i_mcycle,          //! cycle count
  input  logic [63: 0] i_mtime,           //! real time clock

  // instruction bus interface
  output logic [31:0] ibus_addr,          //! instruction bus address
  output logic [ 3:0] ibus_be,            //! instruction bus write byte enable
  output logic        ibus_wr_en,         //! instruction bus write enable
  output logic [31:0] ibus_wr_data,       //! instruction bus write data
  output logic        ibus_rd_en,         //! instruction bus read enable
  input  logic [31:0] ibus_rd_data,       //! instruction bus read data
  input  logic        ibus_busy,          //! instruction bus busy
  input  logic        ibus_ack,           //! instruction bus transfer acknowledge

  // data bus interface
  output logic [31:0] dbus_addr,          //! data bus address
  output logic [ 3:0] dbus_be,            //! data bus write byte enable
  output logic        dbus_wr_en,         //! data bus write enable
  output logic [31:0] dbus_wr_data,       //! data bus write data
  output logic        dbus_rd_en,         //! data bus read enable
  input  logic [31:0] dbus_rd_data,       //! data bus read data
  input  logic        dbus_busy,          //! data bus busy
  input  logic        dbus_ack,           //! data bus transfer acknowledge

  // coprocessor interface
  output logic [31: 0] o_alu_op_a,        //! ALU operand A
  output logic [31: 0] o_alu_op_b,        //! ALU operand B
  input  logic [31: 0] i_copro_out0,      //! result value computed by coprocessor 0
  input  logic [31: 0] i_copro_out1,      //! result value computed by coprocessor 1
  input  logic [31: 0] i_copro_out2,      //! result value computed by coprocessor 2
  input  logic [ 2: 0] i_copro_valid      //! result value computed by coprocessors
);

  /*******************************************************
    Topology preconditions

    Every `p_stage_*` combination is supported by the control unit, but two of
    them are constrained by the memories the core is wired to rather than by the
    control logic, so they are checked here instead of being silently wrong.
  *******************************************************/

  initial begin
    assert(p_stage_MA >= 1) else $error("parameter \"p_stage_MA\" cannot be disabled: the data memory read is registered, so a load result only exists one barrier after the memory access.");
    assert(p_stage_IF >= 1) else $error("parameter \"p_stage_IF\" cannot be disabled: the instruction memory read is registered, so the fetch stage is always a pipeline stage.");
    assert(p_stage_RF <= 1) else $error("parameter \"p_stage_RF\" cannot exceed 1: the RF barrier *is* the register file read register, and a second one would desynchronise the operands from the payload that selects them.");
  end

  /*******************************************************
    Pipeline geometry

    Every array size, forwarding entry and counter width below is derived from
    the `p_stage_*` counts, so a group of any depth needs no dedicated code.
  *******************************************************/

  //! extra fetch barriers: the fetch stage itself is the first one
  localparam int  N_IF       = p_stage_IF - 1;

  //! array sizes, guarded so that a collapsed group still declares one element
  localparam int  D_IF       = (N_IF       > 0) ? N_IF       : 1;
  localparam int  D_IC       = (p_stage_IC > 0) ? p_stage_IC : 1;
  localparam int  D_ID       = (p_stage_ID > 0) ? p_stage_ID : 1;
  localparam int  D_EX       = (p_stage_EX > 0) ? p_stage_EX : 1;
  localparam int  D_MA       = (p_stage_MA > 0) ? p_stage_MA : 1;
  localparam int  D_WB       = (p_stage_WB > 0) ? p_stage_WB : 1;

  //! One forwarding entry per barrier downstream of the RF slot, youngest
  //! first, plus the pending write when the register file read is registered.
  //! A barrier that does not exist contributes no entry, which is exactly the
  //! old `USE_BP_*` rule generalised: a producer can only be forwarded from if
  //! at least one barrier separates it from the consumer.
  localparam int  N_FWD_EX   = p_stage_EX;
  localparam int  N_FWD_MA   = p_stage_MA;
  localparam int  N_FWD_WB   = p_stage_WB;
  localparam int  N_FWD_PW   = (p_stage_RF != 0) ? 1 : 0;
  localparam int  N_FWD      = N_FWD_EX + N_FWD_MA + N_FWD_WB + N_FWD_PW;

  localparam int  OFF_EX     = 0;
  localparam int  OFF_MA     = OFF_EX + N_FWD_EX;
  localparam int  OFF_WB     = OFF_MA + N_FWD_MA;
  localparam int  OFF_PW     = OFF_WB + N_FWD_WB;

  //! `p_fwd_*` expressed over the flat entry list `cpu_hazard` reasons about. A
  //! clear bit does not remove the entry -- the read-after-write is still
  //! detected -- it only forbids bypassing from it, which turns the hazard into
  //! a stall.
  function automatic logic [31: 0] fwd_mask_build();
    logic [31: 0] m;
    m = 32'd0;
    for (int k = 0; k < N_FWD_EX; k++) m[OFF_EX + k] = (p_fwd_ex != 0);
    for (int k = 0; k < N_FWD_MA; k++) m[OFF_MA + k] = (p_fwd_ma != 0);
    for (int k = 0; k < N_FWD_WB; k++) m[OFF_WB + k] = (p_fwd_wb != 0);
    if (N_FWD_PW != 0)                 m[OFF_PW    ] = (p_fwd_pw != 0);
    return m;
  endfunction

  localparam logic [31: 0] FWD_MASK = fwd_mask_build();

  //! instructions that may sit between the RF slot and retirement
  localparam int  N_INFLIGHT = p_stage_EX + p_stage_MA + p_stage_WB;
  localparam int  W_INFLIGHT = $clog2(N_INFLIGHT + 1);

  // regfile
  wire          rf_busy_RF;               //! regfile is busy
  wire          rf_addr_oob_RF;           //! regfile address out of bounds //TODO: handle this exception

  /*******************************************************
    Pipeline control (see `cpu_hazard`)

    `en_<X>`    : the barriers of group X load a new payload this cycle
    `flush_<X>` : the head barrier of group X loads a bubble instead
    `squash_<X>`: every barrier of group X loads a bubble (wrong path)
    `valid_<X>` : slot X holds an instruction that must be executed

    A **group** is a run of `p_stage_<X>` identical barriers; the **slot** is
    its output, the point where a functional unit observes the instruction. A
    group of depth 0 makes its slot a combinational alias of the slot upstream,
    depth 1 is a plain pipeline stage, and depth N spreads the group over N
    cycles at unchanged throughput.

    Adding a new slot is adding a group: declare its chain array, copy one of
    the `g_barrier_*` generate blocks, give the hazard unit its `en`/`flush`,
    and -- if the slot can hold a not-yet-committed register write -- add its
    barriers to the forwarding entry list below. Nothing else has to be
    renumbered, because no logic here counts slots by hand.

    Every architectural side effect below -- register write, memory write,
    redirection, retirement -- is gated by the `valid` bit of the slot that
    produces it, so a wrong-path instruction is invisible whatever its payload.
  *******************************************************/

  logic         en_IF;
  logic         en_IC;
  logic         en_ID;
  logic         en_RF;
  logic         en_EX;
  logic         en_MA;
  logic         en_WB;

  logic         kill_IF;                  //! the fetch output is wrong-path
  logic         flush_IF;
  logic         flush_IC;
  logic         squash_IC;                //! every IC barrier holds a wrong-path instruction
  logic         squash_ID;                //! every ID barrier holds a wrong-path instruction
  logic         squash_EX;                //! every EX barrier holds a wrong-path instruction
  logic         flush_ID;
  logic         flush_RF;
  logic         flush_EX;
  logic         flush_MA;
  logic         flush_WB;

  logic         valid_IF;
  logic         valid_IC;
  logic         valid_ID;
  logic         valid_RF;
  logic         valid_EX;
  logic         valid_MA;
  logic         valid_WB;

  logic         redirect;                 //! a control transfer resolved this cycle
  logic         redirect_early;           //! the target comes from the decoder (`jal`)
  logic [31: 0] redirect_pc;              //! resolved target
  logic         redirect_fetch;           //! redirection seen by the fetch stage
  logic [31: 0] redirect_fetch_pc;        //! redirection target seen by the fetch stage

  //! forwarding network: one entry per in-flight register write, youngest
  //! first. `fwd_*` describes the entries, `fwd_rs1`/`fwd_rs2` are the one-hot
  //! decisions returned by `cpu_hazard`.
  logic [N_FWD-1  : 0] fwd_valid;         //! the entry holds a real instruction
  logic [N_FWD-1  : 0] fwd_en_wb;         //! the entry writes a register
  logic [N_FWD*5-1: 0] fwd_addr;          //! destination register of each entry
  logic [N_FWD-1  : 0] fwd_ready;         //! the entry already holds its result
  logic [31: 0]        fwd_data [N_FWD];  //! value each entry would forward
  logic [N_FWD-1  : 0] fwd_rs1;           //! forward rs1 from this entry
  logic [N_FWD-1  : 0] fwd_rs2;           //! forward rs2 from this entry

  //! per-group summary of the decisions above, kept for the simulation log
  logic         bp_rs1_ex;                //! bypass rs1 from an EX barrier
  logic         bp_rs1_ma;                //! bypass rs1 from an MA barrier
  logic         bp_rs1_wb;                //! bypass rs1 from a WB barrier
  logic         bp_rs1_pw;                //! bypass rs1 from the pending write
  logic         bp_rs2_ex;                //! bypass rs2 from an EX barrier
  logic         bp_rs2_ma;                //! bypass rs2 from an MA barrier
  logic         bp_rs2_wb;                //! bypass rs2 from a WB barrier
  logic         bp_rs2_pw;                //! bypass rs2 from the pending write

  logic         stall_rs1;                //! rs1 causes a load-use stall
  logic         stall_rs2;                //! rs2 causes a load-use stall

  logic [63: 0] minstret;                 //! instructions retired so far
  logic [W_INFLIGHT-1: 0] minstret_inflight; //! valid instructions between RF and retirement
  logic [63: 0] minstret_RF;              //! instruction index seen by the RF slot

  // IF (Instruction Fetch) stage
  logic [31: 0] pc;                       //! program counter of the fetched word
  logic [31: 0] pc_inc;                   //! program counter +4
  isa_instr_t   instr;                    //! raw instruction from instruction memory
  logic         fetch_valid;              //! the fetch stage holds a fetched word

  logic [31: 0] pc_F;                     //! program counter (raw fetch output)
  logic [31: 0] pc_inc_F;                 //! program counter +4 (raw fetch output)
  isa_instr_t   instr_F;                  //! raw instruction (wrong path squashed)
  logic         valid_F;                  //! the raw fetch output is on the right path

  logic [31: 0] pc_IF;                    //! program counter
  logic [31: 0] pc_inc_IF;                //! program counter +4
  isa_instr_t   instr_IF;                 //! raw instruction

  // IC (Instruction deCompress) stage
  isa_instr_t   instr_decomp;             //! decompressed instruction
  logic [31: 0] pc_IC;                    //! program counter
  logic [31: 0] pc_inc_IC;                //! program counter +4
  isa_instr_t   instr_decomp_IC;          //! decompressed instruction

  /*******************************************************
    Stage barrier payloads

    Each barrier carries one packed bundle (see `pck_pipe`) instead of ~35
    individual signals. The per-signal names below are kept as aliases so that
    the rest of the core, the hazard unit and the debug hooks are unchanged.
  *******************************************************/

  pipe_id_t     ctrl_ID,  ctrl_ID_n;      //! ID barrier payload and its next value
  pipe_id_t     ctrl_RF,  ctrl_RF_n;      //! RF barrier payload and its next value
  pipe_ex_t     ctrl_EX,  ctrl_EX_n;      //! EX barrier payload and its next value
  pipe_ma_t     ctrl_MA,  ctrl_MA_n;      //! MA barrier payload and its next value
  pipe_wb_t     ctrl_WB,  ctrl_WB_n;      //! WB barrier payload and its next value

  //! Every barrier of a group, oldest last: `ctrl_<X>` is the slot itself, that
  //! is the last element. The intermediate elements are what a deep group adds,
  //! and the forwarding network reads them exactly like the slot.
  pipe_ex_t     ctrl_EX_c [D_EX];         //! EX barrier chain
  pipe_ma_t     ctrl_MA_c [D_MA];         //! MA barrier chain
  pipe_wb_t     ctrl_WB_c [D_WB];         //! WB barrier chain
  logic         valid_EX_c[D_EX];         //! occupancy of each EX barrier
  logic         valid_MA_c[D_MA];         //! occupancy of each MA barrier
  logic         valid_WB_c[D_WB];         //! occupancy of each WB barrier

  //! the data memory result is already registered when it reaches the first MA
  //! barrier, so it is re-registered only by the barriers that follow it
  logic [31: 0] dmem_data_MA [D_MA];      //! load data seen by each MA barrier

  logic [ 4: 0] wb_addr_PW;               //! destination of the pending write
  logic         en_wb_PW;                 //! the pending write is a real write

  // ID (Instruction Decode) stage
  wire [31: 0] pc_ID           = ctrl_ID.com.pc;
  wire [31: 0] pc_inc_ID       = ctrl_ID.com.pc_inc;
  wire [31: 0] imm_ID          = ctrl_ID.com.imm;
  isa_instr_e  instr_name_ID;  assign instr_name_ID   = ctrl_ID.com.instr_name;
  isa_instr_t  instr_decomp_ID; assign instr_decomp_ID = ctrl_ID.com.instr_decomp;
  wire [ 4: 0] rf_rd1_addr_ID  = ctrl_ID.rs.a.rd1_addr;
  wire [ 4: 0] rf_rd2_addr_ID  = ctrl_ID.rs.a.rd2_addr;
  wire         rf_rd2_used_ID  = ctrl_ID.rs.a.rd2_used;
  wire         stall_rs1_ID    = ctrl_ID.rs.stall_rs1;
  wire         stall_rs2_ID    = ctrl_ID.rs.stall_rs2;
  wire [ 4: 0] wb_addr_ID      = ctrl_ID.wbc.wb_addr;
  sel_wb_e     sel_wb_ID;      assign sel_wb_ID       = ctrl_ID.wbc.sel_wb;
  wire         en_wb_ID        = ctrl_ID.wbc.en_wb;
  sel_br_e     sel_br_ID;      assign sel_br_ID       = ctrl_ID.sel_br;
  wire         cond_branch_ID  = ctrl_ID.cond_branch;
  wire         jump_reg_ID     = ctrl_ID.jump_reg;

  // combinational decode outputs (unregistered)
  isa_instr_e   instr_name;               //! instruction name (debug)
  logic [31: 0] imm;                      //! immediate value
  logic [ 4: 0] wb_addr;                  //! write back addres
  logic [ 4: 0] rf_rd1_addr;              //! regfile read address for port 1
  logic [ 4: 0] rf_rd2_addr;              //! regfile read address for port 2
  logic         rf_rd2_used;              //! register file read port 2 is used
  sel_br_e      sel_br;                   //! program counter mux selector
  sel_alu_op_e  sel_alu_op;               //! ALU operation selector
  sel_alu_opa_e sel_alu_opa;              //! ALU operand A selector
  sel_alu_opb_e sel_alu_opb;              //! ALU operand B selector
  sel_md_op_e   sel_md_op;                //! MULDIV operation selector
  sel_csr_wr_e  sel_csr_wr;               //! csr write data selector
  sel_csr_op_e  sel_csr_op;               //! csr write operation selector
  logic         opa_signed;               //! MULDIV operand A is signed
  logic         opb_signed;               //! MULDIV operand B is signed
  logic         wen_csr;                  //! csr write enable
  logic         ren_csr;                  //! csr read enable
  logic [11: 0] csr_addr;                 //! csr address
  sel_be_e      sel_dmem_be;              //! dmem address mode (word, byte, halfbyte)
  logic         dmem_sext;                //! sign extension on non-word dmem data
  logic         en_dmem_wr;               //! data memory write enable
  logic         en_dmem_rd;               //! data memory read enable
  logic         swap_bytes;               //!
  sel_wb_e      sel_wb;                   //! write back source selector
  logic         en_wb;                    //! register file write enable
  logic         cond_branch;              //! conditionnal branch
  wire  [31: 0] imm_bp;                   //! immediate value (unregistered)
  wire          br_instr_bp;              //! conditionnal branch (inconditionnal branches jal and jalr are excluded) (unregistered)
  wire          cond_br_bp;               //! branch instruction (unregistered)
  wire          jalr_instr_bp;            //! jalr instruction (unregistered)
  wire          predict_taken;            //! the IC slot instruction is predicted taken (unregistered)
  wire          predict_IC;               //! prediction offered to the hazard unit
  wire          predicted_RF;             //! the RF slot instruction was predicted taken
  logic         jump_reg;                 //! jump register
  logic [31: 0] csr_rd_data;

  // RF (Register File) stage
  wire [31: 0] pc_RF           = ctrl_RF.com.pc;
  wire [31: 0] pc_inc_RF       = ctrl_RF.com.pc_inc;
  wire [31: 0] imm_RF          = ctrl_RF.com.imm;
  isa_instr_e  instr_name_RF;  assign instr_name_RF   = ctrl_RF.com.instr_name;
  isa_instr_t  instr_decomp_RF; assign instr_decomp_RF = ctrl_RF.com.instr_decomp;
  wire [ 4: 0] rf_rd1_addr_RF  = ctrl_RF.rs.a.rd1_addr;
  wire [ 4: 0] rf_rd2_addr_RF  = ctrl_RF.rs.a.rd2_addr;
  wire         rf_rd2_used_RF  = ctrl_RF.rs.a.rd2_used;
  wire         stall_rs1_RF    = ctrl_RF.rs.stall_rs1;
  wire         stall_rs2_RF    = ctrl_RF.rs.stall_rs2;
  wire [ 4: 0] wb_addr_RF      = ctrl_RF.wbc.wb_addr;
  sel_wb_e     sel_wb_RF;      assign sel_wb_RF       = ctrl_RF.wbc.sel_wb;
  wire         en_wb_RF        = ctrl_RF.wbc.en_wb;
  sel_alu_op_e sel_alu_op_RF;  assign sel_alu_op_RF   = ctrl_RF.dec.sel_alu_op;
  sel_alu_opa_e sel_alu_opa_RF; assign sel_alu_opa_RF = ctrl_RF.dec.sel_alu_opa;
  sel_alu_opb_e sel_alu_opb_RF; assign sel_alu_opb_RF = ctrl_RF.dec.sel_alu_opb;
  sel_md_op_e  sel_md_op_RF;   assign sel_md_op_RF    = ctrl_RF.dec.sel_md_op;
  sel_csr_wr_e sel_csr_wr_RF;  assign sel_csr_wr_RF   = ctrl_RF.dec.sel_csr_wr;
  sel_csr_op_e sel_csr_op_RF;  assign sel_csr_op_RF   = ctrl_RF.dec.sel_csr_op;
  wire         opa_signed_RF   = ctrl_RF.dec.opa_signed;
  wire         opb_signed_RF   = ctrl_RF.dec.opb_signed;
  wire [11: 0] csr_addr_RF     = ctrl_RF.dec.csr_addr;
  sel_be_e     sel_dmem_be_RF; assign sel_dmem_be_RF  = ctrl_RF.mem.sel_dmem_be;
  wire         dmem_sext_RF    = ctrl_RF.mem.dmem_sext;
  wire         en_dmem_wr_RF   = ctrl_RF.mem.en_dmem_wr;
  wire         en_dmem_rd_RF   = ctrl_RF.mem.en_dmem_rd;
  wire         swap_bytes_RF   = ctrl_RF.mem.swap_bytes;
  sel_br_e     sel_br_RF;      assign sel_br_RF       = ctrl_RF.sel_br;

  logic [31: 0] rf_rd1_data;              //! regfile read data for port 1
  logic [31: 0] rf_rd2_data;              //! regfile read data for port 2
  logic [31: 0] rf_rd1_data_RF;           //! regfile read data for port 1 (bypassed)
  logic [31: 0] rf_rd2_data_RF;           //! regfile read data for port 2 (bypassed)
  logic [31: 0] rf_rd1_data_bp_RF;        //! regfile read data for port 1
  logic [31: 0] rf_rd2_data_bp_RF;        //! regfile read data for port 2

  // EX (EXecusion) stage
  wire [31: 0] pc_EX           = ctrl_EX.com.pc;
  wire [31: 0] pc_inc_EX       = ctrl_EX.com.pc_inc;
  wire [31: 0] imm_EX          = ctrl_EX.com.imm;
  isa_instr_e  instr_name_EX;  assign instr_name_EX   = ctrl_EX.com.instr_name;
  isa_instr_t  instr_decomp_EX; assign instr_decomp_EX = ctrl_EX.com.instr_decomp;
  wire [ 4: 0] rf_rd1_addr_EX  = ctrl_EX.rs.a.rd1_addr;
  wire [ 4: 0] rf_rd2_addr_EX  = ctrl_EX.rs.a.rd2_addr;
  wire         rf_rd2_used_EX  = ctrl_EX.rs.a.rd2_used;
  wire         stall_rs1_EX    = ctrl_EX.rs.stall_rs1;
  wire         stall_rs2_EX    = ctrl_EX.rs.stall_rs2;
  wire [ 4: 0] wb_addr_EX      = ctrl_EX.wbc.wb_addr;
  sel_wb_e     sel_wb_EX;      assign sel_wb_EX       = ctrl_EX.wbc.sel_wb;
  wire         en_wb_EX        = ctrl_EX.wbc.en_wb;
  sel_be_e     sel_dmem_be_EX; assign sel_dmem_be_EX  = ctrl_EX.mem.sel_dmem_be;
  wire         dmem_sext_EX    = ctrl_EX.mem.dmem_sext;
  wire         en_dmem_wr_EX   = ctrl_EX.mem.en_dmem_wr;
  wire         en_dmem_rd_EX   = ctrl_EX.mem.en_dmem_rd;
  wire         swap_bytes_EX   = ctrl_EX.mem.swap_bytes;
  wire [31: 0] rf_rd1_data_EX  = ctrl_EX.dat.rd1_data;
  wire [31: 0] rf_rd2_data_EX  = ctrl_EX.dat.rd2_data;
  wire [31: 0] alu_out_EX      = ctrl_EX.dat.alu_out;
  wire [31: 0] adder_out_EX    = ctrl_EX.dat.adder_out;
  wire [31: 0] muldiv_out_EX   = ctrl_EX.dat.muldiv_out;
  wire [31: 0] csr_rd_data_EX  = ctrl_EX.dat.csr_rd_data;
  wire [31: 0] copro_out0_EX   = ctrl_EX.dat.copro_out0;
  wire [31: 0] copro_out1_EX   = ctrl_EX.dat.copro_out1;
  wire [31: 0] copro_out2_EX   = ctrl_EX.dat.copro_out2;
  wire         exec_done_EX    = ctrl_EX.dat.exec_done;
  wire         branch_taken_EX = ctrl_EX.dat.branch_taken;
  //! control transfer verdict resolved at the RF slot and carried here, acted
  //! upon by `cpu_hazard` instead of the RF verdict when `p_branch_stage` is set
  wire         redirect_req_EX = ctrl_EX.dat.redirect_req;
  wire [31: 0] redirect_pc_EX  = ctrl_EX.dat.redirect_pc;

  //! a collapsed EX slot is a combinational alias of the RF slot, so a frozen
  //! RF slot keeps presenting the same access to the data memory every cycle.
  //! The access must then follow the enable of the slot that actually holds it.
  wire         en_ex_eff = (p_stage_EX != 0) ? en_EX : en_RF;

  //! A multi-cycle execute unit samples its operands on the cycle it starts --
  //! the divider tests its divisor against zero right there -- so it must not
  //! start before the RF slot really holds them. An operand still travelling
  //! down the pipeline would otherwise be latched as a wrong, possibly zero,
  //! divisor. Holding the unit off during a load-use stall costs nothing: the
  //! slot cannot move on anyway.
  wire         exec_start = en_EX & valid_RF & ~stall_rs1 & ~stall_rs2;
  sel_pc_e     sel_pc_EX;      assign sel_pc_EX       = ctrl_EX.dat.sel_pc;
  sel_br_e     sel_br_EX;      assign sel_br_EX       = ctrl_EX.sel_br;
  wire         bp_rs1_ex_EX    = ctrl_EX.bp.rs1_ex;
  wire         bp_rs1_ma_EX    = ctrl_EX.bp.rs1_ma;
  wire         bp_rs1_wb_EX    = ctrl_EX.bp.rs1_wb;
  wire         bp_rs2_ex_EX    = ctrl_EX.bp.rs2_ex;
  wire         bp_rs2_ma_EX    = ctrl_EX.bp.rs2_ma;
  wire         bp_rs2_wb_EX    = ctrl_EX.bp.rs2_wb;

  logic [31: 0] alu_out;                  //! ALU output
  logic [31: 0] adder_out;                //! adder output
  logic [31: 0] muldiv_out;               //! MUL/DIV result
  logic         exec_done;                //! execute stage done
  logic         branch_taken;             //! a branch is taken
  sel_pc_e      sel_pc;                   //! program counter select

  // MA (Memory Access) stage
  wire [31: 0] pc_MA           = ctrl_MA.com.pc;
  wire [31: 0] pc_inc_MA       = ctrl_MA.com.pc_inc;
  wire [31: 0] imm_MA          = ctrl_MA.com.imm;
  isa_instr_e  instr_name_MA;  assign instr_name_MA   = ctrl_MA.com.instr_name;
  isa_instr_t  instr_decomp_MA; assign instr_decomp_MA = ctrl_MA.com.instr_decomp;
  wire [ 4: 0] rf_rd1_addr_MA  = ctrl_MA.rs.a.rd1_addr;
  wire [ 4: 0] rf_rd2_addr_MA  = ctrl_MA.rs.a.rd2_addr;
  wire         rf_rd2_used_MA  = ctrl_MA.rs.a.rd2_used;
  wire         stall_rs1_MA    = ctrl_MA.rs.stall_rs1;
  wire         stall_rs2_MA    = ctrl_MA.rs.stall_rs2;
  wire [ 4: 0] wb_addr_MA      = ctrl_MA.wbc.wb_addr;
  sel_wb_e     sel_wb_MA;      assign sel_wb_MA       = ctrl_MA.wbc.sel_wb;
  wire         en_wb_MA        = ctrl_MA.wbc.en_wb;
  wire [31: 0] rf_rd1_data_MA  = ctrl_MA.dat.rd1_data;
  wire [31: 0] rf_rd2_data_MA  = ctrl_MA.dat.rd2_data;
  wire [31: 0] alu_out_MA      = ctrl_MA.dat.alu_out;
  wire [31: 0] muldiv_out_MA   = ctrl_MA.dat.muldiv_out;
  wire [31: 0] csr_rd_data_MA  = ctrl_MA.dat.csr_rd_data;
  wire [31: 0] copro_out0_MA   = ctrl_MA.dat.copro_out0;
  wire [31: 0] copro_out1_MA   = ctrl_MA.dat.copro_out1;
  wire [31: 0] copro_out2_MA   = ctrl_MA.dat.copro_out2;
  wire         branch_taken_MA = ctrl_MA.dat.branch_taken;
  sel_pc_e     sel_pc_MA;      assign sel_pc_MA       = ctrl_MA.dat.sel_pc;
  sel_br_e     sel_br_MA;      assign sel_br_MA       = ctrl_MA.sel_br;
  wire         bp_rs1_ex_MA    = ctrl_MA.bp.rs1_ex;
  wire         bp_rs1_ma_MA    = ctrl_MA.bp.rs1_ma;
  wire         bp_rs1_wb_MA    = ctrl_MA.bp.rs1_wb;
  wire         bp_rs2_ex_MA    = ctrl_MA.bp.rs2_ex;
  wire         bp_rs2_ma_MA    = ctrl_MA.bp.rs2_ma;
  wire         bp_rs2_wb_MA    = ctrl_MA.bp.rs2_wb;

  logic [31: 0] dbus_rd_data_MA;          //! data bus read data
  logic         dbus_busy_MA;             //! data bus busy
  logic         dbus_ack_MA;              //! data bus acknowledge

  // WB (Write Back) stage
  wire [31: 0] pc_WB           = ctrl_WB.com.pc;
  wire [31: 0] imm_WB          = ctrl_WB.com.imm;
  isa_instr_e  instr_name_WB;  assign instr_name_WB   = ctrl_WB.com.instr_name;
  isa_instr_t  instr_decomp_WB; assign instr_decomp_WB = ctrl_WB.com.instr_decomp;
  wire [ 4: 0] rf_rd1_addr_WB  = ctrl_WB.rs.a.rd1_addr;
  wire [ 4: 0] rf_rd2_addr_WB  = ctrl_WB.rs.a.rd2_addr;
  wire         rf_rd2_used_WB  = ctrl_WB.rs.a.rd2_used;
  wire         stall_rs1_WB    = ctrl_WB.rs.stall_rs1;
  wire         stall_rs2_WB    = ctrl_WB.rs.stall_rs2;
  wire [31: 0] imm_WB_         = ctrl_WB.imm_raw;
  wire [31: 0] rf_rd1_data_WB_ = ctrl_WB.rd1_data_raw;
  wire [31: 0] rf_rd1_data_WB  = ctrl_WB.rd1_data;
  wire [31: 0] rf_rd2_data_WB  = ctrl_WB.rd2_data;
  wire [31: 0] alu_out_WB      = ctrl_WB.alu_out;
  wire         branch_taken_WB = ctrl_WB.branch_taken;
  sel_wb_e     sel_wb_WB;      assign sel_wb_WB       = ctrl_WB.sel_wb;
  wire [ 4: 0] rf_wr_addr_WB   = ctrl_WB.rf_wr_addr;
  wire [31: 0] rf_wr_data_WB   = ctrl_WB.rf_wr_data;
  wire         rf_wr_en_WB     = ctrl_WB.rf_wr_en & valid_WB;
  wire         bp_rs1_ex_WB    = ctrl_WB.bp.rs1_ex;
  wire         bp_rs1_ma_WB    = ctrl_WB.bp.rs1_ma;
  wire         bp_rs1_wb_WB    = ctrl_WB.bp.rs1_wb;
  wire         bp_rs2_ex_WB    = ctrl_WB.bp.rs2_ex;
  wire         bp_rs2_ma_WB    = ctrl_WB.bp.rs2_ma;
  wire         bp_rs2_wb_WB    = ctrl_WB.bp.rs2_wb;

  logic [31: 0] rf_wr_data_PW;            //! regfile write data (post write back)
  logic [31: 0] rf_wr_data;               //! regfile write data
  logic [ 4: 0] rf_wr_addr;               //! regfile write address
  logic         rf_wr_en;                 //! regfile write enable



  /*******************************************************
    Debug hooks used by the simulation log

    The trace is taken at the WB slot, gated by `valid_WB`: one line per retired
    instruction, tagged with the retired count itself, so the log is a faithful
    architectural trace whatever the pipeline shape.
  *******************************************************/

  wire  [31: 0] debug_imm        = imm_WB_;
  wire  [31: 0] debug_rs1_addr   = rf_rd1_addr_WB;
  wire  [31: 0] debug_rs1_data   = rf_rd1_data_WB_;
  wire          debug_rs2_used   = rf_rd2_used_WB;
  wire  [31: 0] debug_rs2_addr   = rf_rd2_addr_WB;
  wire  [31: 0] debug_rs2_data   = rf_rd2_data_WB;
  wire  [31: 0] debug_pc         = pc_WB;
  wire  [63: 0] debug_instret    = minstret;
  wire          debug_valid      = valid_WB;
  wire  [31: 0] debug_wb_addr    = rf_wr_addr_WB;
  wire  [31: 0] debug_wb_data    = rf_wr_data_WB;
  wire          debug_wb_en      = rf_wr_en_WB;
  wire          debug_br_taken   = branch_taken_WB;
  isa_instr_e   debug_instr_name;
  assign debug_instr_name = instr_name_WB;
  wire  [31: 0] debug_instr_code = instr_decomp_WB.code;
  wire          debug_bp_rs1_ex  = bp_rs1_ex_WB;
  wire          debug_bp_rs1_ma  = bp_rs1_ma_WB;
  wire          debug_bp_rs1_wb  = bp_rs1_wb_WB;
  wire          debug_bp_rs2_ex  = bp_rs2_ex_WB;
  wire          debug_bp_rs2_ma  = bp_rs2_ma_WB;
  wire          debug_bp_rs2_wb  = bp_rs2_wb_WB;
  wire          debug_stall_rs1  = stall_rs1_WB;
  wire          debug_stall_rs2  = stall_rs2_WB;
  cpu_hazard #(
    .p_stage_IF         ( p_stage_IF            ),
    .p_stage_IC         ( p_stage_IC            ),
    .p_stage_ID         ( p_stage_ID            ),
    .p_stage_RF         ( p_stage_RF            ),
    .p_stage_EX         ( p_stage_EX            ),
    .p_stage_MA         ( p_stage_MA            ),
    .p_stage_WB         ( p_stage_WB            ),
    .p_early_jal        ( p_early_jal           ),
    .p_branch_pred      ( p_branch_pred         ),
    .p_redirect_buf     ( p_redirect_buf        ),
    .p_branch_stage     ( p_branch_stage        ),
    .p_n_fwd            ( N_FWD                 ),
    .p_fwd_mask         ( FWD_MASK              )
  ) hazard_unit (
    .i_clk              ( i_clk                 ),
    .i_rst              ( i_rst                 ),

    .i_valid_IC         ( valid_IC              ),
    .i_valid_RF         ( valid_RF              ),

    .i_valid_EX         ( valid_EX              ),
    .i_predict_IC       ( predict_IC            ),
    .i_predicted_RF     ( predicted_RF          ),

    .i_rd1_addr_RF      ( rf_rd1_addr_RF        ),
    .i_rd2_addr_RF      ( rf_rd2_addr_RF        ),
    .i_rd2_used_RF      ( rf_rd2_used_RF        ),

    .i_fwd_valid        ( fwd_valid             ),
    .i_fwd_en_wb        ( fwd_en_wb             ),
    .i_fwd_addr         ( fwd_addr              ),
    .i_fwd_ready        ( fwd_ready             ),

    .i_sel_br_IC        ( sel_br                ),
    .i_sel_br_RF        ( sel_br_RF             ),
    .i_branch_taken_RF  ( branch_taken          ),
    .i_redirect_req_EX  ( redirect_req_EX       ),
    .i_exec_done_RF     ( exec_done             ),

    .o_en_IF            ( en_IF                 ),
    .o_en_IC            ( en_IC                 ),
    .o_en_ID            ( en_ID                 ),
    .o_en_RF            ( en_RF                 ),
    .o_en_EX            ( en_EX                 ),
    .o_en_MA            ( en_MA                 ),
    .o_en_WB            ( en_WB                 ),
    .o_kill_IF          ( kill_IF               ),
    .o_flush_IF         ( flush_IF              ),
    .o_flush_IC         ( flush_IC              ),
    .o_flush_ID         ( flush_ID              ),
    .o_flush_RF         ( flush_RF              ),
    .o_flush_EX         ( flush_EX              ),
    .o_flush_MA         ( flush_MA              ),
    .o_flush_WB         ( flush_WB              ),
    .o_squash_IC        ( squash_IC             ),
    .o_squash_ID        ( squash_ID             ),
    .o_squash_EX        ( squash_EX             ),

    .o_redirect         ( redirect              ),
    .o_redirect_early   ( redirect_early        ),

    .o_fwd_rs1          ( fwd_rs1               ),
    .o_fwd_rs2          ( fwd_rs2               ),

    .o_stall_rs1        ( stall_rs1             ),
    .o_stall_rs2        ( stall_rs2             )
  );

  //! synchronous fetch stage
  `KEEP_HIERARCHY
  cpu_fetch_pipe #(
    .p_reset_vector ( p_reset_vector        )
  ) fetch_stage (   
    .i_clk          ( i_clk                 ),
    .i_rst          ( i_rst                 ),
    .i_sleep        ( i_sleep               ),
    .ibus_addr      ( ibus_addr             ),
    .ibus_be        ( ibus_be               ),
    .ibus_wr_en     ( ibus_wr_en            ),
    .ibus_wr_data   ( ibus_wr_data          ),
    .ibus_rd_en     ( ibus_rd_en            ),
    .ibus_rd_data   ( ibus_rd_data          ),
    .ibus_busy      ( ibus_busy             ),
    .ibus_ack       ( ibus_ack              ),
    .i_en           ( en_IF                 ),
    .i_redirect     ( redirect_fetch        ),
    .i_redirect_pc  ( redirect_fetch_pc     ),
    .o_pc           ( pc                    ),
    .o_pc_inc       ( pc_inc                ),
    .o_instr        ( instr                 ),
    .o_valid        ( fetch_valid           )
  );   

  /*******************************************************
    Redirection

    A control transfer is resolved either by the execute unit, which sees the
    RF slot, or -- for `jal`, whose target needs no register value -- straight
    out of the decoder. `p_redirect_buf` inserts a register between the resolved
    target and the fetch stage: it trades one extra wrong-path fetch (accounted
    for by `cpu_hazard`) for a shorter combinational path.
  *******************************************************/

  //! with `p_branch_pred` the RF slot also redirects when a branch predicted
  //! taken turns out not to be: the recovery target is then the sequential
  //! address, which is where the front-end should have gone in the first place
  wire  [31: 0] redirect_pc_taken = (sel_pc == pc_alu) ? (alu_out & 32'hfffffffe)
                                                       : (pc_RF + $signed(imm_RF));
  wire  [31: 0] redirect_pc_RF = ((p_branch_pred != 0) & ~branch_taken) ? pc_inc_RF
                                                                       : redirect_pc_taken;
  wire  [31: 0] redirect_pc_IC = pc_IC + $signed(imm);

  //! The verdict itself, computed where the operands are: the execute unit sits
  //! at the RF slot and nothing can resolve a conditional branch earlier. What
  //! `p_branch_stage` moves is not this computation but the moment it is acted
  //! upon, so the request is built here in both configurations and simply
  //! registered into the EX barrier when the late slot is selected.
  //!
  //! The validity and freeze qualifications are deliberately left out: they
  //! belong to the slot the request is consumed in, and `cpu_hazard` applies
  //! them there.
  wire          mispredict_RF_c = (p_branch_pred != 0) ? (branch_taken ^ predicted_RF)
                                                       :  branch_taken;
  wire          redirect_req_RF = mispredict_RF_c
                                & ~((p_early_jal != 0) & (sel_br_RF == br_jal));

  assign redirect_pc = redirect_early        ? redirect_pc_IC
                     : (p_branch_stage != 0) ? redirect_pc_EX
                     :                         redirect_pc_RF;

  generate
    if (p_redirect_buf) begin : g_redirect_buf
      //! the request is held until the fetch stage can actually take it: a
      //! stall may freeze the front-end during the cycle it would otherwise be
      //! delivered, and dropping it there would let the jump vanish
      always_ff @(posedge i_clk) begin
        if (i_rst) begin
          redirect_fetch    <= 1'b0;
          redirect_fetch_pc <= 32'd0;
        end else if (redirect) begin
          redirect_fetch    <= 1'b1;
          redirect_fetch_pc <= redirect_pc;
        end else if (en_IF) begin
          redirect_fetch    <= 1'b0;
        end
      end
    end else begin : g_redirect_buf
      always_comb begin
        redirect_fetch    = redirect;
        redirect_fetch_pc = redirect_pc;
      end
    end
  endgenerate

  /*******************************************************
    IF slot

    The fetch output is replaced by a bubble while it is wrong-path, so that
    nothing downstream ever has to know that a redirection happened.
  *******************************************************/

  always_comb begin
    pc_F     = pc;
    pc_inc_F = pc_inc;
    if (kill_IF) begin
      valid_F      = 1'b0;
      instr_F.code = pck_isa_i::NOP;
    end else begin
      valid_F      = fetch_valid;
      instr_F      = instr;
    end
  end

  //! extra fetch barriers (`p_stage_IF` > 1): plain buffers between the
  //! instruction memory output and the decompressor, which shorten the fetch
  //! path without changing what the front-end owes after a redirection -- the
  //! wrong-path words they hold are killed by `flush_IF` like any other
  //! front-end barrier.
  generate
    if (N_IF == 0) begin : g_barrier_IF
      always_comb begin
        valid_IF  = valid_F;
        pc_IF     = pc_F;
        pc_inc_IF = pc_inc_F;
        instr_IF  = instr_F;
      end
    end else begin : g_barrier_IF
      logic         v_c  [D_IF];
      logic [31: 0] pc_c [D_IF];
      logic [31: 0] pi_c [D_IF];
      isa_instr_t   in_c [D_IF];
      for (genvar k = 0; k < N_IF; k++) begin : g_lvl
        always_ff @(posedge i_clk) begin
          if (i_rst) begin
            v_c [k]      <= 1'b0;
            pc_c[k]      <= p_reset_vector - 32'd4;
            pi_c[k]      <= 32'd0;
            in_c[k].code <= pck_isa_i::NOP;
          end else if (flush_IF) begin
            v_c [k]      <= 1'b0;
            in_c[k].code <= pck_isa_i::NOP;
          end else if (en_IF) begin
            v_c [k]      <= (k == 0) ? valid_F   : v_c [k-1];
            pc_c[k]      <= (k == 0) ? pc_F      : pc_c[k-1];
            pi_c[k]      <= (k == 0) ? pc_inc_F  : pi_c[k-1];
            in_c[k]      <= (k == 0) ? instr_F   : in_c[k-1];
          end
        end
      end
      assign valid_IF  = v_c [N_IF-1];
      assign pc_IF     = pc_c[N_IF-1];
      assign pc_inc_IF = pi_c[N_IF-1];
      assign instr_IF  = in_c[N_IF-1];
    end
  endgenerate

  cpu_decompressor #(   
    .p_ext_rvc      ( p_ext_rvc             )
  ) decompression_stage (    
    .i_clk          ( i_clk                 ),
    .i_rst          ( i_rst                 ),
    .i_en_decomp    ( en_IC                 ),
    .i_update_comp  ( en_IC                 ),
    .i_bypass_decomp(                       ),
    .i_branch_taken ( 1'b0                  ),
    .i_half_pc_addr (                       ),
    .i_half_npc_addr(                       ),
    .i_instr        ( instr_IF              ),
    .o_instr        ( instr_decomp          ),
    .o_offset_pc    (                       ),
    .o_compressed   (                       ),
    .o_freeze_pc    (                       ),
    .o_refetch      (                       )
  );   

  //! IC barriers
  generate
    if (p_stage_IC == 0) begin : g_barrier_IC
      //! no barrier: the IC slot is a combinational alias of the IF slot, and
      //! `flush_IC` has nothing to act on -- whatever had to be killed was
      //! killed upstream, at the fetch output
      always_comb begin
        valid_IC        = valid_IF;
        pc_IC           = pc_IF;
        pc_inc_IC       = pc_inc_IF;
        instr_decomp_IC = instr_decomp;
      end
    end else begin : g_barrier_IC
      logic         v_c  [D_IC];
      logic [31: 0] pc_c [D_IC];
      logic [31: 0] pi_c [D_IC];
      isa_instr_t   in_c [D_IC];
      for (genvar k = 0; k < p_stage_IC; k++) begin : g_lvl
        always_ff @(posedge i_clk) begin
          if (i_rst) begin
            v_c [k]      <= 1'b0;
            pc_c[k]      <= p_reset_vector - 32'd4;
            pi_c[k]      <= 32'd0;
            in_c[k].code <= pck_isa_i::NOP;
          end else if ((k == 0) ? flush_IC : squash_IC) begin
            v_c [k]      <= 1'b0;
            in_c[k].code <= pck_isa_i::NOP;
          end else if (en_IC) begin
            v_c [k]      <= (k == 0) ? valid_IF     : v_c [k-1];
            pc_c[k]      <= (k == 0) ? pc_IF        : pc_c[k-1];
            pi_c[k]      <= (k == 0) ? pc_inc_IF    : pi_c[k-1];
            in_c[k]      <= (k == 0) ? instr_decomp : in_c[k-1];
          end
        end
      end
      assign valid_IC        = v_c [p_stage_IC-1];
      assign pc_IC           = pc_c[p_stage_IC-1];
      assign pc_inc_IC       = pi_c[p_stage_IC-1];
      assign instr_decomp_IC = in_c[p_stage_IC-1];
    end
  endgenerate
  //! asynchronous decode stage
  `KEEP_HIERARCHY
  cpu_decode #(
    .p_ext_rvm      ( p_ext_rvm             ),
    .p_ext_rvzicsr  ( p_ext_rvzicsr         ),
    .p_ext_custom   ( p_ext_custom          ),
    .p_decode_buf   ( 0                     )
  ) decode_stage (
    .i_clk          ( i_clk                 ),
    .i_en_decode    ( 1'b1                  ),
    .i_instr        ( instr_decomp_IC       ),
    .o_instr_name   ( instr_name            ),
    .o_rf_wr_addr   ( wb_addr               ),
    .o_rf_rd1_addr  ( rf_rd1_addr           ),
    .o_rf_rd2_addr  ( rf_rd2_addr           ),
    .o_rf_rd2_used  ( rf_rd2_used           ),
    .o_csr_addr     ( csr_addr              ),
    .o_imm          ( imm                   ),
    .o_sel_br       ( sel_br                ),
    .o_sel_alu_op   ( sel_alu_op            ),
    .o_sel_alu_opa  ( sel_alu_opa           ),
    .o_sel_alu_opb  ( sel_alu_opb           ),
    .o_sel_md_op    ( sel_md_op             ),
    .o_sel_csr_wr   ( sel_csr_wr            ),
    .o_sel_csr_op   ( sel_csr_op            ),
    .o_opa_signed   ( opa_signed            ),
    .o_opb_signed   ( opb_signed            ),
    .o_sel_wb       ( sel_wb                ),
    .o_sel_dmem_be  ( sel_dmem_be           ),
    .o_dmem_sext    ( dmem_sext             ),
    .o_dmem_wr      ( en_dmem_wr            ),
    .o_dmem_rd      ( en_dmem_rd            ),
    .o_swap_bytes   ( swap_bytes            ),
    .o_en_wb        ( en_wb                 ),
    .o_wen_csr      ( wen_csr               ),
    .o_ren_csr      ( ren_csr               ),
    .o_cond_branch  ( cond_branch           ),
    .o_br_instr_bp  ( br_instr_bp           ),
    .o_cond_br_bp   ( cond_br_bp            ),
    .o_jalr_instr_bp( jalr_instr_bp         ),
    .o_imm_bp       ( imm_bp                ),
    .o_jump_reg     ( jump_reg              )
  );

  /*******************************************************
    Branch prediction (`p_branch_pred`)

    The predictor sits on the decoder outputs, at the IC slot, which is where
    `p_early_jal` already resolves `jal`: everything it needs -- the kind of
    control transfer and the branch immediate -- is available there, and the
    predicted target is the same `pc_IC + imm` the early `jal` path computes.
    A prediction is therefore just an early redirection like any other, and the
    only thing the rest of the pipeline has to remember is the `predicted` bit
    riding with the instruction down to the RF slot, where `cpu_hazard` compares
    it to the verdict of the execute unit.
  *******************************************************/

  generate
    if (p_branch_pred == 1) begin : g_branch_pred  // static prediction
      cpu_static_branch_predictor branch_predictor (
        .i_cond_branch  ( cond_br_bp            ),
        .i_branch_instr ( br_instr_bp           ),
        .i_jalr_instr   ( jalr_instr_bp         ),
        .i_imm          ( imm_bp                ),
        .i_pc           ( pc_IC                 ),
        .i_pc_inc       ( pc_inc_IC             ),
        .o_predicted_pc (                       ),
        .o_predict_taken( predict_taken         )
      );
    end else begin : g_branch_pred
      assign predict_taken = 1'b0;
    end
  endgenerate

  assign predict_IC   = predict_taken;
  assign predicted_RF = ctrl_RF.predicted;

  //! bubble injected in the ID barrier at reset
  function automatic pipe_id_t id_reset();
    id_reset        = PIPE_ID_NOP;
    id_reset.com.pc = p_reset_vector - 32'd4;
  endfunction

  //! bubble injected in the ID barrier on a flush (the pc is kept for debug)
  function automatic pipe_id_t id_bubble(input pipe_id_t cur);
    id_bubble            = PIPE_ID_NOP;
    id_bubble.com.pc     = cur.com.pc;
    id_bubble.com.pc_inc = cur.com.pc_inc;
  endfunction

  //! where each ID payload field comes from
  always_comb begin
    ctrl_ID_n.com.pc           = pc_IC;
    ctrl_ID_n.com.pc_inc       = pc_inc_IC;
    ctrl_ID_n.com.imm          = imm;
    ctrl_ID_n.com.instr_name   = instr_name;
    ctrl_ID_n.com.instr_decomp = instr_decomp_IC;
    ctrl_ID_n.rs.a.rd1_addr    = rf_rd1_addr;
    ctrl_ID_n.rs.a.rd2_addr    = rf_rd2_addr;
    ctrl_ID_n.rs.a.rd2_used    = rf_rd2_used;
    ctrl_ID_n.rs.stall_rs1     = stall_rs1;
    ctrl_ID_n.rs.stall_rs2     = stall_rs2;
    ctrl_ID_n.wbc.wb_addr      = wb_addr;
    ctrl_ID_n.wbc.sel_wb       = sel_wb;
    ctrl_ID_n.wbc.en_wb        = en_wb;
    ctrl_ID_n.dec.sel_alu_op   = sel_alu_op;
    ctrl_ID_n.dec.sel_alu_opa  = sel_alu_opa;
    ctrl_ID_n.dec.sel_alu_opb  = sel_alu_opb;
    ctrl_ID_n.dec.sel_md_op    = sel_md_op;
    ctrl_ID_n.dec.sel_csr_wr   = sel_csr_wr;
    ctrl_ID_n.dec.sel_csr_op   = sel_csr_op;
    ctrl_ID_n.dec.opa_signed   = opa_signed;
    ctrl_ID_n.dec.opb_signed   = opb_signed;
    ctrl_ID_n.dec.csr_addr     = csr_addr;
    ctrl_ID_n.dec.wen_csr      = wen_csr;
    ctrl_ID_n.dec.ren_csr      = ren_csr;
    ctrl_ID_n.mem.sel_dmem_be  = sel_dmem_be;
    ctrl_ID_n.mem.dmem_sext    = dmem_sext;
    ctrl_ID_n.mem.en_dmem_wr   = en_dmem_wr;
    ctrl_ID_n.mem.en_dmem_rd   = en_dmem_rd;
    ctrl_ID_n.mem.swap_bytes   = swap_bytes;
    ctrl_ID_n.sel_br           = sel_br;
    ctrl_ID_n.cond_branch      = cond_branch;
    ctrl_ID_n.jump_reg         = jump_reg;
    ctrl_ID_n.predicted        = predict_IC;
  end

  generate
    if (p_stage_ID == 0) begin : g_barrier_ID
      //! no barrier: the ID slot is a combinational alias of the IC slot
      always_comb begin
        ctrl_ID  = ctrl_ID_n;
        valid_ID = valid_IC;
      end
    end else begin : g_barrier_ID
      pipe_id_t c   [D_ID];
      logic     v_c [D_ID];
      for (genvar k = 0; k < p_stage_ID; k++) begin : g_lvl
        always_ff @(posedge i_clk) begin
          if (i_rst) begin
            c  [k] <= id_reset();
            v_c[k] <= 1'b0;
          end else if ((k == 0) ? flush_ID : squash_ID) begin
            c  [k] <= id_bubble(c[k]);
            v_c[k] <= 1'b0;
          end else if (en_ID) begin
            c  [k] <= (k == 0) ? ctrl_ID_n : c  [k-1];
            v_c[k] <= (k == 0) ? valid_IC  : v_c[k-1];
          end
        end
      end
      assign ctrl_ID  = c  [p_stage_ID-1];
      assign valid_ID = v_c[p_stage_ID-1];
    end
  endgenerate

  /*******************************************************
    Register file read

    When `p_stage_RF` is set the read is registered, so the read port is the
    other half of the RF barrier and must freeze with it. Rather than gating the
    output register -- which would leave it holding a value that predates any
    write committed during the stall -- the *address* is held: the same
    registers are read again every cycle the RF slot cannot move, so the
    operands stay fresh whatever the length of the stall.
  *******************************************************/

  wire [ 4: 0] rf_rd1_addr_rd = en_RF ? rf_rd1_addr_ID : rf_rd1_addr_RF;
  wire [ 4: 0] rf_rd2_addr_rd = en_RF ? rf_rd2_addr_ID : rf_rd2_addr_RF;

  //! register file (array of processor registers)
  `KEEP_HIERARCHY
  cpu_regfile #(
    .p_ext_rve      ( p_ext_rve             ),
    .p_rf_read_buf  ( p_stage_RF            ),
    .p_rf_sp        ( 0                     )
  ) regfile (
    .i_clk          ( i_clk                 ),
    .i_rst          ( i_rst                 ),
    .o_busy         ( rf_busy_RF            ),
    .o_addr_oob     ( rf_addr_oob_RF        ),
    .i_rd1_en       ( 1'b1                  ),
    .i_rd1_addr     ( rf_rd1_addr_rd        ),
    .o_rd1_data     ( rf_rd1_data           ),
    .i_rd2_en       ( 1'b1                  ),
    .i_rd2_addr     ( rf_rd2_addr_rd        ),
    .o_rd2_data     ( rf_rd2_data           ),
    .i_wr_en        ( rf_wr_en_WB           ),
    .i_wr_addr      ( rf_wr_addr_WB         ),
    .i_wr_data      ( rf_wr_data_WB         ) 
  );


  //! bubble injected in the RF barrier at reset
  function automatic pipe_id_t rf_reset();
    rf_reset        = PIPE_ID_NOP;
    rf_reset.com.pc = p_reset_vector - 32'd4;
  endfunction

  //! bubble injected in the RF barrier on a flush (the pc is kept for debug)
  function automatic pipe_id_t rf_bubble(input pipe_id_t cur);
    rf_bubble            = PIPE_ID_NOP;
    rf_bubble.com.pc     = cur.com.pc;
    rf_bubble.com.pc_inc = cur.com.pc_inc;
  endfunction

  //! the RF barrier simply forwards the ID payload
  assign ctrl_RF_n = ctrl_ID;

  generate
    if (p_stage_RF) begin : g_barrier_RF
      always_ff @(posedge i_clk) begin
        if (i_rst) begin
          ctrl_RF  <= rf_reset();
          valid_RF <= 1'b0;
        end else if (flush_RF) begin
          ctrl_RF  <= rf_bubble(ctrl_RF);
          valid_RF <= 1'b0;
        end else if (en_RF) begin
          ctrl_RF  <= ctrl_RF_n;
          valid_RF <= valid_ID;
        end
      end
    end else begin : g_barrier_RF
      //! no barrier: the RF slot is a combinational alias of the ID slot
      always_comb begin
        ctrl_RF  = ctrl_RF_n;
        valid_RF = valid_ID;
      end
    end
  endgenerate

  //! apply the forwarding decision: `fwd_rs*` is one-hot, so the order in
  //! which the entries are scanned does not matter
  always_comb begin: operand_bypass
    rf_rd1_data_bp_RF = rf_rd1_data;
    rf_rd2_data_bp_RF = rf_rd2_data;
    for (int i = 0; i < N_FWD; i++) begin
      if (fwd_rs1[i]) rf_rd1_data_bp_RF = fwd_data[i];
      if (fwd_rs2[i]) rf_rd2_data_bp_RF = fwd_data[i];
    end
  end

  assign rf_rd1_data_RF = rf_rd1_data_bp_RF;
  assign rf_rd2_data_RF = rf_rd2_data_bp_RF;


  /*******************************************************
    Retired instruction counter
  *******************************************************/

  always_ff @(posedge i_clk) begin: retired_instructions
    if (i_rst) begin
      minstret <= 64'd0;
    end else if (valid_WB & en_WB) begin
      minstret <= minstret + 64'd1;
    end
  end

  //! one in-flight instruction per barrier downstream of the RF slot: an absent
  //! barrier means the two slots it separates hold the same instruction, so it
  //! must not be counted. Counting the barriers themselves rather than the
  //! slots is what makes this right for a group of any depth.
  always_comb begin: inflight_count
    minstret_inflight = '0;
    for (int k = 0; k < N_FWD_EX; k++)
      if (valid_EX_c[k]) minstret_inflight = minstret_inflight + 1'b1;
    for (int k = 0; k < N_FWD_MA; k++)
      if (valid_MA_c[k]) minstret_inflight = minstret_inflight + 1'b1;
    for (int k = 0; k < N_FWD_WB; k++)
      if (valid_WB_c[k]) minstret_inflight = minstret_inflight + 1'b1;
  end

  assign minstret_RF = minstret + 64'(minstret_inflight);

  // control and status registers
  `KEEP_HIERARCHY
  cpu_csr #(
    .p_ext_rvzicsr  ( p_ext_rvzicsr         )
  ) csr (
    .i_clk          ( i_clk                 ),
    .i_rst          ( i_rst                 ),
    .i_mcycle       ( i_mcycle              ), 
    .i_mtime        ( i_mtime               ),  
    .i_minstret     ( minstret_RF           ),
    .i_imm          ( imm_RF                ),
    .i_sel_csr_wr   ( sel_csr_wr_RF         ),
    .i_sel_csr_op   ( sel_csr_op_RF         ),
    .i_addr         ( csr_addr_RF           ),
    .i_wr_en        ( 1'b0                  ),  
    .i_rf_rd1_data  ( rf_rd1_data_RF        ),
    .o_rd_data      ( csr_rd_data           )
  );
  `KEEP_HIERARCHY
  cpu_exec #(
    .p_rf_read_buf  ( 0                     ),
    .p_branch_buf   ( 0                     ),
    .p_ext_rvm      ( p_ext_rvm             ),
    .p_mul_fast     ( p_mul_fast            ),
    .p_mul_1_cycle  ( p_mul_1_cycle         ),
    .p_alu_share_adder ( p_alu_share_adder  ),
    .p_alu_shift_bits  ( p_alu_shift_bits   )
  ) exec_stage (
    .i_clk          ( i_clk                 ),
    .i_rst          ( i_rst                 ),
    .i_en_exec      ( exec_start            ),
    .i_rf_rd1_data  ( rf_rd1_data_RF        ),
    .i_rf_rd2_data  ( rf_rd2_data_RF        ),
    .i_imm          ( imm_RF                ),
    .i_pc           ( pc_RF                 ),
    .i_dmem_rd_data ( 32'd0                 ),
    .i_sel_wb       ( sel_wb_RF             ),
    .i_sel_br       ( sel_br_RF             ),
    .i_sel_alu_op   ( sel_alu_op_RF         ),
    .i_sel_alu_opa  ( sel_alu_opa_RF        ),
    .i_sel_alu_opb  ( sel_alu_opb_RF        ),
    .i_sel_md_op    ( sel_md_op_RF          ),
    .i_opa_signed   ( opa_signed_RF         ),
    .i_opb_signed   ( opb_signed_RF         ),
    .o_alu_op_a     ( o_alu_op_a            ),
    .o_alu_op_b     ( o_alu_op_b            ),
    .o_alu_out      ( alu_out               ),
    .o_adder_out    ( adder_out             ),
    .o_muldiv_out   ( muldiv_out            ),
    .o_exec_done    ( exec_done             ),
    .o_branch_taken ( branch_taken          ),
    .o_sel_pc       ( sel_pc                ) 
  );

  //! bubble injected in the EX barrier at reset
  function automatic pipe_ex_t ex_reset();
    ex_reset        = PIPE_EX_RST;
    ex_reset.com.pc = p_reset_vector - 32'd4;
  endfunction

  //! bubble injected in the EX barrier on a flush (the pc is kept for debug)
  function automatic pipe_ex_t ex_bubble(input pipe_ex_t cur);
    ex_bubble            = PIPE_EX_NOP;
    ex_bubble.com.pc     = cur.com.pc;
    ex_bubble.com.pc_inc = cur.com.pc_inc;
  endfunction

  //! where each EX payload field comes from
  always_comb begin
    ctrl_EX_n.com              = ctrl_RF.com;
    ctrl_EX_n.rs               = ctrl_RF.rs;
    ctrl_EX_n.wbc              = ctrl_RF.wbc;
    ctrl_EX_n.mem              = ctrl_RF.mem;
    ctrl_EX_n.sel_br           = ctrl_RF.sel_br;
    ctrl_EX_n.dat.rd1_data     = rf_rd1_data_RF;
    ctrl_EX_n.dat.rd2_data     = rf_rd2_data_RF;
    ctrl_EX_n.dat.alu_out      = alu_out;
    ctrl_EX_n.dat.adder_out    = adder_out;
    ctrl_EX_n.dat.muldiv_out   = muldiv_out;
    ctrl_EX_n.dat.csr_rd_data  = csr_rd_data;
    ctrl_EX_n.dat.copro_out0   = i_copro_out0;
    ctrl_EX_n.dat.copro_out1   = i_copro_out1;
    ctrl_EX_n.dat.copro_out2   = i_copro_out2;
    ctrl_EX_n.dat.exec_done    = exec_done;
    ctrl_EX_n.dat.branch_taken = branch_taken;
    ctrl_EX_n.dat.sel_pc       = sel_pc;
    ctrl_EX_n.dat.redirect_req = redirect_req_RF;
    ctrl_EX_n.dat.redirect_pc  = redirect_pc_RF;
    ctrl_EX_n.bp.rs1_ex        = bp_rs1_ex;
    ctrl_EX_n.bp.rs1_ma        = bp_rs1_ma;
    ctrl_EX_n.bp.rs1_wb        = bp_rs1_wb;
    ctrl_EX_n.bp.rs2_ex        = bp_rs2_ex;
    ctrl_EX_n.bp.rs2_ma        = bp_rs2_ma;
    ctrl_EX_n.bp.rs2_wb        = bp_rs2_wb;
  end

  generate
    if (p_stage_EX == 0) begin : g_barrier_EX
      //! no barrier: the EX slot is a combinational alias of the RF slot, and
      //! it forwards nothing -- it *is* the consumer
      always_comb begin
        ctrl_EX     = ctrl_EX_n;
        valid_EX    = valid_RF;
        ctrl_EX_c[0]  = ctrl_EX_n;
        valid_EX_c[0] = valid_RF;
      end
    end else begin : g_barrier_EX
      for (genvar k = 0; k < p_stage_EX; k++) begin : g_lvl
        always_ff @(posedge i_clk) begin
          if (i_rst) begin
            ctrl_EX_c [k] <= ex_reset();
            valid_EX_c[k] <= 1'b0;
          end else if ((k == 0) ? flush_EX : squash_EX) begin
            ctrl_EX_c [k] <= ex_bubble(ctrl_EX_c[k]);
            valid_EX_c[k] <= 1'b0;
          end else if (en_EX) begin
            ctrl_EX_c [k] <= (k == 0) ? ctrl_EX_n : ctrl_EX_c [k-1];
            valid_EX_c[k] <= (k == 0) ? valid_RF  : valid_EX_c[k-1];
          end
        end
      end
      assign ctrl_EX  = ctrl_EX_c [p_stage_EX-1];
      assign valid_EX = valid_EX_c[p_stage_EX-1];
    end
  endgenerate

  //! data merory read/write stage
  `KEEP_HIERARCHY
  cpu_mem #(
    .p_mem_buf      ( 0                     )
  ) mem_stage (
    .i_clk          ( i_clk                 ),
    //.dbus           ( dbus                  ),
    .dbus_addr      ( dbus_addr             ),
    .dbus_be        ( dbus_be               ),
    .dbus_wr_en     ( dbus_wr_en            ),
    .dbus_wr_data   ( dbus_wr_data          ),
    .dbus_rd_en     ( dbus_rd_en            ),
    .dbus_rd_data   ( dbus_rd_data          ),
    .dbus_busy      ( dbus_busy             ),
    .dbus_ack       ( dbus_ack              ),
    .i_swap_bytes   ( swap_bytes_EX         ),
    .i_adder_out    ( adder_out_EX          ),
    .i_sel_dmem_be  ( sel_dmem_be_EX        ),
    .i_dmem_sext    ( dmem_sext_EX          ),
    .i_en_dmem_wr   ( en_dmem_wr_EX & valid_EX & en_ex_eff ),
    .i_rf_rd2_data  ( rf_rd2_data_EX        ),
    .i_en_dmem_rd   ( en_dmem_rd_EX & valid_EX & en_ex_eff ),
    .o_dbus_rd_data ( dbus_rd_data_MA       ),
    .o_dbus_busy    ( dbus_busy_MA          ),
    .o_dbus_ack     ( dbus_ack_MA           ) 
  );

  //! bubble injected in the MA barrier at reset
  function automatic pipe_ma_t ma_reset();
    ma_reset        = PIPE_MA_RST;
    ma_reset.com.pc = p_reset_vector - 32'd4;
  endfunction

  //! bubble injected in the MA barrier on a flush (the pc is kept for debug)
  function automatic pipe_ma_t ma_bubble(input pipe_ma_t cur);
    ma_bubble            = PIPE_MA_NOP;
    ma_bubble.com.pc     = cur.com.pc;
    ma_bubble.com.pc_inc = cur.com.pc_inc;
  endfunction

  //! the MA barrier forwards the EX payload as is
  assign ctrl_MA_n = '{
    com: ctrl_EX.com, rs: ctrl_EX.rs, wbc: ctrl_EX.wbc,
    dat: ctrl_EX.dat, bp: ctrl_EX.bp, sel_br: ctrl_EX.sel_br
  };

  //! `p_stage_MA` is at least 1: the data memory read is registered, so the
  //! load result reaches the first MA barrier already synchronous with it and
  //! is only re-registered by the barriers that follow.
  generate
    for (genvar k = 0; k < p_stage_MA; k++) begin : g_barrier_MA
      always_ff @(posedge i_clk) begin
        if (i_rst) begin
          ctrl_MA_c [k] <= ma_reset();
          valid_MA_c[k] <= 1'b0;
        end else if ((k == 0) & flush_MA) begin
          ctrl_MA_c [k] <= ma_bubble(ctrl_MA_c[k]);
          valid_MA_c[k] <= 1'b0;
        end else if (en_MA) begin
          ctrl_MA_c [k] <= (k == 0) ? ctrl_MA_n : ctrl_MA_c [k-1];
          valid_MA_c[k] <= (k == 0) ? valid_EX  : valid_MA_c[k-1];
        end
      end
      if (k == 0) begin : g_dmem
        assign dmem_data_MA[k] = dbus_rd_data_MA;
      end else begin : g_dmem
        always_ff @(posedge i_clk) begin
          if (i_rst)          dmem_data_MA[k] <= 32'd0;
          else if (en_MA)     dmem_data_MA[k] <= dmem_data_MA[k-1];
        end
      end
    end
  endgenerate

  assign ctrl_MA  = ctrl_MA_c [p_stage_MA-1];
  assign valid_MA = valid_MA_c[p_stage_MA-1];

  //! write back stage
  `KEEP_HIERARCHY
  cpu_write_back #(
    .p_wb_buf       ( 0                     )
  ) write_back_stage (
    .i_clk          ( i_clk                 ),
    .i_rst          ( i_rst                 ),
    .i_en_wb        ( en_wb_MA & valid_MA   ),
    .i_sel_wb       ( sel_wb_MA             ),
    .i_rf_wr_addr   ( wb_addr_MA            ),
    .i_alu_out      ( alu_out_MA            ),
    .i_muldiv_out   ( muldiv_out_MA         ),
    .i_csr_rd_data  ( csr_rd_data_MA        ),
    .i_copro_out0   ( copro_out0_MA         ),
    .i_copro_out1   ( copro_out1_MA         ),
    .i_copro_out2   ( copro_out2_MA         ),
    .i_pc_inc       ( pc_inc_MA             ),
    .i_dbus_rd_data ( dmem_data_MA[p_stage_MA-1] ),
    .o_rf_wr_addr   ( rf_wr_addr            ),
    .o_rf_wr_data   ( rf_wr_data            ),
    .o_rf_wr_en     ( rf_wr_en              ) 
  );

  //! bubble injected in the WB barrier at reset
  function automatic pipe_wb_t wb_reset();
    wb_reset        = PIPE_WB_RST;
    wb_reset.com.pc = p_reset_vector - 32'd4;
  endfunction

  //! bubble injected in the WB barrier on a flush (the pc is kept for debug)
  function automatic pipe_wb_t wb_bubble(input pipe_wb_t cur);
    wb_bubble        = PIPE_WB_NOP;
    wb_bubble.com.pc = cur.com.pc;
  endfunction

  //! where each WB payload field comes from
  always_comb begin
    ctrl_WB_n.com.pc           = pc_MA;
    ctrl_WB_n.com.pc_inc       = pc_inc_MA;
    ctrl_WB_n.com.imm          = imm_MA;
    ctrl_WB_n.com.instr_name   = instr_name_MA;
    ctrl_WB_n.com.instr_decomp = instr_decomp_MA;
    ctrl_WB_n.rs               = ctrl_MA.rs;
    ctrl_WB_n.bp               = ctrl_MA.bp;
    ctrl_WB_n.imm_raw          = imm_MA;
    ctrl_WB_n.rd1_data_raw     = rf_rd1_data_MA;
    ctrl_WB_n.rd1_data         = rf_rd1_data_MA;
    ctrl_WB_n.rd2_data         = rf_rd2_data_MA;
    ctrl_WB_n.alu_out          = alu_out_MA;
    ctrl_WB_n.branch_taken     = branch_taken_MA;
    ctrl_WB_n.sel_wb           = sel_wb_MA;
    ctrl_WB_n.rf_wr_addr       = rf_wr_addr;
    ctrl_WB_n.rf_wr_data       = rf_wr_data;
    ctrl_WB_n.rf_wr_en         = rf_wr_en;
  end

  generate
    if (p_stage_WB == 0) begin : g_barrier_WB
      //! no barrier: the WB slot is a combinational alias of the MA slot
      always_comb begin
        ctrl_WB     = ctrl_WB_n;
        valid_WB    = valid_MA;
        ctrl_WB_c[0]  = ctrl_WB_n;
        valid_WB_c[0] = valid_MA;
      end
    end else begin : g_barrier_WB
      for (genvar k = 0; k < p_stage_WB; k++) begin : g_lvl
        always_ff @(posedge i_clk) begin
          if (i_rst) begin
            ctrl_WB_c [k] <= wb_reset();
            valid_WB_c[k] <= 1'b0;
          end else if ((k == 0) & flush_WB) begin
            ctrl_WB_c [k] <= wb_bubble(ctrl_WB_c[k]);
            valid_WB_c[k] <= 1'b0;
          end else if (en_WB) begin
            ctrl_WB_c [k] <= (k == 0) ? ctrl_WB_n : ctrl_WB_c [k-1];
            valid_WB_c[k] <= (k == 0) ? valid_MA  : valid_WB_c[k-1];
          end
        end
      end
      assign ctrl_WB  = ctrl_WB_c [p_stage_WB-1];
      assign valid_WB = valid_WB_c[p_stage_WB-1];
    end
  endgenerate

  /*******************************************************
    Pending write

    When the register file read is registered, the write issued by the WB slot
    one cycle ago is not visible on the read port yet, so it is one more
    forwarding entry -- the oldest one.
  *******************************************************/

  generate
    if (p_stage_RF != 0) begin : g_pending_write
      always_ff @(posedge i_clk) begin
        if (i_rst) begin
          rf_wr_data_PW <= 32'd0;
          wb_addr_PW    <= 5'd0;
          en_wb_PW      <= 1'b0;
        end else begin
          rf_wr_data_PW <= rf_wr_data_WB;
          wb_addr_PW    <= rf_wr_addr_WB;
          en_wb_PW      <= rf_wr_en_WB;
        end
      end
    end else begin : g_pending_write
      always_comb begin
        rf_wr_data_PW = 32'd0;
        wb_addr_PW    = 5'd0;
        en_wb_PW      = 1'b0;
      end
    end
  endgenerate

  /*******************************************************
    Forwarding network

    One entry per barrier downstream of the RF slot, youngest first, described
    uniformly whatever the group it belongs to. `cpu_hazard` picks the youngest
    matching entry; the mux below applies its decision. Nothing here mentions a
    particular pipeline shape, which is what makes a group of any depth work
    without a line of extra code.
  *******************************************************/

  //! the write back source mux, evaluated wherever an in-flight result has to
  //! be forwarded from. `dmem` is only reachable from an MA barrier.
  function automatic logic [31: 0] wb_mux(
    input sel_wb_e     sel,
    input pipe_dat_t   dat,
    input logic [31:0] pc_ret,
    input logic [31:0] dmem
  );
    case (sel)
      wb_alu    : wb_mux = dat.alu_out;
      wb_muldiv : wb_mux = dat.muldiv_out;
      wb_pc     : wb_mux = pc_ret;
      wb_dmem   : wb_mux = dmem;
      wb_csr    : wb_mux = dat.csr_rd_data;
      wb_copro0 : wb_mux = dat.copro_out0;
      wb_copro1 : wb_mux = dat.copro_out1;
      wb_copro2 : wb_mux = dat.copro_out2;
      default   : wb_mux = 32'd0;
    endcase
  endfunction

  generate
    //! EX barriers: a load has no result yet, the data memory is only read from
    //! the EX slot. That is the one entry that can be matched but not forwarded.
    for (genvar k = 0; k < N_FWD_EX; k++) begin : g_fwd_EX
      assign fwd_valid[ OFF_EX+k        ] = valid_EX_c[k];
      assign fwd_en_wb[ OFF_EX+k        ] = ctrl_EX_c [k].wbc.en_wb;
      assign fwd_addr [(OFF_EX+k)*5 +: 5] = ctrl_EX_c [k].wbc.wb_addr;
      assign fwd_ready[ OFF_EX+k        ] = (ctrl_EX_c[k].wbc.sel_wb != wb_dmem);
      assign fwd_data [ OFF_EX+k        ] = wb_mux(ctrl_EX_c[k].wbc.sel_wb,
                                                   ctrl_EX_c[k].dat,
                                                   ctrl_EX_c[k].com.pc_inc,
                                                   32'd0);
    end
    //! MA barriers: the last one is the write back mux itself, already built
    for (genvar k = 0; k < N_FWD_MA; k++) begin : g_fwd_MA
      assign fwd_valid[ OFF_MA+k        ] = valid_MA_c[k];
      assign fwd_en_wb[ OFF_MA+k        ] = ctrl_MA_c [k].wbc.en_wb;
      assign fwd_addr [(OFF_MA+k)*5 +: 5] = ctrl_MA_c [k].wbc.wb_addr;
      assign fwd_ready[ OFF_MA+k        ] = 1'b1;
      if (k == p_stage_MA-1) begin : g_data
        assign fwd_data[OFF_MA+k] = rf_wr_data;
      end else begin : g_data
        assign fwd_data[OFF_MA+k] = wb_mux(ctrl_MA_c[k].wbc.sel_wb,
                                           ctrl_MA_c[k].dat,
                                           ctrl_MA_c[k].com.pc_inc,
                                           dmem_data_MA[k]);
      end
    end
    //! WB barriers: the value is already resolved and carried by the payload
    for (genvar k = 0; k < N_FWD_WB; k++) begin : g_fwd_WB
      assign fwd_valid[ OFF_WB+k        ] = valid_WB_c[k];
      assign fwd_en_wb[ OFF_WB+k        ] = ctrl_WB_c [k].rf_wr_en;
      assign fwd_addr [(OFF_WB+k)*5 +: 5] = ctrl_WB_c [k].rf_wr_addr;
      assign fwd_ready[ OFF_WB+k        ] = 1'b1;
      assign fwd_data [ OFF_WB+k        ] = ctrl_WB_c [k].rf_wr_data;
    end
    //! pending write
    if (N_FWD_PW != 0) begin : g_fwd_PW
      assign fwd_valid[ OFF_PW        ] = 1'b1;
      assign fwd_en_wb[ OFF_PW        ] = en_wb_PW;
      assign fwd_addr [ OFF_PW*5 +: 5 ] = wb_addr_PW;
      assign fwd_ready[ OFF_PW        ] = 1'b1;
      assign fwd_data [ OFF_PW        ] = rf_wr_data_PW;
    end
  endgenerate

  //! which group each decision came from, for the simulation log only
  always_comb begin: bypass_trace
    bp_rs1_ex = 1'b0; bp_rs1_ma = 1'b0; bp_rs1_wb = 1'b0; bp_rs1_pw = 1'b0;
    bp_rs2_ex = 1'b0; bp_rs2_ma = 1'b0; bp_rs2_wb = 1'b0; bp_rs2_pw = 1'b0;
    for (int i = 0; i < N_FWD; i++) begin
      if (fwd_rs1[i]) begin
        if      (i < OFF_MA) bp_rs1_ex = 1'b1;
        else if (i < OFF_WB) bp_rs1_ma = 1'b1;
        else if (i < OFF_PW) bp_rs1_wb = 1'b1;
        else                 bp_rs1_pw = 1'b1;
      end
      if (fwd_rs2[i]) begin
        if      (i < OFF_MA) bp_rs2_ex = 1'b1;
        else if (i < OFF_WB) bp_rs2_ma = 1'b1;
        else if (i < OFF_PW) bp_rs2_wb = 1'b1;
        else                 bp_rs2_pw = 1'b1;
      end
    end
  end


  /******************
    Debug functions
  ******************/

`ifdef verilator

  function [31: 0] get_imm();
    // verilator public
    get_imm = debug_imm;
  endfunction
  function [31: 0] get_rs1_addr();
    // verilator public
    get_rs1_addr = debug_rs1_addr;
  endfunction
  function [31: 0] get_rs1_data();
    // verilator public
    get_rs1_data = debug_rs1_data;
  endfunction
  function get_rs2_used();
    // verilator public
    get_rs2_used = debug_rs2_used;
  endfunction
  function [31: 0] get_rs2_addr();
    // verilator public
    get_rs2_addr = debug_rs2_addr;
  endfunction
  function [31: 0] get_rs2_data();
    // verilator public
    get_rs2_data = debug_rs2_data;
  endfunction
  function [31: 0] get_wb_addr();
    // verilator public
    get_wb_addr = debug_wb_addr;
  endfunction
  function [31: 0] get_wb_data();
    // verilator public
    get_wb_data = debug_wb_data;
  endfunction
  function get_wb_en();
    // verilator public
    get_wb_en = debug_wb_en;
  endfunction
  function get_br_taken();
    // verilator public
    get_br_taken = debug_br_taken;
  endfunction
  function [31: 0] get_pc();
  // verilator public
    get_pc = debug_pc;
  endfunction
  function [63: 0] get_instret();
  // verilator public
    get_instret = debug_instret;
  endfunction
  function get_valid();
  // verilator public
    get_valid = debug_valid;
  endfunction
  function string  get_instr_name();
  // verilator public
    get_instr_name = strtail(debug_instr_name.name());
  endfunction
  function [31: 0] get_instr_code();
  // verilator public
    get_instr_code = debug_instr_code;
  endfunction

  function get_stall_rs1();
    // verilator public
    get_stall_rs1 = debug_stall_rs1;
  endfunction
  function get_stall_rs2();
    // verilator public
    get_stall_rs2 = debug_stall_rs2;
  endfunction
  function get_bp_rs1_ex();
    // verilator public
    get_bp_rs1_ex = debug_bp_rs1_ex;
  endfunction
  function get_bp_rs1_ma();
    // verilator public
    get_bp_rs1_ma = debug_bp_rs1_ma;
  endfunction
  function get_bp_rs1_wb();
    // verilator public
    get_bp_rs1_wb = debug_bp_rs1_wb;
  endfunction
  function get_bp_rs2_ex();
    // verilator public
    get_bp_rs2_ex = debug_bp_rs2_ex;
  endfunction
  function get_bp_rs2_ma();
    // verilator public
    get_bp_rs2_ma = debug_bp_rs2_ma;
  endfunction
  function get_bp_rs2_wb();
    // verilator public
    get_bp_rs2_wb = debug_bp_rs2_wb;
  endfunction

  function automatic string strtail(string str);
    strtail = str.substr(4, str.len()-1);
  endfunction
`endif

endmodule

`endif // __CPU_CORE_PIPE__
 
