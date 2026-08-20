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
`else
  `include "soc/soc_config.sv"
  `include "core/packages/pck_control.sv"
  `include "core/packages/pck_isa.sv"
  `include "core/packages/pck_isa_i.sv"
  `include "core/packages/pck_mem_bus.sv"
`endif

module cpu_core_pipe 
  import pck_control::*;
  import pck_isa::*;
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

  // pipeline settings:
  parameter p_stage_IF     = 1,           //!
  parameter p_stage_IC     = 0,           //!
  parameter p_stage_ID     = 1,           //!
  parameter p_stage_RF     = 0,           //!
  parameter p_stage_EX     = 1,           //!
  parameter p_stage_MA     = 1,           //!
  parameter p_stage_WB     = 1,           //!

  // non pipeline settings:
  parameter p_prefetch_buf = 0,           //! use a prefetch buffer
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

  // regfile
  wire          rf_rd1_en;                //! read first register (used only if p_rf_sp = 1)
  wire          rf_rd2_en;                //! read second register (used only if p_rf_sp = 1)
  wire          rf_busy_RF;               //! regfile is busy
  wire          rf_addr_oob_RF;           //! regfile address out of bounds //TODO: handle this exception

  // decompress


  wire          compressed;               //! instruction is compressed
  wire          update_comp;              //! update decompressor fsm
  wire          offset_pc;                //! force fetching next pc
  wire          refetch;                  //!
  wire          freeze_pc;                //!

  // csr
  logic [63: 0] minstret;


  /*******************************************************
  *******************************************************/

  logic         update_pc;
  logic         update_pc_test;
  logic         update_instret;

  logic         en_IF;
  logic         en_IC;
  logic         en_ID;
  logic         en_ID_reg;
  logic         en_ID_instr;
  logic         en_RF;
  logic         en_RF_addr;
  logic         en_EX;
  logic         en_MA;
  logic         en_WB;



  logic         valid_IF;
  logic         valid_IC;
  logic         valid_ID;
  logic         valid_RF;
  logic         valid_EX;
  logic         valid_MA;
  logic         valid_WB;

  logic         squash_IF;
  logic         squash_IC;
  logic         squash_ID;
  logic         squash_RF;
  logic         squash_EX;
  logic         squash_MA;
  logic         squash_WB;

  logic         bp_jal;                   //! bypass for jal
  logic         bp_jalr;                  //! bypass for jalr
  logic         bp_branch;                //! bypass for other branches
  logic         bp_rs1_rf;
  logic         bp_rs1_ex;
  logic         bp_rs1_ma;
  logic         bp_rs1_wb;
  logic         bp_rs1_pw;
  logic         bp_rs2_rf;
  logic         bp_rs2_ex;
  logic         bp_rs2_ma;
  logic         bp_rs2_wb;
  logic         bp_rs2_pw;
  
  // IF (Instruction Fetch) stage
  logic [31: 0] pc_IF;                    //! program counter
  logic [31: 0] pc_inc_IF;                //! program +4 / +2
  isa_instr_t   instr_IF;                 //! raw instruction from instruction memory
  logic         branch_taken_IF;          //! a branch is taken
  logic         half_pc_addr_IF;          //! high when the pc is not a multiple of 4
  logic         half_npc_addr_IF;         //! high when the next pc is not a multiple of 4
  logic         bypass_decomp_IF;         //! bypass the decompressor
  logic [63: 0] minstret_IF;              //! instruction counter

  logic [31: 0] pc;                       //! program counter
  logic [31: 0] pc_inc;                   //! program +4 / +2
  isa_instr_t   instr;                    //! raw instruction from instruction memory
  logic         half_pc_addr;             //! high when the pc is not a multiple of 4
  logic         half_npc_addr;            //! high when the next pc is not a multiple of 4
  logic         bypass_decomp;            //! bypass the decompressor

  logic         force_instret;
  logic [63: 0] minstret_force_value;

  // IC (Instruction deCompress) stage
  logic [31: 0] pc_IC;                    //! program counter
  logic [31: 0] pc_inc_IC;                //! program +4 / +2
  logic [31: 0] pc_reg_IC;                //! program counter
  logic [31: 0] pc_inc_reg_IC;            //! program +4 / +2
  isa_instr_t   instr_decomp_IC;          //! decompressed instruction
  logic [63: 0] minstret_IC;              //! instruction counter

  isa_instr_t   instr_decomp;             //! decompressed instruction
  isa_instr_t   instr_decomp_reg;         //! decompressed instruction

  // ID (Instruction Decode) stage
  logic [31: 0] pc_ID;                    //! program counter
  logic [31: 0] pc_inc_ID;                //! program +4 / +2
  isa_instr_e   instr_name_ID;            //! instruction name (debug)
  isa_instr_t   instr_decomp_ID;          //! decompressed instruction
  logic [63: 0] minstret_ID;              //! instruction counter
  logic [31: 0] imm_ID;                   //! immediate value
  logic [ 4: 0] wb_addr_ID;               //! write back addres
  logic [ 4: 0] rf_rd1_addr_ID;           //! regfile read address for port 1
  logic [ 4: 0] rf_rd2_addr_ID;           //! regfile read address for port 2
  logic         rf_rd2_used_ID;           //! register file read port 2 is used
  sel_br_e      sel_br_ID;                //! program counter mux selector
  sel_alu_op_e  sel_alu_op_ID;            //! ALU operation selector
  sel_alu_opa_e sel_alu_opa_ID;           //! ALU operand A selector
  sel_alu_opb_e sel_alu_opb_ID;           //! ALU operand B selector
  sel_md_op_e   sel_md_op_ID;             //! MULDIV operation selector
  sel_csr_wr_e  sel_csr_wr_ID;            //! csr write data selector
  sel_csr_op_e  sel_csr_op_ID;            //! csr write operation selector
  logic         opa_signed_ID;            //! MULDIV operand A is signed
  logic         opb_signed_ID;            //! MULDIV operand B is signed
  logic         wen_csr_ID;               //! csr write enable
  logic         ren_csr_ID;               //! csr read enable
  logic [11: 0] csr_addr_ID;              //! csr address
  sel_be_e      sel_dmem_be_ID;           //! dmem address mode (word, byte, halfbyte)
  logic         dmem_sext_ID;             //! sign extension on non-word dmem data
  logic         en_dmem_wr_ID;            //! data memory write enable
  logic         en_dmem_rd_ID;            //! data memory read enable
  logic         swap_bytes_ID;            //! 
  sel_wb_e      sel_wb_ID;                //! write back source selector
  logic         en_wb_ID;                 //! register file write enable
  logic         cond_branch_ID;           //! conditionnal branch
  logic         jump_reg_ID;              //! jump register
  logic         stall_rs1_ID;             //! a stall is needed
  logic         stall_rs2_ID;             //! a stall is needed

  isa_instr_e   instr_name;               //! instruction name (debug)
  isa_instr_e   instr_name_abs;           //! instruction name (debug)
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
  logic         jump_reg;                 //! jump register
  logic         stall_rs1;                //! a stall is needed
  logic         stall_rs2;                //! a stall is needed
  logic [31: 0] csr_rd_data;

  // RF (Register File) stage
  logic [31: 0] pc_RF;                    //! program counter
  logic [31: 0] pc_inc_RF;                //! program +4 / +2
  isa_instr_e   instr_name_RF;            //! instruction name (debug)
  isa_instr_t   instr_decomp_RF;          //! decompressed instruction
  logic [63: 0] minstret_RF;              //! instruction counter
  logic [31: 0] imm_RF;                   //! immediate value
  logic [ 4: 0] wb_addr_RF;               //! write back addres
  logic [ 4: 0] rf_rd1_addr_RF;           //! regfile read address for port 1
  logic [ 4: 0] rf_rd2_addr_RF;           //! regfile read address for port 2
  logic         rf_rd2_used_RF;           //! register file read port 2 is used
  logic [31: 0] rf_rd1_data_RF;           //! regfile read data for port 1
  logic [31: 0] rf_rd2_data_RF;           //! regfile read data for port 2
  logic [31: 0] rf_rd1_data_bp_RF;        //! regfile read data for port 1
  logic [31: 0] rf_rd2_data_bp_RF;        //! regfile read data for port 2
  sel_br_e      sel_br_RF;                //! program counter mux selector
  sel_alu_op_e  sel_alu_op_RF;            //! ALU operation selector
  sel_alu_opa_e sel_alu_opa_RF;           //! ALU operand A selector
  sel_alu_opb_e sel_alu_opb_RF;           //! ALU operand B selector
  sel_md_op_e   sel_md_op_RF;             //! MULDIV operation selector
  sel_csr_wr_e  sel_csr_wr_RF;            //! csr write data selector
  sel_csr_op_e  sel_csr_op_RF;            //! csr write operation selector
  logic         opa_signed_RF;            //! MULDIV operand A is signed
  logic         opb_signed_RF;            //! MULDIV operand B is signed
  logic         wen_csr_RF;               //! csr write enable
  logic         ren_csr_RF;               //! csr read enable
  logic [11: 0] csr_addr_RF;              //! csr address
  sel_be_e      sel_dmem_be_RF;           //! dmem address mode (word, byte, halfbyte)
  logic         dmem_sext_RF;             //! sign extension on non-word dmem data
  logic         en_dmem_wr_RF;            //! data memory write enable
  logic         en_dmem_rd_RF;            //! data memory read enable
  logic         swap_bytes_RF;            //! 
  sel_wb_e      sel_wb_RF;                //! write back source selector
  logic         en_wb_RF;                 //! register file write enable
  logic         stall_rs1_RF;             //! a stall is needed
  logic         stall_rs2_RF;             //! a stall is needed

  logic [ 4: 0] wb_addr_RF_;               //! write back addres
  logic [ 4: 0] rf_rd1_addr_RF_;           //! regfile read address for port 1
  logic [ 4: 0] rf_rd2_addr_RF_;           //! regfile read address for port 2
  logic         rf_rd2_used_RF_;           //! register file read port 2 is used
  sel_br_e      sel_br_RF_;                //! program counter mux selector
  sel_wb_e      sel_wb_RF_;                //! write back source selector

  logic [31: 0] rf_rd1_data;              //! regfile read data for port 1
  logic [31: 0] rf_rd2_data;              //! regfile read data for port 2
  logic [31: 0] rf_rd1_data_reg;          //! regfile read data for port 1
  logic [31: 0] rf_rd2_data_reg;          //! regfile read data for port 2

  // EX (EXecusion) stage
  logic [31: 0] pc_EX;                    //! program counter
  logic [31: 0] pc_inc_EX;                //! program +4 / +2
  isa_instr_e   instr_name_EX;            //! instruction name (debug)
  isa_instr_t   instr_decomp_EX;          //! decompressed instruction
  logic [63: 0] minstret_EX;              //! instruction counter
  sel_pc_e      sel_pc_EX;                //! program counter select
  logic [31: 0] imm_EX;                   //! immediate value
  logic [ 4: 0] wb_addr_EX;               //! write back address
  logic [ 4: 0] rf_rd1_addr_EX;           //! regfile read address for port 1
  logic [ 4: 0] rf_rd2_addr_EX;           //! regfile read address for port 2
  logic         rf_rd2_used_EX;           //! register file read port 2 is used
  logic [31: 0] rf_rd1_data_EX;           //! regfile read data for port 1
  logic [31: 0] rf_rd2_data_EX;           //! regfile read data for port 2
  sel_br_e      sel_br_EX;                //! program counter mux selector
  logic [31: 0] alu_out_EX;               //! ALU output
  logic [31: 0] adder_out_EX;             //! adder output
  logic [31: 0] muldiv_out_EX;            //! MUL/DIV result
  logic [31: 0] csr_rd_data_EX;           //! read data from csr
  logic         exec_done_EX;             //! execute stage done
  logic         branch_taken_EX;          //! a branch is taken
  sel_be_e      sel_dmem_be_EX;           //! dmem address mode (word, byte, halfbyte)
  logic         dmem_sext_EX;             //! sign extension on non-word dmem data
  logic         en_dmem_wr_EX;            //! data memory write enable
  logic         en_dmem_rd_EX;            //! data memory read enable
  logic         swap_bytes_EX;            //! 
  sel_wb_e      sel_wb_EX;                //! write back source selector
  logic         en_wb_EX;                 //! register file write enable
  logic         stall_rs1_EX;             //! a stall is needed
  logic         stall_rs2_EX;             //! a stall is needed
  logic [31: 0] copro_out0_EX;
  logic [31: 0] copro_out1_EX;
  logic [31: 0] copro_out2_EX;

  logic         bp_rs1_ex_EX;
  logic         bp_rs1_ma_EX;
  logic         bp_rs1_wb_EX;
  logic         bp_rs2_ex_EX;
  logic         bp_rs2_ma_EX;
  logic         bp_rs2_wb_EX;

  logic [31: 0] alu_out;                  //! ALU output
  logic [31: 0] adder_out;                //! adder output
  logic [31: 0] muldiv_out;               //! MUL/DIV result
  logic         exec_done;                //! execute stage done
  logic         branch_taken;             //! a branch is taken
  sel_pc_e      sel_pc;                   //! program counter select

  // MA (Memory Access) stage
  logic [31: 0] pc_MA;                    //! program counter
  logic [31: 0] pc_inc_MA;                //! program +4 / +2
  isa_instr_e   instr_name_MA;            //! instruction name (debug)
  isa_instr_t   instr_decomp_MA;          //! decompressed instruction
  logic [63: 0] minstret_MA;              //! instruction counter
  sel_pc_e      sel_pc_MA;                //! program counter select
  logic [31: 0] imm_MA;                   //! immediate value
  logic [ 4: 0] wb_addr_MA;               //! write back address
  logic [ 4: 0] rf_rd1_addr_MA;           //! regfile read address for port 1
  logic [ 4: 0] rf_rd2_addr_MA;           //! regfile read address for port 2
  logic         rf_rd2_used_MA;           //! register file read port 2 is used
  logic [31: 0] rf_rd1_data_MA;           //! regfile read data for port 1
  logic [31: 0] rf_rd2_data_MA;           //! regfile read data for port 1
  sel_br_e      sel_br_MA;                //! program counter mux selector
  logic [31: 0] alu_out_MA;               //! ALU output
  logic [31: 0] muldiv_out_MA;            //! MUL/DIV result
  logic [31: 0] csr_rd_data_MA;           //! read data from csr
  logic         branch_taken_MA;          //! a branch is taken
  logic [31: 0] dbus_rd_data_MA;          //! data bus read data
  logic         dbus_busy_MA;             //! data bus busy
  logic         dbus_ack_MA;              //! data bus acknowledge
  sel_wb_e      sel_wb_MA;                //! write back source selector
  logic         en_wb_MA;                 //! register file write enable
  logic         stall_rs1_MA;             //! a stall is needed
  logic         stall_rs2_MA;             //! a stall is needed

  logic [31: 0] copro_out0_MA;
  logic [31: 0] copro_out1_MA;
  logic [31: 0] copro_out2_MA;

  logic         bp_rs1_ex_MA;
  logic         bp_rs1_ma_MA;
  logic         bp_rs1_wb_MA;
  logic         bp_rs2_ex_MA;
  logic         bp_rs2_ma_MA;
  logic         bp_rs2_wb_MA;

  // WB (Write Back) stage
  logic [31: 0] pc_WB;                    //! program counter
  logic [31: 0] jump_pc_WB;               //! program counter
  isa_instr_e   instr_name_WB;            //! instruction name (debug)
  isa_instr_t   instr_decomp_WB;          //! decompressed instruction
  logic [63: 0] minstret_WB;              //! instruction counter
  sel_pc_e      sel_pc_WB;                //! program counter select
  logic [31: 0] imm_WB;                   //! immediate value
  logic [31: 0] imm_WB_;                  //! immediate value (without branch bypass)
  logic [ 4: 0] rf_rd1_addr_WB;           //! regfile read address for port 1
  logic [ 4: 0] rf_rd2_addr_WB;           //! regfile read address for port 2
  logic         rf_rd2_used_WB;           //! register file read port 2 is used
  logic [31: 0] rf_rd1_data_WB;           //! regfile read data for port 1
  logic [31: 0] rf_rd1_data_WB_;          //! regfile read data for port 1 (without branch bypass)
  logic [31: 0] rf_rd2_data_WB;           //! regfile read data for port 1
  logic [31: 0] alu_out_WB;               //! ALU output
  logic         branch_taken_WB;          //! a branch is taken
  logic [31: 0] rf_wr_data_WB;            //! regfile write data
  logic [ 4: 0] rf_wr_addr_WB;            //! regfile write address
  logic         rf_wr_en_WB;              //! regfile write enable
  logic         stall_rs1_WB;             //! a stall is needed
  logic         stall_rs2_WB;             //! a stall is needed
  sel_wb_e      sel_wb_WB;                //! write back source selector

  logic [31: 0] rf_wr_data_PW;            //! regfile write data

  logic         bp_rs1_ex_WB;
  logic         bp_rs1_ma_WB;
  logic         bp_rs1_wb_WB;
  logic         bp_rs2_ex_WB;
  logic         bp_rs2_ma_WB;
  logic         bp_rs2_wb_WB;
  
  logic [31: 0] branch_instr_pc;

  logic [31: 0] rf_wr_data;               //! regfile write data
  logic [ 4: 0] rf_wr_addr;               //! regfile write address
  logic         rf_wr_en;                 //! regfile write enable

  logic [31: 0] test;

  // debug signals used for log
  wire  [31: 0] debug_imm        = imm_WB_;
  wire  [31: 0] debug_rs1_addr   = rf_rd1_addr_WB;
  wire  [31: 0] debug_rs1_data   = rf_rd1_data_WB_;
  wire          debug_rs2_used   = rf_rd2_used_WB;
  wire  [31: 0] debug_rs2_addr   = rf_rd2_addr_WB;
  wire  [31: 0] debug_rs2_data   = rf_rd2_data_WB;
  wire  [31: 0] debug_pc         = pc_WB;
  wire  [63: 0] debug_instret    = minstret_WB;
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
  //wire          debug_bp_rs1_pw  = bp_rs1_pw_WB;
  wire          debug_bp_rs2_ex  = bp_rs2_ex_WB;
  wire          debug_bp_rs2_ma  = bp_rs2_ma_WB;
  wire          debug_bp_rs2_wb  = bp_rs2_wb_WB;
  //wire          debug_bp_rs2_pw  = bp_rs2_pw_WB;
  wire          debug_stall_rs1  = stall_rs1_WB;
  wire          debug_stall_rs2  = stall_rs2_WB;
  

  cpu_hazard #(
    .p_stage_IF         ( p_stage_IF            ),
    .p_stage_IC         ( p_stage_IC            ),
    .p_stage_ID         ( p_stage_ID            ),
    .p_stage_RF         ( p_stage_RF            ),
    .p_stage_EX         ( p_stage_EX            ),
    .p_stage_MA         ( p_stage_MA            ),
    .p_stage_WB         ( p_stage_WB            )
  ) hazard_unit (
    .i_clk              ( i_clk                 ),
    .i_rst              ( i_rst                 ),
    .i_rf_rd1_addr      ( rf_rd1_addr           ),
    .i_rf_rd2_addr      ( rf_rd2_addr           ),
    .i_rf_rd2_used      ( rf_rd2_used           ),
    .i_rf_rd1_addr_ID   ( rf_rd1_addr_ID        ),
    .i_rf_rd2_addr_ID   ( rf_rd2_addr_ID        ),
    .i_rf_rd2_used_ID   ( rf_rd2_used_ID        ),
    .i_rf_rd1_addr_RF   ( rf_rd1_addr_RF        ),
    .i_rf_rd2_addr_RF   ( rf_rd2_addr_RF        ),
    .i_rf_rd2_used_RF   ( rf_rd2_used_RF        ),
    .i_rf_rd1_addr_EX   ( rf_rd1_addr_EX        ),
    .i_rf_rd2_addr_EX   ( rf_rd2_addr_EX        ),
    .i_rf_rd2_used_EX   ( rf_rd2_used_EX        ),
    .i_rf_rd1_addr_MA   ( rf_rd1_addr_MA        ),
    .i_rf_rd2_addr_MA   ( rf_rd2_addr_MA        ),
    .i_rf_rd2_used_MA   ( rf_rd2_used_MA        ),
    .i_wb_addr_ID       ( wb_addr_ID            ),
    .i_wb_addr_RF       ( wb_addr_RF            ),
    .i_wb_addr_EX       ( wb_addr_EX            ),
    .i_wb_addr_MA       ( wb_addr_MA            ),
    .i_wb_addr_WB       ( rf_wr_addr_WB         ),
    .i_pc               ( pc_EX                 ),
    .i_sel_br           ( sel_br                ),
    .i_sel_br_ID        ( sel_br_ID             ),
    .i_sel_br_RF        ( sel_br_RF             ),
    .i_sel_br_EX        ( sel_br_EX             ),
    .i_sel_br_MA        ( sel_br_MA             ),
    .i_branch_taken     ( branch_taken          ),
    .i_branch_taken_EX  ( branch_taken_EX       ),
    .i_branch_taken_MA  ( branch_taken_MA       ),
    .i_sel_wb           ( sel_wb                ),
    .i_sel_wb_ID        ( sel_wb_ID             ),
    .i_sel_wb_RF        ( sel_wb_RF             ),
    .i_sel_wb_EX        ( sel_wb_EX             ),
    .i_sel_wb_MA        ( sel_wb_MA             ),
    .i_sel_wb_WB        ( sel_wb_WB             ),
    .i_valid_IF         ( valid_IF              ),
    .i_valid_IC         ( valid_IC              ),
    .i_valid_ID         ( valid_ID              ),
    .i_valid_RF         ( valid_RF              ),
    .i_valid_EX         ( valid_EX              ),
    .i_valid_MA         ( valid_MA              ),
    .i_valid_WB         ( valid_WB              ),
    .o_squash_IF        ( squash_IF             ),
    .o_squash_IC        ( squash_IC             ),
    .o_squash_ID        ( squash_ID             ),
    .o_squash_RF        ( squash_RF             ),
    .o_squash_EX        ( squash_EX             ),
    .o_squash_MA        ( squash_MA             ),
    .o_squash_WB        ( squash_WB             ),
    .o_update_pc        ( update_pc             ),
    .o_update_pc_test   ( update_pc_test        ),
    .o_update_instret   ( update_instret        ),
    .o_en_IF            ( en_IF                 ),
    .o_en_IC            ( en_IC                 ),
    .o_en_ID            ( en_ID                 ),
    .o_en_ID_instr      ( en_ID_instr           ),
    .o_en_RF            ( en_RF                 ),
    .o_en_RF_addr       ( en_RF_addr            ),
    .o_en_EX            ( en_EX                 ),
    .o_en_MA            ( en_MA                 ),
    .o_en_WB            ( en_WB                 ),
    .o_bp_rs1_rf        ( bp_rs1_rf             ),
    .o_bp_rs1_ex        ( bp_rs1_ex             ),
    .o_bp_rs1_ma        ( bp_rs1_ma             ),
    .o_bp_rs1_wb        ( bp_rs1_wb             ),
    .o_bp_rs1_pw        ( bp_rs1_pw             ),
    .o_bp_rs2_rf        ( bp_rs2_rf             ),
    .o_bp_rs2_ex        ( bp_rs2_ex             ),
    .o_bp_rs2_ma        ( bp_rs2_ma             ),
    .o_bp_rs2_wb        ( bp_rs2_wb             ),
    .o_bp_rs2_pw        ( bp_rs2_pw             ),
    .o_bp_jal           ( bp_jal                ),
    .o_bp_jalr          ( bp_jalr               ),
    .o_bp_branch        ( bp_branch             ),
    .o_force_instret    ( force_instret         ),
    .o_stall_rs1        ( stall_rs1             ),
    .o_stall_rs2        ( stall_rs2             )
  );


  //! synchronous fetch stage
  `KEEP_HIERARCHY
  cpu_fetch_pipe #(
    .p_reset_vector ( p_reset_vector        ),
    .p_branch_buf   ( p_branch_buf          )
  ) fetch_stage (   
    .i_clk          ( i_clk                 ),
    .i_rst          ( i_rst                 ),
    .i_sleep        ( i_sleep               ),
    //.ibus           ( ibus                  ),
    .ibus_addr      ( ibus_addr             ),
    .ibus_be        ( ibus_be               ),
    .ibus_wr_en     ( ibus_wr_en            ),
    .ibus_wr_data   ( ibus_wr_data          ),
    .ibus_rd_en     ( ibus_rd_en            ),
    .ibus_rd_data   ( ibus_rd_data          ),
    .ibus_busy      ( ibus_busy             ),
    .ibus_ack       ( ibus_ack              ),
    .i_en_fetch     ( en_IF                 ),
    .i_update_pc    ( 1'b1                  ),
    .i_update_instret( update_instret       ),
    .i_refetch      ( 1'b0                  ),
    .i_freeze_pc    ( ~update_pc            ),
    .i_freeze_pc_test( ~update_pc_test      ),
    .i_compressed   ( 1'b0                  ),
    .i_offset_pc    ( 1'b0                  ),
    .i_sel_pc       ( sel_pc_WB             ),
    .i_alu_out      ( alu_out_WB            ),
    .i_imm          ( imm_WB                ),
    .i_rf_rd1_data  ( rf_rd1_data_WB        ),
    .i_jump_pc      ( jump_pc_WB            ),
    .i_force_instret( force_instret         ),
    .i_minstret     ( minstret_force_value  ),
    .o_pc           ( pc                    ),
    .o_pc_inc       ( pc_inc                ),
    .o_instr        ( instr                 ),
    .o_bypass_decomp( bypass_decomp         ),
    .o_half_pc_addr ( half_pc_addr          ),
    .o_half_npc_addr( half_npc_addr         ),
    .o_minstret     ( minstret              )
  );   

  assign minstret_force_value = minstret_MA+1;

  generate
    if (1) begin
      always_ff @(posedge i_clk) begin
        if (i_rst) begin
          branch_taken_IF <= 0;
          /*pc_IF           <= pc;
          pc_inc_IF       <= pc_inc;*/
        end else begin
          if (squash_IF) begin
            branch_taken_IF <= 0;
           /* pc_IF           <= pc_IF;
            pc_inc_IF       <= pc_inc_IF;*/
          end else /*if (en_IF)*/ begin
            branch_taken_IF <= branch_taken_WB;
            /*pc_IF           <= pc;
            pc_inc_IF       <= pc_inc;*/
          end
        end
      end

      always_comb begin
        if (squash_IF) begin
          valid_IF         = 0;
          pc_IF            = pc;
          pc_inc_IF        = pc_inc;
          instr_IF.code    = pck_isa_i::NOP;
          bypass_decomp_IF = 0;
          half_pc_addr_IF  = 0;
          half_npc_addr_IF = 0;
          minstret_IF      = 0;
        end else begin
          valid_IF         = 1'b1;
          pc_IF            = pc;
          pc_inc_IF        = pc_inc;
          instr_IF         = instr;
          bypass_decomp_IF = bypass_decomp;
          half_pc_addr_IF  = half_pc_addr;
          half_npc_addr_IF = half_npc_addr;
          minstret_IF      = minstret;
        end
      end
    end else begin
      always_comb begin
        valid_IF        = 1;
        branch_taken_IF = branch_taken_WB;
      end
    end
  endgenerate

  cpu_decompressor #(   
    .p_ext_rvc      ( p_ext_rvc             )
  ) decompression_stage (    
    .i_clk          ( i_clk                 ),
    .i_rst          ( i_rst                 ),
    .i_en_decomp    ( en_IC                 ),
    .i_update_comp  ( en_IC                 ),
    .i_bypass_decomp(                       ), // bypass_decomp_IF
    .i_branch_taken ( branch_taken_EX       ), // branch_taken_IF 
    .i_half_pc_addr (                       ), // half_pc_addr_IF 
    .i_half_npc_addr(                       ), // half_npc_addr_IF
    .i_instr        ( instr_IF              ),
    .o_instr        ( instr_decomp          ),
    .o_offset_pc    (                       ),
    .o_compressed   (                       ),
    .o_freeze_pc    (                       ),
    .o_refetch      (                       )
  );   

  generate
    if (0) begin
      always_ff @(posedge i_clk) begin
        if (i_rst) begin
          valid_IC        <= 0;
          pc_IC           <= p_reset_vector - 4;
          pc_inc_IC       <= 0;
          minstret_IC     <= 0;
          instr_decomp_IC <= pck_isa_i::NOP;
        end else begin
          if (en_IC) begin
            valid_IC        <= valid_IF;
            pc_IC           <= pc_IF;
            pc_inc_IC       <= pc_inc_IF;
            minstret_IC     <= minstret_IF;
            instr_decomp_IC <= instr_decomp;
          end
        end
      end
    end else begin
      always_comb begin
        valid_IC        = valid_IF;
        pc_IC           = pc_IF;
        pc_inc_IC       = pc_inc_IF;
        minstret_IC     = minstret_IF;

        if (p_stage_ID) begin
          instr_decomp_IC = instr_decomp;
        end else begin
          if (en_ID_instr) begin
            instr_decomp_IC = instr_decomp;
          end else begin
            instr_decomp_IC = instr_decomp_reg;
          end
        end
      end
  end

    always_ff @(posedge i_clk) begin
        en_ID_reg        <= en_ID;
        pc_reg_IC        <= pc_IC;
        pc_inc_reg_IC    <= pc_inc_IC;
        instr_decomp_reg <= instr_decomp;
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

  always_comb begin
    if (!squash_IF && !squash_ID) begin
      instr_name_abs = instr_name;
    end
  end

  generate
    if (p_stage_ID) begin
      always_ff @(posedge i_clk) begin
        if (i_rst) begin
          valid_ID        <= 0;
          pc_ID           <= p_reset_vector - 4;
          pc_inc_ID       <= 0;
          instr_name_ID   <= isa_nop;
          instr_decomp_ID <= pck_isa_i::NOP;
          minstret_ID     <= 0;
          rf_rd1_addr_ID  <= 0;
          rf_rd2_addr_ID  <= 0;
          rf_rd2_used_ID  <= 0;
          csr_addr_ID     <= 0;
          imm_ID          <= 0;
          sel_br_ID       <= br_none;
          sel_alu_op_ID   <= alu_nop;
          sel_alu_opa_ID  <= opa_none;
          sel_alu_opb_ID  <= opb_none;
          sel_md_op_ID    <= muldiv_nop;
          sel_csr_wr_ID   <= csr_wr_none;
          sel_csr_op_ID   <= csr_op_none;
          opa_signed_ID   <= 0;
          opb_signed_ID   <= 0;
          sel_wb_ID       <= wb_none;
          sel_dmem_be_ID  <= be_none;
          dmem_sext_ID    <= 0;
          en_dmem_wr_ID   <= 0;
          en_dmem_rd_ID   <= 0;
          swap_bytes_ID   <= 0;
          en_wb_ID        <= 0;
          wen_csr_ID      <= 0;
          ren_csr_ID      <= 0;
          cond_branch_ID  <= 0;
          jump_reg_ID     <= 0;
          stall_rs1_ID    <= 0;
          stall_rs2_ID    <= 0;
        end else begin
          if (squash_ID) begin
            valid_ID        <= 0;
            pc_ID           <= pc_ID;
            pc_inc_ID       <= pc_inc_ID;
            wb_addr_ID      <= 0;
            instr_name_ID   <= isa_nop;
            instr_decomp_ID <= pck_isa_i::NOP;
            minstret_ID     <= 0;
            rf_rd1_addr_ID  <= 0;
            rf_rd2_addr_ID  <= 0;
            rf_rd2_used_ID  <= 0;
            csr_addr_ID     <= 0;
            imm_ID          <= 0;
            sel_br_ID       <= br_none;
            sel_alu_op_ID   <= alu_nop;
            sel_alu_opa_ID  <= opa_none;
            sel_alu_opb_ID  <= opb_none;
            sel_md_op_ID    <= muldiv_nop;
            sel_csr_wr_ID   <= csr_wr_none;
            sel_csr_op_ID   <= csr_op_none;
            opa_signed_ID   <= 0;
            opb_signed_ID   <= 0;
            sel_wb_ID       <= wb_none;
            sel_dmem_be_ID  <= be_none;
            dmem_sext_ID    <= 0;
            en_dmem_wr_ID   <= 0;
            en_dmem_rd_ID   <= 0;
            swap_bytes_ID   <= 0;
            en_wb_ID        <= 0;
            wen_csr_ID      <= 0;
            ren_csr_ID      <= 0;
            cond_branch_ID  <= 0;
            jump_reg_ID     <= 0;
            stall_rs1_ID    <= 0;
            stall_rs2_ID    <= 0;
          end else if (en_ID) begin
            valid_ID        <= valid_IC;
            pc_ID           <= pc_IC;
            pc_inc_ID       <= pc_inc_IC;
            wb_addr_ID      <= wb_addr;
            instr_name_ID   <= instr_name;
            instr_decomp_ID <= instr_decomp_IC;
            minstret_ID     <= minstret_IC;
            if (en_RF_addr) begin
              rf_rd1_addr_ID  <= rf_rd1_addr;
              rf_rd2_addr_ID  <= rf_rd2_addr;
              rf_rd2_used_ID  <= rf_rd2_used;
            end
            csr_addr_ID     <= csr_addr;
            imm_ID          <= imm;
            sel_br_ID       <= sel_br;
            sel_alu_op_ID   <= sel_alu_op;
            sel_alu_opa_ID  <= sel_alu_opa;
            sel_alu_opb_ID  <= sel_alu_opb;
            sel_md_op_ID    <= sel_md_op;
            sel_csr_wr_ID   <= sel_csr_wr;
            sel_csr_op_ID   <= sel_csr_op;
            opa_signed_ID   <= opa_signed;
            opb_signed_ID   <= opb_signed;
            sel_wb_ID       <= sel_wb;
            sel_dmem_be_ID  <= sel_dmem_be;
            dmem_sext_ID    <= dmem_sext;
            en_dmem_wr_ID   <= en_dmem_wr;
            en_dmem_rd_ID   <= en_dmem_rd;
            swap_bytes_ID   <= swap_bytes;
            en_wb_ID        <= en_wb;
            wen_csr_ID      <= wen_csr;
            ren_csr_ID      <= ren_csr;
            cond_branch_ID  <= cond_branch;
            jump_reg_ID     <= jump_reg;
            stall_rs1_ID    <= stall_rs1;
            stall_rs2_ID    <= stall_rs2;
          end else begin
            valid_ID        <= valid_ID;
          end
        end
      end
    end else begin
      always_comb begin
        valid_ID        = valid_IC;
        if (en_ID_reg) begin
          pc_ID           = pc_IC;
          pc_inc_ID       = pc_inc_IC;
        end else begin
          pc_ID           = pc_reg_IC;
          pc_inc_ID       = pc_inc_reg_IC;
        end
        wb_addr_ID      = wb_addr;
        instr_name_ID   = instr_name;
        instr_decomp_ID = instr_decomp_IC;
        minstret_ID     = minstret_IC;
        rf_rd1_addr_ID  = rf_rd1_addr;
        rf_rd2_addr_ID  = rf_rd2_addr;
        rf_rd2_used_ID  = rf_rd2_used;
        csr_addr_ID     = csr_addr;
        imm_ID          = imm;
        sel_br_ID       = sel_br;
        sel_alu_op_ID   = sel_alu_op;
        sel_alu_opa_ID  = sel_alu_opa;
        sel_alu_opb_ID  = sel_alu_opb;
        sel_md_op_ID    = sel_md_op;
        sel_csr_wr_ID   = sel_csr_wr;
        sel_csr_op_ID   = sel_csr_op;
        opa_signed_ID   = opa_signed;
        opb_signed_ID   = opb_signed;
        sel_wb_ID       = sel_wb;
        sel_dmem_be_ID  = sel_dmem_be;
        dmem_sext_ID    = dmem_sext;
        en_dmem_wr_ID   = en_dmem_wr;
        en_dmem_rd_ID   = en_dmem_rd;
        swap_bytes_ID   = swap_bytes;
        en_wb_ID        = en_wb;
        wen_csr_ID      = wen_csr;
        ren_csr_ID      = ren_csr;
        cond_branch_ID  = cond_branch;
        jump_reg_ID     = jump_reg;
        stall_rs1_ID    = stall_rs1;
        stall_rs2_ID    = stall_rs2;
      end
    end
  endgenerate

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
    .i_rd1_addr     ( rf_rd1_addr_ID        ),
    .o_rd1_data     ( rf_rd1_data           ),
    .i_rd2_en       ( 1'b1                  ),
    .i_rd2_addr     ( rf_rd2_addr_ID        ),
    .o_rd2_data     ( rf_rd2_data           ),
    .i_wr_en        ( rf_wr_en_WB           ),
    .i_wr_addr      ( rf_wr_addr_WB         ),
    .i_wr_data      ( rf_wr_data_WB         ) 
  );


  generate
    if (p_stage_RF) begin
      always_ff @(posedge i_clk) begin
        if (i_rst) begin
          valid_RF        <= 0;
          pc_RF           <= p_reset_vector - 4;
          pc_inc_RF       <= 0;
          instr_name_RF   <= isa_nop;
          instr_decomp_RF <= pck_isa_i::NOP;
          minstret_RF     <= 0;
          imm_RF          <= 0;
          wb_addr_RF      <= 0;
          sel_br_RF       <= br_none;
          sel_alu_op_RF   <= alu_nop;
          sel_alu_opa_RF  <= opa_none;
          sel_alu_opb_RF  <= opb_none;
          sel_md_op_RF    <= muldiv_nop;
          sel_csr_wr_RF   <= csr_wr_none;
          sel_csr_op_RF   <= csr_op_none;
          opa_signed_RF   <= 0;
          opb_signed_RF   <= 0;
          wen_csr_RF      <= 0;
          ren_csr_RF      <= 0;
          csr_addr_RF     <= 0;
          sel_dmem_be_RF  <= be_none;
          dmem_sext_RF    <= 0;
          en_dmem_wr_RF   <= 0;
          en_dmem_rd_RF   <= 0;
          swap_bytes_RF   <= 0;
          sel_wb_RF       <= wb_none;
          en_wb_RF        <= 0;
          stall_rs1_RF    <= 0;
          stall_rs2_RF    <= 0;
          rf_rd1_data_reg <= 0;
          rf_rd2_data_reg <= 0;
        end else begin
          if (squash_RF) begin
            valid_RF        <= 0;
          /*end else if (en_RF) begin
            valid_RF        <= valid_ID;
          end
          if (0) begin
          */pc_RF           <= pc_RF;
            pc_inc_RF       <= pc_inc_RF;
            instr_name_RF   <= isa_nop;
            instr_decomp_RF <= pck_isa_i::NOP;
            minstret_RF     <= 0;
            imm_RF          <= 0;
            wb_addr_RF      <= 0;
            sel_br_RF       <= br_none;
            sel_alu_op_RF   <= alu_nop;
            sel_alu_opa_RF  <= opa_none;
            sel_alu_opb_RF  <= opb_none;
            sel_md_op_RF    <= muldiv_nop;
            sel_csr_wr_RF   <= csr_wr_none;
            sel_csr_op_RF   <= csr_op_none;
            opa_signed_RF   <= 0;
            opb_signed_RF   <= 0;
            wen_csr_RF      <= 0;
            ren_csr_RF      <= 0;
            csr_addr_RF     <= 0;
            sel_dmem_be_RF  <= be_none;
            dmem_sext_RF    <= 0;
            en_dmem_wr_RF   <= 0;
            en_dmem_rd_RF   <= 0;
            swap_bytes_RF   <= 0;
            sel_wb_RF       <= wb_none;
            en_wb_RF        <= 0;
            stall_rs1_RF    <= 0;
            stall_rs2_RF    <= 0;
            rf_rd1_data_reg <= 0;
            rf_rd2_data_reg <= 0;
          end else if (en_RF) begin
            valid_RF        <= valid_ID;
            pc_RF           <= pc_ID;
            pc_inc_RF       <= pc_inc_ID;
            instr_name_RF   <= instr_name_ID;
            instr_decomp_RF <= instr_decomp_ID;
            minstret_RF     <= minstret_ID;
            imm_RF          <= imm_ID;
            rf_rd1_addr_RF  <= rf_rd1_addr_ID;
            rf_rd2_addr_RF  <= rf_rd2_addr_ID;
            rf_rd2_used_RF  <= rf_rd2_used_ID;
            wb_addr_RF      <= wb_addr_ID;
            sel_br_RF       <= sel_br_ID;
            sel_alu_op_RF   <= sel_alu_op_ID;
            sel_alu_opa_RF  <= sel_alu_opa_ID;
            sel_alu_opb_RF  <= sel_alu_opb_ID;
            sel_md_op_RF    <= sel_md_op_ID;
            sel_csr_wr_RF   <= sel_csr_wr_ID;
            sel_csr_op_RF   <= sel_csr_op_ID;
            opa_signed_RF   <= opa_signed_ID;
            opb_signed_RF   <= opb_signed_ID;
            wen_csr_RF      <= wen_csr_ID;
            ren_csr_RF      <= ren_csr_ID;
            csr_addr_RF     <= csr_addr_ID;
            sel_dmem_be_RF  <= sel_dmem_be_ID;
            dmem_sext_RF    <= dmem_sext_ID;
            en_dmem_wr_RF   <= en_dmem_wr_ID;
            en_dmem_rd_RF   <= en_dmem_rd_ID;
            swap_bytes_RF   <= swap_bytes_ID;
            sel_wb_RF       <= sel_wb_ID;
            en_wb_RF        <= en_wb_ID; 
            stall_rs1_RF    <= stall_rs1_ID;
            stall_rs2_RF    <= stall_rs2_ID;
            rf_rd1_data_reg <= rf_rd1_data;
            rf_rd2_data_reg <= rf_rd2_data;
          end
          rf_rd1_addr_RF_ <= rf_rd1_addr_ID;
          rf_rd2_addr_RF_ <= rf_rd2_addr_ID;
          rf_rd2_used_RF_ <= rf_rd2_used_ID;
          wb_addr_RF_     <= wb_addr_ID;
          sel_br_RF_      <= sel_br_ID;
          sel_wb_RF_      <= sel_wb_ID;
        end
      end
    end else begin
      always_comb begin
        valid_RF        = valid_ID;
        pc_RF           = pc_ID;
        pc_inc_RF       = pc_inc_ID;
        instr_name_RF   = instr_name_ID;
        instr_decomp_RF = instr_decomp_ID;
        minstret_RF     = minstret_ID;
        imm_RF          = imm_ID;
        rf_rd1_addr_RF  = rf_rd1_addr_ID;
        rf_rd2_addr_RF  = rf_rd2_addr_ID;
        rf_rd2_used_RF  = rf_rd2_used_ID;
        wb_addr_RF      = wb_addr_ID;
        sel_br_RF       = sel_br_ID;
        sel_alu_op_RF   = sel_alu_op_ID;
        sel_alu_opa_RF  = sel_alu_opa_ID;
        sel_alu_opb_RF  = sel_alu_opb_ID;
        sel_md_op_RF    = sel_md_op_ID;
        sel_csr_wr_RF   = sel_csr_wr_ID;
        sel_csr_op_RF   = sel_csr_op_ID;
        opa_signed_RF   = opa_signed_ID;
        opb_signed_RF   = opb_signed_ID;
        wen_csr_RF      = wen_csr_ID;
        ren_csr_RF      = ren_csr_ID;
        csr_addr_RF     = csr_addr_ID;
        sel_dmem_be_RF  = sel_dmem_be_ID;
        dmem_sext_RF    = dmem_sext_ID;
        en_dmem_wr_RF   = en_dmem_wr_ID;
        en_dmem_rd_RF   = en_dmem_rd_ID;
        swap_bytes_RF   = swap_bytes_ID;
        sel_wb_RF       = sel_wb_ID;
        en_wb_RF        = en_wb_ID;
        stall_rs1_RF    = stall_rs1_ID;
        stall_rs2_RF    = stall_rs2_ID;
        rf_rd1_data_reg = rf_rd1_data;
        rf_rd2_data_reg = rf_rd2_data;
      end
    end

  endgenerate

  always_comb begin

    // no bypass
    rf_rd1_data_bp_RF  = rf_rd1_data;
    // EX stage bypass
    if (p_stage_EX && bp_rs1_ex) begin
      case (sel_wb_EX)
        wb_alu    : rf_rd1_data_bp_RF = alu_out_EX;
        wb_muldiv : rf_rd1_data_bp_RF = muldiv_out_EX;
        wb_pc     : rf_rd1_data_bp_RF = pc_inc_EX;
        wb_csr    : rf_rd1_data_bp_RF = csr_rd_data_EX;
        wb_copro0 : rf_rd1_data_bp_RF = copro_out0_EX;
        wb_copro1 : rf_rd1_data_bp_RF = copro_out1_EX;
        wb_copro2 : rf_rd1_data_bp_RF = copro_out2_EX;
        default   : rf_rd1_data_bp_RF = 32'd0;
      endcase
    end
    // MA stage bypass
    else if (bp_rs1_ma) begin
      rf_rd1_data_bp_RF = rf_wr_data;
    end
    // WB stage bypass
    else if (bp_rs1_wb) begin
      rf_rd1_data_bp_RF = rf_wr_data_WB;
    end
    // PW stage bypass
    else if (bp_rs1_pw & p_stage_RF) begin
      rf_rd1_data_bp_RF = rf_wr_data_PW;
    end
    
    // no bypass
    rf_rd2_data_bp_RF  = rf_rd2_data;
    // EX stage bypass
    if (p_stage_EX && bp_rs2_ex) begin
      case (sel_wb_EX)
        wb_alu    : rf_rd2_data_bp_RF = alu_out_EX;
        wb_muldiv : rf_rd2_data_bp_RF = muldiv_out_EX;
        wb_pc     : rf_rd2_data_bp_RF = pc_inc_EX;
        wb_csr    : rf_rd2_data_bp_RF = csr_rd_data_EX;
        wb_copro0 : rf_rd2_data_bp_RF = copro_out0_EX;
        wb_copro1 : rf_rd2_data_bp_RF = copro_out1_EX;
        wb_copro2 : rf_rd2_data_bp_RF = copro_out2_EX;
        default   : rf_rd2_data_bp_RF = 32'd0;
      endcase
    end
    // MA stage bypass
    else if (bp_rs2_ma) begin
      rf_rd2_data_bp_RF = rf_wr_data;
    end
    // WB stage bypass
    else if (bp_rs2_wb) begin
      rf_rd2_data_bp_RF = rf_wr_data_WB;
    end
    // WB stage bypass
    else if (bp_rs2_pw & p_stage_RF) begin
      rf_rd2_data_bp_RF = rf_wr_data_PW;
    end

  end

  assign rf_rd1_data_RF = rf_rd1_data_bp_RF;
  assign rf_rd2_data_RF = rf_rd2_data_bp_RF;
  //assign rf_rd1_data_RF = squash_RF ? 32'b0 : rf_rd1_data_bp_RF;
  //assign rf_rd2_data_RF = squash_RF ? 32'b0 : rf_rd2_data_bp_RF;


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

  //! execusion stage
  `KEEP_HIERARCHY
  cpu_exec #(
    .p_rf_read_buf  ( 0                     ),
    .p_branch_buf   ( 0                     ),
    .p_ext_rvm      ( p_ext_rvm             ),
    .p_mul_fast     ( p_mul_fast            ),
    .p_mul_1_cycle  ( p_mul_1_cycle         )
  ) exec_stage (
    .i_clk          ( i_clk                 ),
    .i_rst          ( i_rst                 ),
    .i_en_exec      ( en_EX                 ),
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

  generate
    if (p_stage_EX) begin
      always_ff @(posedge i_clk) begin
        if (i_rst) begin
          valid_EX        <= 0;
          pc_EX           <= p_reset_vector - 4;
          pc_inc_EX       <= 0;
          instr_name_EX   <= isa_nop;
          instr_decomp_EX <= pck_isa_i::NOP;
          minstret_EX     <= 0;
          imm_EX          <= 0;
          wb_addr_EX      <= 0;
          rf_rd1_addr_EX  <= 0;
          rf_rd2_addr_EX  <= 0;
          rf_rd2_used_EX  <= 0;
          rf_rd1_data_EX  <= 0;
          rf_rd2_data_EX  <= 0;
          sel_br_EX       <= br_none;
          sel_dmem_be_EX  <= be_none;
          dmem_sext_EX    <= 0;
          en_dmem_wr_EX   <= 0;
          en_dmem_rd_EX   <= 0;
          swap_bytes_EX   <= 0;
          sel_wb_EX       <= wb_none;
          en_wb_EX        <= 0;
          alu_out_EX      <= 0;
          adder_out_EX    <= 0;
          muldiv_out_EX   <= 0;
          csr_rd_data_EX  <= 0;
          exec_done_EX    <= 0;
          branch_taken_EX <= 0;
          sel_pc_EX       <= pc_plus_4;
          copro_out0_EX   <= 0;
          copro_out1_EX   <= 0;
          copro_out2_EX   <= 0;
          stall_rs1_EX    <= 0;
          stall_rs2_EX    <= 0;
          bp_rs1_ex_EX    <= 0;
          bp_rs1_ma_EX    <= 0;
          bp_rs1_wb_EX    <= 0;
          bp_rs2_ex_EX    <= 0;
          bp_rs2_ma_EX    <= 0;
          bp_rs2_wb_EX    <= 0;
        end else begin
          if (squash_EX) begin
            valid_EX        <= 0;
            pc_EX           <= pc_EX;
            pc_inc_EX       <= pc_inc_EX;
            instr_name_EX   <= isa_nop;
            instr_decomp_EX <= pck_isa_i::NOP;
            minstret_EX     <= 0;
            imm_EX          <= 0;
            wb_addr_EX      <= 0;
            rf_rd1_addr_EX  <= 0;
            rf_rd2_addr_EX  <= 0;
            rf_rd2_used_EX  <= 0;
            rf_rd1_data_EX  <= 0;
            rf_rd2_data_EX  <= 0;
            sel_br_EX       <= br_none;
            sel_dmem_be_EX  <= be_none;
            dmem_sext_EX    <= 0;
            en_dmem_wr_EX   <= 0;
            en_dmem_rd_EX   <= 0;
            swap_bytes_EX   <= 0;
            sel_wb_EX       <= wb_none;
            en_wb_EX        <= 0;
            alu_out_EX      <= 0;
            adder_out_EX    <= 0;
            muldiv_out_EX   <= 0;
            csr_rd_data_EX  <= 0;
            exec_done_EX    <= 0;
            branch_taken_EX <= 0;
            sel_pc_EX       <= pc_none;
            copro_out0_EX   <= 0;
            copro_out1_EX   <= 0;
            copro_out2_EX   <= 0;
            stall_rs1_EX    <= 0;
            stall_rs2_EX    <= 0;
            bp_rs1_ex_EX    <= 0;
            bp_rs1_ma_EX    <= 0;
            bp_rs1_wb_EX    <= 0;
            bp_rs2_ex_EX    <= 0;
            bp_rs2_ma_EX    <= 0;
            bp_rs2_wb_EX    <= 0;
          end else if (en_EX) begin
            valid_EX        <= valid_RF;
            pc_EX           <= pc_RF;
            pc_inc_EX       <= pc_inc_RF;
            instr_name_EX   <= instr_name_RF;
            instr_decomp_EX <= instr_decomp_RF;
            minstret_EX     <= minstret_RF;
            imm_EX          <= imm_RF;
            wb_addr_EX      <= wb_addr_RF;
            rf_rd1_addr_EX  <= rf_rd1_addr_RF;
            rf_rd2_addr_EX  <= rf_rd2_addr_RF;
            rf_rd2_used_EX  <= rf_rd2_used_RF;
            rf_rd1_data_EX  <= rf_rd1_data_RF;
            rf_rd2_data_EX  <= rf_rd2_data_RF;
            sel_br_EX       <= sel_br_RF;
            copro_out0_EX   <= i_copro_out0;
            copro_out1_EX   <= i_copro_out1;
            copro_out2_EX   <= i_copro_out2;
            sel_dmem_be_EX  <= sel_dmem_be_RF;
            dmem_sext_EX    <= dmem_sext_RF;
            en_dmem_wr_EX   <= en_dmem_wr_RF;
            en_dmem_rd_EX   <= en_dmem_rd_RF;
            swap_bytes_EX   <= swap_bytes_RF;
            sel_wb_EX       <= sel_wb_RF;
            en_wb_EX        <= en_wb_RF;
            alu_out_EX      <= alu_out;
            adder_out_EX    <= adder_out;
            muldiv_out_EX   <= muldiv_out;
            csr_rd_data_EX  <= csr_rd_data;
            exec_done_EX    <= exec_done;
            branch_taken_EX <= branch_taken;
            sel_pc_EX       <= sel_pc;
            stall_rs1_EX    <= stall_rs1_RF;
            stall_rs2_EX    <= stall_rs2_RF;
            bp_rs1_ex_EX    <= bp_rs1_ex;
            bp_rs1_ma_EX    <= bp_rs1_ma;
            bp_rs1_wb_EX    <= bp_rs1_wb;
            bp_rs2_ex_EX    <= bp_rs2_ex;
            bp_rs2_ma_EX    <= bp_rs2_ma;
            bp_rs2_wb_EX    <= bp_rs2_wb;
          end
        end
      end
    end else begin
      always_comb begin
        valid_EX        = valid_RF;
        pc_EX           = pc_RF;
        pc_inc_EX       = pc_inc_RF;
        instr_name_EX   = instr_name_RF;
        instr_decomp_EX = instr_decomp_RF;
        minstret_EX     = minstret_RF;
        imm_EX          = imm_RF;
        wb_addr_EX      = wb_addr_RF;
        rf_rd1_addr_EX  = rf_rd1_addr_RF;
        rf_rd2_addr_EX  = rf_rd2_addr_RF;
        rf_rd2_used_EX  = rf_rd2_used_RF;
        rf_rd1_data_EX  = rf_rd1_data_RF;
        rf_rd2_data_EX  = rf_rd2_data_RF;
        sel_br_EX       = sel_br_RF;
        copro_out0_EX   = i_copro_out0;
        copro_out1_EX   = i_copro_out1;
        copro_out2_EX   = i_copro_out2;
        sel_dmem_be_EX  = sel_dmem_be_RF;
        dmem_sext_EX    = dmem_sext_RF;
        en_dmem_wr_EX   = en_dmem_wr_RF;
        en_dmem_rd_EX   = en_dmem_rd_RF;
        swap_bytes_EX   = swap_bytes_RF;
        sel_wb_EX       = sel_wb_RF;
        en_wb_EX        = en_wb_RF;
        alu_out_EX      = alu_out;
        adder_out_EX    = adder_out;
        muldiv_out_EX   = muldiv_out;
        csr_rd_data_EX  = csr_rd_data;
        exec_done_EX    = exec_done;
        branch_taken_EX = branch_taken;
        sel_pc_EX       = sel_pc;/*
        stall_rs1_EX    = 0;
        stall_rs2_EX    = 0;*/
        bp_rs1_ex_EX    = bp_rs1_ex;
        bp_rs1_ma_EX    = bp_rs1_ma;
        bp_rs1_wb_EX    = bp_rs1_wb;
        bp_rs2_ex_EX    = bp_rs2_ex;
        bp_rs2_ma_EX    = bp_rs2_ma;
        bp_rs2_wb_EX    = bp_rs2_wb;
      end
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
    .i_en_dmem_wr   ( en_dmem_wr_EX         ),
    .i_rf_rd2_data  ( rf_rd2_data_EX        ),
    .i_en_dmem_rd   ( en_dmem_rd_EX         ),
    .o_dbus_rd_data ( dbus_rd_data_MA       ),
    .o_dbus_busy    ( dbus_busy_MA          ),
    .o_dbus_ack     ( dbus_ack_MA           ) 
  );

  generate
    if (1) begin
      always_ff @(posedge i_clk) begin
        if (i_rst) begin
          valid_MA        <= 0;
          pc_MA           <= p_reset_vector - 4;
          pc_inc_MA       <= 0;
          instr_name_MA   <= isa_nop;
          instr_decomp_MA <= pck_isa_i::NOP;
          minstret_MA     <= 0;
          sel_pc_MA       <= pc_plus_4;
          imm_MA          <= 0;
          wb_addr_MA      <= 0;
          rf_rd1_addr_MA  <= 0;
          rf_rd2_addr_MA  <= 0;
          rf_rd2_used_MA  <= 0;
          rf_rd1_data_MA  <= 0;
          rf_rd2_data_MA  <= 0;
          sel_br_MA       <= br_none;
          alu_out_MA      <= 0;
          muldiv_out_MA   <= 0;
          csr_rd_data_MA  <= 0;
          branch_taken_MA <= 0;
          copro_out0_MA   <= 0;
          copro_out1_MA   <= 0;
          copro_out2_MA   <= 0;
          sel_wb_MA       <= wb_none;
          en_wb_MA        <= 0;
          stall_rs1_EX    <= 0;
          stall_rs2_EX    <= 0;
          bp_rs1_ex_MA    <= 0;
          bp_rs1_ma_MA    <= 0;
          bp_rs1_wb_MA    <= 0;
          bp_rs2_ex_MA    <= 0;
          bp_rs2_ma_MA    <= 0;
          bp_rs2_wb_MA    <= 0;
        end else begin
          if (en_MA) begin
            valid_MA        <= valid_EX;
            pc_MA           <= pc_EX;
            instr_name_MA   <= instr_name_EX;
            instr_decomp_MA <= instr_decomp_EX;
            minstret_MA     <= minstret_EX;
            sel_pc_MA       <= sel_pc_EX;
            imm_MA          <= imm_EX;
            wb_addr_MA      <= wb_addr_EX;
            rf_rd1_addr_MA  <= rf_rd1_addr_EX;
            rf_rd2_addr_MA  <= rf_rd2_addr_EX;
            rf_rd2_used_MA  <= rf_rd2_used_EX;
            rf_rd1_data_MA  <= rf_rd1_data_EX;
            rf_rd2_data_MA  <= rf_rd2_data_EX;
            sel_br_MA       <= sel_br_EX;
            alu_out_MA      <= alu_out_EX;
            muldiv_out_MA   <= muldiv_out_EX;
            csr_rd_data_MA  <= csr_rd_data_EX;
            branch_taken_MA <= branch_taken_EX;
            copro_out0_MA   <= copro_out0_EX;
            copro_out1_MA   <= copro_out1_EX;
            copro_out2_MA   <= copro_out2_EX;
            /*if (bp_jal) begin
              pc_inc_MA       <= pc_inc_ID;
              sel_wb_MA       <= wb_pc;
              en_wb_MA        <= 1'b1;
            end else if (bp_jalr) begin
              pc_inc_MA       <= pc_inc_EX;
              sel_wb_MA       <= wb_pc;
              en_wb_MA        <= 1'b1;
            end else begin*/
              pc_inc_MA       <= pc_inc_EX;
              sel_wb_MA       <= sel_wb_EX;
              en_wb_MA        <= en_wb_EX;
            //end
            stall_rs1_MA    <= stall_rs1_EX;
            stall_rs2_MA    <= stall_rs2_EX;
            bp_rs1_ex_MA    <= bp_rs1_ex_EX;
            bp_rs1_ma_MA    <= bp_rs1_ma_EX;
            bp_rs1_wb_MA    <= bp_rs1_wb_EX;
            bp_rs2_ex_MA    <= bp_rs2_ex_EX;
            bp_rs2_ma_MA    <= bp_rs2_ma_EX;
            bp_rs2_wb_MA    <= bp_rs2_wb_EX;
          end
        end
      end
    end else begin
      always_comb begin
        valid_MA        = valid_EX;
        pc_MA           = pc_EX;
        pc_inc_MA       = pc_inc_EX;
        instr_name_MA   = instr_name_EX;
        instr_decomp_MA = instr_decomp_EX;
        minstret_MA     = minstret_EX;
        sel_pc_MA       = sel_pc_EX;
        imm_MA          = imm_EX;
        wb_addr_MA      = wb_addr_EX;
        rf_rd1_addr_MA  = rf_rd1_addr_EX;
        rf_rd2_addr_MA  = rf_rd2_addr_EX;
        rf_rd2_used_MA  = rf_rd2_used_EX;
        rf_rd1_data_MA  = rf_rd1_data_EX;
        rf_rd2_data_MA  = rf_rd2_data_EX;
        sel_br_MA       = sel_br_EX;
        alu_out_MA      = alu_out_EX;
        muldiv_out_MA   = muldiv_out_EX;
        csr_rd_data_MA  = csr_rd_data_EX;
        branch_taken_MA = branch_taken_EX;
        copro_out0_MA   = copro_out0_EX;
        copro_out1_MA   = copro_out1_EX;
        copro_out2_MA   = copro_out2_EX;
        sel_wb_MA       = sel_wb_EX;
        en_wb_MA        = en_wb_EX;
        stall_rs1_MA    = stall_rs1_EX;
        stall_rs2_MA    = stall_rs2_EX;
        bp_rs1_ex_MA    = bp_rs1_ex_EX;
        bp_rs1_ma_MA    = bp_rs1_ma_EX;
        bp_rs1_wb_MA    = bp_rs1_wb_EX;
        bp_rs2_ex_MA    = bp_rs2_ex_EX;
        bp_rs2_ma_MA    = bp_rs2_ma_EX;
        bp_rs2_wb_MA    = bp_rs2_wb_EX;
      end
    end
  endgenerate

  //! write back stage
  `KEEP_HIERARCHY
  cpu_write_back #(
    .p_wb_buf       ( 0                     )
  ) write_back_stage (
    .i_clk          ( i_clk                 ),
    .i_rst          ( i_rst                 ),
    .i_en_wb        ( en_wb_MA              ),
    .i_sel_wb       ( sel_wb_MA             ),
    .i_rf_wr_addr   ( wb_addr_MA            ),
    .i_alu_out      ( alu_out_MA            ),
    .i_muldiv_out   ( muldiv_out_MA         ),
    .i_csr_rd_data  ( csr_rd_data_MA        ),
    .i_copro_out0   ( copro_out0_MA         ),
    .i_copro_out1   ( copro_out1_MA         ),
    .i_copro_out2   ( copro_out2_MA         ),
    .i_pc_inc       ( pc_inc_MA             ),
    .i_dbus_rd_data ( dbus_rd_data_MA       ),
    .o_rf_wr_addr   ( rf_wr_addr            ),
    .o_rf_wr_data   ( rf_wr_data            ),
    .o_rf_wr_en     ( rf_wr_en              ) 
  );

  generate
    if (1) begin
      always_ff @(posedge i_clk) begin
        if (i_rst) begin
          valid_WB        <= 0;
          pc_WB           <= p_reset_vector - 4;
          instr_name_WB   <= isa_nop;
          minstret_WB     <= 0;
          instr_decomp_WB <= pck_isa_i::NOP;
          sel_pc_WB       <= pc_plus_4;
          imm_WB          <= 0;
          imm_WB_         <= 0;
          rf_rd1_addr_WB  <= 0;
          rf_rd2_addr_WB  <= 0;
          rf_rd2_used_WB  <= 0;
          rf_rd1_data_WB  <= 0;
          rf_rd1_data_WB_ <= 0;
          rf_rd2_data_WB  <= 0;
          rf_wr_addr_WB   <= 0;
          rf_wr_data_WB   <= 0;
          rf_wr_en_WB     <= 0;
          alu_out_WB      <= 0;
          branch_taken_WB <= 0;
          stall_rs1_WB    <= 0;
          stall_rs2_WB    <= 0;
          bp_rs1_ex_WB    <= 0;
          bp_rs1_ma_WB    <= 0;
          bp_rs1_wb_WB    <= 0;
          bp_rs2_ex_WB    <= 0;
          bp_rs2_ma_WB    <= 0;
          bp_rs2_wb_WB    <= 0;
        end else begin
          if (en_WB) begin
            if (squash_WB) begin
              valid_WB        <= 0;
              pc_WB           <= pc_WB;
              instr_name_WB   <= isa_nop;
              instr_decomp_WB <= pck_isa_i::NOP;
              minstret_WB     <= 0;
              sel_pc_WB       <= pc_none;
              imm_WB          <= 0;
              imm_WB_         <= 0;
              rf_rd1_addr_WB  <= 0;
              rf_rd2_addr_WB  <= 0;
              rf_rd2_used_WB  <= 0;
              rf_rd1_data_WB_ <= 0;
              rf_rd1_data_WB  <= 0;
              rf_rd2_data_WB  <= 0;
              rf_wr_addr_WB   <= 0;
              rf_wr_data_WB   <= 0;
              rf_wr_en_WB     <= 0;
              alu_out_WB      <= 0;
              branch_taken_WB <= 0;
              stall_rs1_WB    <= 0;
              stall_rs2_WB    <= 0;
              bp_rs1_ex_WB    <= 0;
              bp_rs1_ma_WB    <= 0;
              bp_rs1_wb_WB    <= 0;
              bp_rs2_ex_WB    <= 0;
              bp_rs2_ma_WB    <= 0;
              bp_rs2_wb_WB    <= 0;
            end else begin
              valid_WB        <= valid_MA;
              pc_WB           <= pc_MA;
              instr_name_WB   <= instr_name_MA;
              instr_decomp_WB <= instr_decomp_MA;
              minstret_WB     <= minstret_MA;
              imm_WB_         <= imm_MA;
              branch_taken_WB <= branch_taken_MA;
              rf_rd1_addr_WB  <= rf_rd1_addr_MA;
              rf_rd2_addr_WB  <= rf_rd2_addr_MA;
              rf_rd1_data_WB_ <= rf_rd1_data_MA;
              rf_rd2_used_WB  <= rf_rd2_used_MA;
              rf_rd2_data_WB  <= rf_rd2_data_MA;
              rf_wr_addr_WB   <= rf_wr_addr;
              rf_wr_data_WB   <= rf_wr_data;
              rf_wr_en_WB     <= rf_wr_en;
              sel_wb_WB       <= sel_wb_MA;

              alu_out_WB      <= alu_out; // only used by jalr
              rf_rd1_data_WB  <= rf_rd1_data_MA; // unused
              if (bp_branch) begin
                // bypass from the stage just before the execution stage
                sel_pc_WB       <= pc_imm;
                jump_pc_WB      <= pc_RF;
                imm_WB          <= imm_RF;
              end else if (bp_jal) begin
                sel_pc_WB       <= pc_imm;
                jump_pc_WB      <= pc;
                imm_WB          <= imm;
              end else if (bp_jalr) begin
                sel_pc_WB       <= pc_alu;
                jump_pc_WB      <= pc;
                imm_WB          <= imm_EX;
              end else begin
                sel_pc_WB       <= pc_plus_4; // branches = bypass, so not 'sel_pc_MA'
                jump_pc_WB      <= pc_MA;
                imm_WB          <= imm_MA;
              end
              stall_rs1_WB    <= stall_rs1_MA;
              stall_rs2_WB    <= stall_rs2_MA;
              bp_rs1_ex_WB    <= bp_rs1_ex_MA;
              bp_rs1_ma_WB    <= bp_rs1_ma_MA;
              bp_rs1_wb_WB    <= bp_rs1_wb_MA;
              bp_rs2_ex_WB    <= bp_rs2_ex_MA;
              bp_rs2_ma_WB    <= bp_rs2_ma_MA;
              bp_rs2_wb_WB    <= bp_rs2_wb_MA;
            end 
          end
        end
      end
    end else begin
      always_comb begin
        valid_WB        = valid_MA;
        pc_WB           = pc_MA;
        instr_name_WB   = instr_name_MA;
        instr_decomp_WB = instr_decomp_MA;
        sel_pc_WB       = sel_pc_MA;
        imm_WB          = imm_MA;
        rf_rd1_data_WB  = rf_rd1_data_MA;
        alu_out_WB      = alu_out_MA;
        branch_taken_WB = branch_taken_MA;
      end
    end
  endgenerate


  if (p_stage_RF) begin
    always_ff @(posedge i_clk) begin
      if (i_rst) begin
        rf_wr_data_PW   <= 0;
      end else begin
        rf_wr_data_PW   <= rf_wr_data_WB;
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
 