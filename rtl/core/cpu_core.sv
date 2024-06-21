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

//! CPU core

`ifndef __CPU_CORE__
`define __CPU_CORE__

`ifdef VIVADO
  `include "../soc/soc_config.sv"
  `include "packages/pck_control.sv"
  `include "packages/pck_isa.sv"
  `include "packages/pck_mem_bus.sv"
`else
  `include "soc/soc_config.sv"
  `include "core/packages/pck_control.sv"
  `include "core/packages/pck_isa.sv"
  `include "core/packages/pck_mem_bus.sv"
`endif

module cpu_core 
  import pck_control::*;
  import pck_isa::*;
#( // parameters:
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

  initial begin
    assert(!p_rf_sp || p_rf_read_buf) else $error("parameter \"p_rf_sp\" cannot be enabled if \"p_rf_read_buf\" is disabled: cannot read asynchronously through two ports on a single port RAM.");
  end

  // fsm
  wire          exec_done;                //! execute stage done
  wire          dmem_wr;                  //! data memory write
  wire          dmem_rd;                  //! data memory read
  wire          wb;                       //! write back
  wire          en_fetch;                 //! fetch next instruction
  wire          update_pc;                //! update program counter
  wire          en_decomp;                //! decompression enable
  wire          en_decode;                //! decode enable
  wire          en_dmem_wr;               //! data memory write enable
  wire          en_dmem_rd;               //! data memory read enable
  wire          en_exec;                  //! enable execution
  wire          en_wb;                    //! register file write enable. if p_wb_buf and p_decode_buf are enabled, the decode stage will be used for wb
  wire          wb_state;                 //! fsm currently in wb state
  wire          cond_branch;              //! conditionnal branch
 wire          jump_reg;                 //! jump register

  // regfile
  wire          rf_rd1_en;                //! read first register (used only if p_rf_sp = 1)
  wire  [ 4: 0] rf_rd1_addr;              //! regfile read address for port 1
  wire  [31: 0] rf_rd1_data;              //! regfile read data for port 1
  wire          rf_rd2_en;                //! read second register (used only if p_rf_sp = 1)
  wire  [ 4: 0] rf_rd2_addr;              //! regfile read address for port 2
  logic         rf_rd2_used;              //! register file read port 2 is used
  wire  [31: 0] rf_rd2_data;              //! regfile read data for port 2
  wire          rf_wr_en;                 //! regfile  write enable
  wire  [ 4: 0] rf_wr_addr;               //! regfile write address
  wire  [31: 0] rf_wr_data;               //! regfile write data
  wire          rf_busy;                  //! regfile is busy
  wire          rf_addr_oob;              //! regfile address out of bounds //TODO: handle this exception

  // fetch
  logic [31: 0] pc;                       //! program counter
  logic [31: 0] pc_inc;                   //! program +4 / +2
  sel_pc_e      sel_pc;                   //! program counter select
  isa_instr_t   instr;                    //! instruction from 'fetch' to 'decompress' stage

  // decompress
  wire          half_pc_addr;             //! high when the pc is not a multiple of 4
  wire          half_npc_addr;            //! high when the next pc is not a multiple of 4
  isa_instr_t   instr_decomp;             //! instruction from 'decompress' to 'decode' stage
  wire          compressed;               //! instruction is compressed
  wire          update_comp;              //! update decompressor fsm
  wire          bypass_decomp;            //! bypass the decompressor
  wire          offset_pc;                //! force fetching next pc
  wire          refetch;                  //!
  wire          freeze_pc;                //!

  // decode
  isa_instr_e   instr_name;               //! instruction name (debug)
  logic [31: 0] imm;                      //! immediate value
  sel_br_e      sel_br;                   //! program counter mux selector
  wire  [ 4: 0] wb_addr;                  //! write back address
  wire  [31: 0] imm_bp;                   //! immediate value (unregistered)
  wire          br_instr_bp;              //! conditionnal branch (inconditionnal branches jal and jalr are excluded) (unregistered)
  wire          cond_br_bp;               //! branch instruction (unregistered)
  wire          jalr_instr_bp;            //! jalr instruction (unregistered)

  // exec
  sel_alu_op_e  sel_alu_op;               //! ALU operation selector
  sel_alu_opa_e sel_alu_opa;              //! ALU operand A selector
  sel_alu_opb_e sel_alu_opb;              //! ALU operand B selector
  sel_md_op_e   sel_md_op;                //! MULDIV operation selector
  sel_csr_wr_e  sel_csr_wr;               //! csr write data selector
  sel_csr_op_e  sel_csr_op;               //! csr write operation selector

  wire          opa_signed;               //! MULDIV operand A is signed
  wire          opb_signed;               //! MULDIV operand B is signed
  logic [31: 0] alu_out;                  //! ALU output
  logic [31: 0] adder_out;                //! adder output
  logic [31: 0] muldiv_out;               //! MUL/DIV result
  logic [31: 0] dmem_rd_data;             //TODO: connect to memory
  logic         branch_taken;             //! a branch is taken

  // mem
  sel_be_e      sel_dmem_be;              //! dmem address mode (word, byte, halfbyte)
  wire          dmem_sext;                //! sign extension on non-word dmem data
  //logic         dbus_busy;                //! data bus busy
  //logic         dbus_ack;                 //! data bus acknowledge
  logic [31: 0] dbus_rd_data_al;          //! data bus read data
  wire          swap_bytes;               //! swap bytes (big endian -> little endian)

  // write back
  sel_wb_e      sel_wb;                   //! write back source selector

  // csr
  wire          wen_csr;                  //! csr write enable
  wire          ren_csr;                  //! csr read enable
  logic [11: 0] csr_addr;
  logic [63: 0] minstret;
  logic [31: 0] csr_rd_data;              //! read data from csr

  // debug signals used for log
  wire          debug_valid      = 1'b1;
  wire  [31: 0] debug_imm        = imm;
  wire  [31: 0] debug_rs1_addr   = rf_rd1_addr;
  wire  [31: 0] debug_rs1_data   = rf_rd1_data;
  wire          debug_rs2_used   = rf_rd2_used;
  wire  [31: 0] debug_rs2_addr   = rf_rd2_addr;
  wire  [31: 0] debug_rs2_data   = rf_rd2_data;
  wire  [31: 0] debug_wb_addr    = rf_wr_addr;
  wire  [31: 0] debug_wb_data    = rf_wr_data;
  wire          debug_wb_en      = wb;
  wire          debug_br_taken   = branch_taken;
  logic [31: 0] debug_pc;
  logic [63: 0] debug_instret;
  isa_instr_e   debug_instr_name;
  wire  [31: 0] debug_instr_code = instr_decomp;
  //logic [31: 0] debug_instr_code;
  logic [31: 0] debug_pc_tmp;
  logic [63: 0] debug_instret_tmp;
  isa_instr_e   debug_instr_name_tmp;
  logic [31: 0] debug_instr_code_tmp;
  logic [31: 0] debug_pc_tmp2;
  logic [63: 0] debug_instret_tmp2;
  isa_instr_e   debug_instr_name_tmp2;
  logic [31: 0] debug_instr_code_tmp2;
  logic [31: 0] debug_pc_tmp3;
  logic [63: 0] debug_instret_tmp3;
  isa_instr_e   debug_instr_name_tmp3;
  logic [31: 0] debug_instr_code_tmp3;

  wire          debug_bp_rs1_ex  = 0;
  wire          debug_bp_rs1_ma  = 0;
  wire          debug_bp_rs1_wb  = 0;
  wire          debug_bp_rs2_ex  = 0;
  wire          debug_bp_rs2_ma  = 0;
  wire          debug_bp_rs2_wb  = 0;
  wire          debug_stall_rs1  = 0;
  wire          debug_stall_rs2  = 0;

  //TODO: rewrite this quick and dirty shenanigan
  //TODO: use the parameters or the fsm to control registers
  always_ff @(posedge i_clk) begin
    debug_pc_tmp <= pc;
    debug_instret_tmp <= minstret;
    debug_instr_code_tmp <= instr_decomp;
    debug_instr_name_tmp <= instr_name;
    debug_pc_tmp2 <= debug_pc_tmp;
    debug_instret_tmp2 <= debug_instret_tmp;
    debug_instr_name_tmp2 <= debug_instr_name_tmp;
    debug_instr_code_tmp2 <= debug_instr_code_tmp;
    debug_pc_tmp3 <= debug_pc_tmp2;
    debug_instret_tmp3 <= debug_instret_tmp2;
    debug_instr_name_tmp3 <= debug_instr_name_tmp2;
    debug_instr_code_tmp3 <= debug_instr_code_tmp2;
    if (dmem_rd) begin
      debug_pc <= debug_pc_tmp3;
      debug_instret <= debug_instret_tmp3;
      debug_instr_name <= debug_instr_name_tmp3;
      //debug_instr_code <= debug_instr_code_tmp3;
    end else begin
      debug_pc <= debug_pc_tmp2;
      debug_instret <= debug_instret_tmp2;
      debug_instr_name <= debug_instr_name_tmp2;
      //debug_instr_code <= debug_instr_code_tmp2;
    end
  end


  //! final state machine controling the stages
  `KEEP_HIERARCHY
  cpu_fsm #(
    .p_rf_sp        ( p_rf_sp         ),
    .p_rf_read_buf  ( p_rf_read_buf   ),
    .p_prefetch_buf ( p_prefetch_buf  ),
    .p_decode_buf   ( p_decode_buf    ),
    .p_mem_buf      ( p_mem_buf       ),
    .p_branch_buf   ( p_branch_buf    ),
    .p_wait_for_ack ( p_wait_for_ack  ),
    .p_wb_buf       ( p_wb_buf        )
  ) fsm (
    .i_clk          ( i_clk           ),
    .i_rst          ( i_rst           ),
    .i_sleep        ( i_sleep         ),
    .i_refetch      ( refetch         ),
    .i_exec_done    ( exec_done       ),
    .i_dmem_wr      ( dmem_wr         ),
    .i_dmem_rd      ( dmem_rd         ),
    .i_wb           ( wb              ),
    .i_rf_rd2_used  ( rf_rd2_used     ),
    .i_rf_busy      ( rf_busy         ),
    .i_sel_br       ( sel_br          ),
    .i_cond_branch  ( cond_branch     ),
    .i_jump_reg     ( jump_reg        ),
    .i_bad_predict  ( bad_predict     ),
    .i_dbus_busy    ( dbus_busy       ),
    .i_dbus_ack     ( dbus_ack        ),
    .i_ibus_busy    ( ibus_busy       ),
    .i_ibus_ack     ( ibus_ack        ),
    .o_en_fetch     ( en_fetch        ),
    .o_update_pc    ( update_pc       ),
    .o_en_decomp    ( en_decomp       ),
    .o_update_comp  ( update_comp     ),
    .o_en_decode    ( en_decode       ),
    .o_en_rf_rd1    ( rf_rd1_en       ),
    .o_en_rf_rd2    ( rf_rd2_en       ),
    .o_en_exec      ( en_exec         ),
    .o_en_dmem_wr   ( en_dmem_wr      ),
    .o_en_dmem_rd   ( en_dmem_rd      ),
    .o_en_wb        ( en_wb           ),
    .o_wb_state     ( wb_state        )
  );
  
  //! fetch stage
  `KEEP_HIERARCHY
  cpu_fetch #(
    .p_reset_vector ( p_reset_vector  ),
    .p_prefetch_buf ( p_prefetch_buf  ),
    .p_branch_buf   ( p_branch_buf    ),
    .p_counters     ( p_counters      )
  ) fetch_stage (
    .i_clk          ( i_clk           ),
    .i_rst          ( i_rst           ),
    .i_sleep        ( i_sleep         ),
    //.ibus           ( ibus            ),
    .ibus_addr      ( ibus_addr       ),
    .ibus_be        ( ibus_be         ),
    .ibus_wr_en     ( ibus_wr_en      ),
    .ibus_wr_data   ( ibus_wr_data    ),
    .ibus_rd_en     ( ibus_rd_en      ),
    .ibus_rd_data   ( ibus_rd_data    ),
    .ibus_busy      ( ibus_busy       ),
    .ibus_ack       ( ibus_ack        ),
    .i_en_fetch     ( en_fetch        ),
    .i_update_pc    ( update_pc       ),
    .i_refetch      ( refetch         ),
    .i_freeze_pc    ( freeze_pc       ),
    .i_compressed   ( compressed      ),
    .i_offset_pc    ( offset_pc       ),
    .i_wb_state     ( wb_state        ),
    .i_sel_pc       ( sel_pc          ),
    .i_alu_out      ( alu_out         ),
    .i_imm          ( imm             ),
    .i_rf_rd1_data  ( rf_rd1_data     ),
    .i_cond_br_bp   ( cond_br_bp      ),
    .i_br_instr_bp  ( br_instr_bp     ),
    .i_jalr_instr_bp( jalr_instr_bp   ),
    .i_imm_bp       ( imm_bp          ),
    .o_pc           ( pc              ),
    .o_pc_inc       ( pc_inc          ),
    .o_instr        ( instr           ),
    .o_bad_predict  ( bad_predict     ),
    .o_bypass_decomp( bypass_decomp   ),
    .o_half_pc_addr ( half_pc_addr    ),
    .o_half_npc_addr( half_npc_addr   ),
    .o_minstret     ( minstret        )
  );

  // instruction decompression
  cpu_decompressor #(
    .p_ext_rvc      ( p_ext_rvc       )
  ) decompression_stage ( 
    .i_clk          ( i_clk           ),
    .i_rst          ( i_rst           ),
    .i_en_decomp    ( en_decomp       ),
    .i_update_comp  ( update_comp     ),
    .i_bypass_decomp( bypass_decomp   ),
    .i_branch_taken ( branch_taken    ),
    .i_half_pc_addr ( half_pc_addr    ),
    .i_half_npc_addr( half_npc_addr   ),
    .i_instr        ( instr           ),
    .o_instr        ( instr_decomp    ),
    .o_offset_pc    ( offset_pc       ),
    .o_compressed   ( compressed      ),
    .o_freeze_pc    ( freeze_pc       ),
    .o_refetch      ( refetch         )
  );

  //! decode stage
  `KEEP_HIERARCHY
  cpu_decode #(
    .p_ext_rvm      ( p_ext_rvm       ),
    .p_ext_rvzicsr  ( p_ext_rvzicsr   ),
    .p_ext_custom   ( p_ext_custom    ),
    .p_decode_buf   ( p_decode_buf    )
  ) decode_stage (
    .i_clk          ( i_clk           ),
    .i_rst          ( i_rst           ),
    .i_en_decode    ( en_decode       ),
    .i_instr        ( instr_decomp    ),
    .o_instr_name   ( instr_name      ),
    .o_rf_wr_addr   ( wb_addr         ),
    .o_rf_rd1_addr  ( rf_rd1_addr     ),
    .o_rf_rd2_addr  ( rf_rd2_addr     ),
    .o_rf_rd2_used  ( rf_rd2_used     ),
    .o_csr_addr     ( csr_addr        ),
    .o_imm          ( imm             ),
    .o_sel_br       ( sel_br          ),
    .o_sel_alu_op   ( sel_alu_op      ),
    .o_sel_alu_opa  ( sel_alu_opa     ),
    .o_sel_alu_opb  ( sel_alu_opb     ),
    .o_sel_md_op    ( sel_md_op       ),
    .o_sel_csr_wr   ( sel_csr_wr      ),
    .o_sel_csr_op   ( sel_csr_op      ),
    .o_opa_signed   ( opa_signed      ),
    .o_opb_signed   ( opb_signed      ),
    .o_sel_wb       ( sel_wb          ),
    .o_sel_dmem_be  ( sel_dmem_be     ),
    .o_dmem_sext    ( dmem_sext       ),
    .o_dmem_wr      ( dmem_wr         ),
    .o_dmem_rd      ( dmem_rd         ),
    .o_swap_bytes   ( swap_bytes      ),
    .o_en_wb        ( wb              ),
    .o_wen_csr      ( wen_csr         ),
    .o_ren_csr      ( ren_csr         ),
    .o_cond_branch  ( cond_branch     ),
    .o_br_instr_bp  ( br_instr_bp     ),
    .o_cond_br_bp   ( cond_br_bp      ),
    .o_jalr_instr_bp( jalr_instr_bp   ),
    .o_imm_bp       ( imm_bp          ),
    .o_jump_reg     ( jump_reg        )
  );

  //! register file (array of processor registers)
  `KEEP_HIERARCHY
  cpu_regfile #(
    .p_ext_rve      ( p_ext_rve       ),
    .p_rf_read_buf  ( p_rf_read_buf   ),
    .p_rf_sp        ( p_rf_sp         )
  ) regfile (
    .i_clk          ( i_clk           ),
    .i_rst          ( i_rst           ),
    .o_busy         ( rf_busy         ),
    .o_addr_oob     ( rf_addr_oob     ),
    .i_rd1_en       ( rf_rd1_en       ),
    .i_rd1_addr     ( rf_rd1_addr     ),
    .o_rd1_data     ( rf_rd1_data     ),
    .i_rd2_en       ( rf_rd2_en       ),
    .i_rd2_addr     ( rf_rd2_addr     ),
    .o_rd2_data     ( rf_rd2_data     ),
    .i_wr_en        ( rf_wr_en        ),
    .i_wr_addr      ( rf_wr_addr      ),
    .i_wr_data      ( rf_wr_data      ) 
  );

  // control and status registers
  `KEEP_HIERARCHY
  cpu_csr #(
    .p_ext_rvzicsr  ( p_ext_rvzicsr   )
  ) csr (
    .i_clk          ( i_clk           ),
    .i_rst          ( i_rst           ),
    .i_mcycle       ( i_mcycle        ), 
    .i_mtime        ( i_mtime         ),  
    .i_minstret     ( minstret        ),  
    .i_imm          ( imm             ),
    .i_sel_csr_wr   ( sel_csr_wr      ),
    .i_sel_csr_op   ( sel_csr_op      ),
    .i_addr         ( csr_addr        ),
    .i_wr_en        ( 1'b0            ),
    .i_rf_rd1_data  ( rf_rd1_data     ),
    .o_rd_data      ( csr_rd_data     )
  );

  //! execusion stage
  `KEEP_HIERARCHY
  cpu_exec #(
    .p_rf_read_buf  ( p_rf_read_buf   ),
    .p_branch_buf   ( p_branch_buf    ),
    .p_ext_rvm      ( p_ext_rvm       ),
    .p_mul_fast     ( p_mul_fast      ),
    .p_mul_1_cycle  ( p_mul_1_cycle   )
  ) exec_stage (
    .i_clk          ( i_clk           ),
    .i_rst          ( i_rst           ),
    .i_en_exec      ( en_exec         ),
    .i_rf_rd1_data  ( rf_rd1_data     ),
    .i_rf_rd2_data  ( rf_rd2_data     ),
    .i_imm          ( imm             ),
    .i_pc           ( pc              ),
    .i_dmem_rd_data ( 32'd0/*dmem_rd_data*/    ),
    .i_sel_wb       ( sel_wb          ),
    .i_sel_br       ( sel_br          ),
    .i_sel_alu_op   ( sel_alu_op      ),
    .i_sel_alu_opa  ( sel_alu_opa     ),
    .i_sel_alu_opb  ( sel_alu_opb     ),
    .i_sel_md_op    ( sel_md_op       ),
    .i_opa_signed   ( opa_signed      ),
    .i_opb_signed   ( opb_signed      ),
    .o_alu_op_a     ( o_alu_op_a      ),
    .o_alu_op_b     ( o_alu_op_b      ),
    .o_alu_out      ( alu_out         ),
    .o_adder_out    ( adder_out       ),
    .o_muldiv_out   ( muldiv_out      ),
    .o_exec_done    ( exec_done       ),
    .o_branch_taken ( branch_taken    ),
    .o_sel_pc       ( sel_pc          ) 
  );

  //! data merory read/write stage
  `KEEP_HIERARCHY
  cpu_mem #(
    .p_mem_buf      ( p_mem_buf       )
  ) mem_stage (
    .i_clk          ( i_clk           ),
    //.dbus           ( dbus            ),
    .dbus_addr      ( dbus_addr       ),
    .dbus_be        ( dbus_be         ),
    .dbus_wr_en     ( dbus_wr_en      ),
    .dbus_wr_data   ( dbus_wr_data    ),
    .dbus_rd_en     ( dbus_rd_en      ),
    .dbus_rd_data   ( dbus_rd_data    ),
    .dbus_busy      ( dbus_busy       ),
    .dbus_ack       ( dbus_ack        ),
    .i_swap_bytes   ( swap_bytes      ),
    .i_adder_out    ( adder_out       ),
    .i_sel_dmem_be  ( sel_dmem_be     ),
    .i_dmem_sext    ( dmem_sext       ),
    .i_en_dmem_wr   ( en_dmem_wr      ),
    .i_rf_rd2_data  ( rf_rd2_data     ),
    .i_en_dmem_rd   ( en_dmem_rd      ),
    .o_dbus_rd_data ( dbus_rd_data_al ),
    .o_dbus_busy    ( /*dbus_busy   */    ),
    .o_dbus_ack     ( /*dbus_ack    */    ) 
  );

  //! write back stage
  `KEEP_HIERARCHY
  cpu_write_back #(
    .p_wb_buf       ( p_wb_buf        )
  ) write_back_stage (
    .i_clk          ( i_clk           ),
    .i_rst          ( i_rst           ),
    .i_en_wb        ( en_wb           ),
    .i_sel_wb       ( sel_wb          ),
    .i_rf_wr_addr   ( wb_addr         ),
    .i_alu_out      ( alu_out         ),
    .i_muldiv_out   ( muldiv_out      ),
    .i_csr_rd_data  ( csr_rd_data     ),
    .i_copro_out0   ( i_copro_out0    ),
    .i_copro_out1   ( i_copro_out1    ),
    .i_copro_out2   ( i_copro_out2    ),
    .i_pc_inc       ( pc_inc          ),
    .i_dbus_rd_data ( dbus_rd_data_al ),
    .o_rf_wr_addr   ( rf_wr_addr      ),
    .o_rf_wr_data   ( rf_wr_data      ),
    .o_rf_wr_en     ( rf_wr_en        ) 
  );

endmodule

`endif // __CPU_CORE__
