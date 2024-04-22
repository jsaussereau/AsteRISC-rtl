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

`ifndef __CPU_EXEC__
`define __CPU_EXEC__

`ifdef VIVADO
  `include "../soc/soc_config.sv"
  `include "packages/pck_control.sv"
`else
  `include "soc/soc_config.sv"
  `include "core/packages/pck_control.sv"
`endif

module cpu_exec 
  import pck_control::*;
#(
  parameter p_rf_read_buf  = 0,           //! register file has synchronous read
  parameter p_branch_buf   = 0,           //! add buffers to alu comp outputs (+1 cycle for conditionnal branches)
  parameter p_ext_rvm      = 1,           //! use RV32M extension (multiplication and division)
  parameter p_mul_fast     = 0,           //! fast mul
  parameter p_mul_1_cycle  = 0            //! one cycle mul
)(
  input  wire          i_clk,             //! global clock
  input  wire          i_rst,             //! global reset

  input  wire          i_en_exec,         //! enable execution

  input  wire  [31: 0] i_rf_rd1_data,     //! regfile memory read data on port 1
  input  wire  [31: 0] i_rf_rd2_data,     //! regfile memory read data on port 2
  input  wire  [31: 0] i_imm,             //! immediate value
  input  wire  [31: 0] i_pc,              //! program counter
  input  wire  [31: 0] i_dmem_rd_data,    //! DMEM read data

  input  sel_wb_e      i_sel_wb,          //! write back selector
  input  sel_br_e      i_sel_br,          //! branch type
  input  sel_alu_op_e  i_sel_alu_op,      //! ALU operation selector
  input  sel_alu_opa_e i_sel_alu_opa,     //! ALU operand A selector
  input  sel_alu_opb_e i_sel_alu_opb,     //! ALU operand B selector
  input  wire          i_opa_signed,      //! MULDIV operand A is signed
  input  wire          i_opb_signed,      //! MULDIV operand B is signed
  input  sel_md_op_e   i_sel_md_op,       //! MULDIV operation selector
  output logic [31: 0] o_alu_op_a,        //! ALU operand A
  output logic [31: 0] o_alu_op_b,        //! ALU operand B
  output logic [31: 0] o_alu_out,         //! ALU result
  output logic [31: 0] o_adder_out,       //! adder result
  output logic [31: 0] o_muldiv_out,      //! MUL/DIV result
  output logic         o_exec_done,       //! execution is done
  output logic         o_branch_taken,    //! a branch is taken
  output sel_pc_e      o_sel_pc           //! program counter select
);

  logic [31: 0] alu_op_a;                 //! ALU operand A
  logic [31: 0] alu_op_b;                 //! ALU operand B
  wire  [31: 0] alu_out;                  //! ALU comb output
  wire          null_out;                 //! ALU result is equal to 0

  logic         md_exec_done;             //! MUL/DIV execusion done: data valid
  logic         mul_en;                   //! MUL enable
  logic         div_en;                   //! DIV enable
  logic         alu_use_md;               //! use muldiv opa and opb outputs as ALU input
  logic [32: 0] alu_op_a_md;              //! ALU operand A from muldiv
  logic [32: 0] alu_op_b_md;              //! ALU operand B from muldiv
  logic [31: 0] alu_adder_out;            //! adder output
  logic [33: 0] alu_adder_fout;           //! full length adder output
  logic [ 1: 0] signed_mode;              //! operands signed/unsigned

  logic [33: 0] imd_val_d_ex[2];          //! intermediate register for multicycle ops
  logic [33: 0] imd_val_q_ex[2];          //! intermediate register for multicycle ops
  logic [ 1: 0] imd_val_we_ex;

  wire          br_cond_eq;  
  wire          br_cond_lt;  
  wire          br_cond_ltu;  
  logic         br_cond_eq_sync;  
  logic         br_cond_lt_sync;  
  logic         br_cond_ltu_sync;  
  logic         alu_null_out;
  sel_pc_e      sel_pc;

  logic [31: 0] alu_out_sync;
  logic [31: 0] alu_out_async;
  sel_pc_e      sel_pc_sync;
  sel_pc_e      sel_pc_async;

  logic [31: 0] imm;
  logic [31: 0] pc;
  sel_wb_e      sel_wb;
  sel_br_e      sel_br;
  sel_alu_op_e  sel_alu_op;
  sel_alu_opa_e sel_alu_opa;
  sel_alu_opb_e sel_alu_opb;
  logic         opa_signed;
  logic         opb_signed;
  sel_md_op_e   sel_md_op;

  //! input synchronization
  generate
    if (p_rf_read_buf) begin
      always_ff @(posedge i_clk) begin
        imm         <= i_imm;
        pc          <= i_pc;
        sel_wb      <= i_sel_wb;
        sel_br      <= i_sel_br;
        sel_alu_op  <= i_sel_alu_op;
        sel_alu_opa <= i_sel_alu_opa;
        sel_alu_opb <= i_sel_alu_opb;
        opa_signed  <= i_opa_signed;
        opb_signed  <= i_opb_signed;
        sel_md_op   <= i_sel_md_op;
      end
    end else begin
      always_comb begin
        imm         = i_imm;
        pc          = i_pc;
        sel_wb      = i_sel_wb;
        sel_br      = i_sel_br;
        sel_alu_op  = i_sel_alu_op;
        sel_alu_opa = i_sel_alu_opa;
        sel_alu_opb = i_sel_alu_opb;
        opa_signed  = i_opa_signed;
        opb_signed  = i_opb_signed;
        sel_md_op   = i_sel_md_op;
      end
    end
  endgenerate
  
  //! ALU: Arithmetic Logic Unit
  `KEEP_HIERARCHY
  cpu_alu #(
    .p_ext_rvm     ( p_ext_rvm       )
  ) alu (
    .i_op_a        ( alu_op_a        ),
    .i_op_b        ( alu_op_b        ),
    .i_op_a_md     ( alu_op_a_md     ),
    .i_op_b_md     ( alu_op_b_md     ),
    .i_use_md      ( alu_use_md      ), 
    .i_sel_op      ( sel_alu_op      ),
    .o_adder_out   ( alu_adder_out   ),
    .o_adder_fout  ( alu_adder_fout  ),
    .o_out         ( alu_out         ),
    .o_null_out    ( alu_null_out    ),
    .o_ops_eq      ( br_cond_eq      ),
    .o_ops_lt      ( br_cond_lt      ),
    .o_ops_ltu     ( br_cond_ltu     )
  );

  //! ALU op A select
  always_comb begin: alu_op_a_sel
    case (sel_alu_opa)
      opa_rf     : alu_op_a = i_rf_rd1_data;
      opa_pc     : alu_op_a = pc;
      opa_muldiv : alu_op_a = alu_op_a_md;
      default    : alu_op_a = 32'd0;
    endcase
  end
  assign o_alu_op_a = alu_op_a;

  //! ALU op B select
  always_comb begin: alu_op_b_sel
    case (sel_alu_opb)
      opb_rf     : alu_op_b = i_rf_rd2_data;
      opb_imm    : alu_op_b = imm;
      opb_muldiv : alu_op_b = alu_op_b_md;
      //opb_csrr   : alu_op_b = i_dmem_rd_data;
      default    : alu_op_b = 32'd0;
    endcase
  end
  assign o_alu_op_b = alu_op_b;

  //! exec done
  always_comb begin: execution_done
    case (sel_wb)
      wb_alu    : o_exec_done = 1'b1;
      wb_muldiv : o_exec_done = md_exec_done;
      //wb_copro0 : o_exec_done = 1'b1; //TODO: connect to coprocessor
      //wb_copro1 : o_exec_done = 1'b1; //TODO: connect to coprocessor
      //wb_copro2 : o_exec_done = 1'b1; //TODO: connect to coprocessor
      wb_pc     : o_exec_done = 1'b1;
      wb_dmem   : o_exec_done = 1'b1;
      default   : o_exec_done = 1'b1;
    endcase
  end

  generate
    if (p_branch_buf) begin
      always_ff @(posedge i_clk) begin
        br_cond_eq_sync  <= br_cond_eq;  
        br_cond_lt_sync  <= br_cond_lt;  
        br_cond_ltu_sync <= br_cond_ltu;  
      end
    end else begin
      always_comb begin
        br_cond_eq_sync  = br_cond_eq;  
        br_cond_lt_sync  = br_cond_lt;  
        br_cond_ltu_sync = br_cond_ltu;  
      end
    end
  endgenerate

  //! branch logic
  always_comb begin: branch_logic
    case (sel_br)
      br_bne  : sel_pc = (!br_cond_eq_sync ) ? pc_imm : pc_plus_4;
      br_beq  : sel_pc = ( br_cond_eq_sync ) ? pc_imm : pc_plus_4;
      br_blt  : sel_pc = ( br_cond_lt_sync ) ? pc_imm : pc_plus_4;
      br_bge  : sel_pc = (!br_cond_lt_sync ) ? pc_imm : pc_plus_4;
      br_bltu : sel_pc = ( br_cond_ltu_sync) ? pc_imm : pc_plus_4;
      br_bgeu : sel_pc = (!br_cond_ltu_sync) ? pc_imm : pc_plus_4;
      br_jal  : sel_pc = pc_imm;
      br_jalr : sel_pc = pc_alu;
      default : sel_pc = pc_plus_4;
    endcase
  end

  always_comb begin
    case (sel_br)
      br_bne  : o_branch_taken = (!br_cond_eq_sync ) ? 1'b1 : 1'b0;
      br_beq  : o_branch_taken = ( br_cond_eq_sync ) ? 1'b1 : 1'b0;
      br_blt  : o_branch_taken = ( br_cond_lt_sync ) ? 1'b1 : 1'b0;
      br_bge  : o_branch_taken = (!br_cond_lt_sync ) ? 1'b1 : 1'b0;
      br_bltu : o_branch_taken = ( br_cond_ltu_sync) ? 1'b1 : 1'b0;
      br_bgeu : o_branch_taken = (!br_cond_ltu_sync) ? 1'b1 : 1'b0;
      br_jal  : o_branch_taken = 1'b1;
      br_jalr : o_branch_taken = 1'b1;
      default : o_branch_taken = 1'b0;
    endcase
  end

  assign o_adder_out = alu_adder_out;
  assign o_alu_out = alu_out;
  assign o_sel_pc  = sel_pc;


generate
  if (p_ext_rvm) begin
    cpu_muldiv #(
      .p_mul_fast     ( p_mul_fast      ),
      .p_mul_1_cycle  ( p_mul_1_cycle   )
    ) muldiv (
      .i_clk          ( i_clk           ),
      .i_rst          ( i_rst           ),
      .i_en_exec      ( i_en_exec       ),
      .i_opa_signed   ( opa_signed      ),
      .i_opb_signed   ( opb_signed      ),
      .i_sel_md_op    ( sel_md_op       ),
      .i_op_a         ( i_rf_rd1_data   ),
      .i_op_b         ( i_rf_rd2_data   ),
      .i_alu_out      ( alu_out         ),
      .i_alu_null_out ( alu_null_out    ),
      .i_alu_adder_out( alu_adder_fout  ),
      .o_alu_op_a_md  ( alu_op_a_md     ),
      .o_alu_op_b_md  ( alu_op_b_md     ),
      .o_alu_use_md   ( alu_use_md      ),
      .o_out          ( o_muldiv_out    ),
      .o_exec_done    ( md_exec_done    )
    );
  end else begin
    always_comb begin: muldiv_dummy
      alu_op_a_md  = 33'd0;
      alu_op_b_md  = 33'd0;
      alu_use_md   = 1'b0;
      o_muldiv_out = 32'd0;
      md_exec_done = 1'b1;
    end
  end
endgenerate

endmodule

`endif // __CPU_EXEC__
