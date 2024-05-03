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


`ifndef __CPU_BRANCH_PREDICTOR__
`define __CPU_BRANCH_PREDICTOR__

`ifdef VIVADO
 `include "packages/pck_control.sv"
 `include "packages/pck_isa.sv"
`else
 `include "core/packages/pck_control.sv"
 `include "core/packages/pck_isa.sv"
`endif

//FIXME: handle instruction cut between two addresses

module cpu_branch_predictor
  import pck_control::*;
  import pck_isa::*;
#(
  parameter p_reset_vector = 32'hf0000000,
  parameter p_branch_buf   = 0,           //! add buffers to alu comp outputs (+1 cycle for conditionnal branches)
  parameter p_mini_decoder = 0            //! use a mini decoder for branch prediction
)(
  input  wire          i_clk,             //! global clock
  input  wire          i_rst,             //! global reset
 
  input  wire  [31: 0] i_pc,              //! program counter
  input  isa_instr_t   i_instr,           //! instruction to be analyzed
  input  wire          i_en,              //! enable
  input  wire          i_cond_br_bp,      //! conditionnal branch (inconditionnal branches jal and jalr are excluded) (unregistered)
  input  wire          i_br_instr_bp,     //! branch instruction (/!\ jalr is excluded) (unregistered)
  input  wire          i_jalr_instr_bp,   //! jalr instruction (unregistered)
  input  wire  [31: 0] i_imm_bp,          //! immediate value (unregistered)
  output wire  [31: 0] o_predicted_pc,    //! predicted pc value
  output wire          o_branch_instr     //! branch instruction
);
  
  logic         bad_predict;
  logic         cond_br;
  logic         br_instr;
  logic         jalr_instr;
  logic [31: 0] predicted_pc;

  wire          cond_br_bp;
  wire          br_instr_bp;
  logic         jalr_instr_bp;
  wire          imm_bp;

  instr_type_e  instr_type;
  logic [31: 0] imm;

  always_comb begin
    instr_type = instr_r;

    // default values
    br_instr       = 1'b1;
    cond_br        = 1'b1;
    jalr_instr     = 1'b0;

    casez (i_instr.code)
      // branches
      pck_isa_i::BEQ     : instr_type = instr_b;
      pck_isa_i::BNE     : instr_type = instr_b;
      pck_isa_i::BLT     : instr_type = instr_b;
      pck_isa_i::BGE     : instr_type = instr_b;
      pck_isa_i::BLTU    : instr_type = instr_b;
      pck_isa_i::BGEU    : instr_type = instr_b;

      // jumps
      pck_isa_i::JAL     : begin
        instr_type       = instr_j;
        cond_br          = 1'b0;
      end
      pck_isa_i::JALR    : begin
        instr_type       = instr_i;
        cond_br          = 1'b0;
        jalr_instr       = 1'b1;
      end

      // other instructions
      default            : begin
        br_instr         = 1'b0;
        cond_br          = 1'b0;
      end
    endcase
  end

  //! IMM GEN: immediate value generator
  cpu_imm_gen #(
    .p_branch_imm ( 1          )
  ) imm_gen (
    .i_instr      ( i_instr    ),
    .i_instr_type ( instr_type ),
    .o_imm        ( imm        )
  );

  assign cond_br_bp    = p_mini_decoder ? cond_br    : i_cond_br_bp;
  assign br_instr_bp   = p_mini_decoder ? br_instr   : i_br_instr_bp;
  assign jalr_instr_bp = p_mini_decoder ? jalr_instr : i_jalr_instr_bp;
  assign imm_bp        = p_mini_decoder ? imm        : i_imm_bp;

  cpu_static_branch_predictor branch_predictor (
    .i_cond_branch  ( cond_br_bp    ),
    .i_branch_instr ( br_inst_bp    ),
    .i_jalr_instr   ( jalr_instr_bp ),
    .i_imm          ( imm_bp        ),
    .i_pc           ( i_pc          ),
    .o_predicted_pc ( predicted_pc  )
  );
  
  assign o_predicted_pc = predicted_pc;
  assign o_branch_instr = p_mini_decoder ? br_instr : i_br_instr_bp;

endmodule

`endif // __CPU_BRANCH_PREDICTOR__
