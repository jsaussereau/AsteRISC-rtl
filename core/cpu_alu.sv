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

//! Arithmetic Logic Unit: arithmetic and bitwise operations on integers.
//! Immediates are muxed into rs2 (op_b)

`ifndef __CPU_ALU__
`define __CPU_ALU__

`ifdef VIVADO
 `include "packages/pck_control.sv"
`else
 `include "core/packages/pck_control.sv"
`endif
 
module cpu_alu 
  import pck_control::*;
#(
  parameter p_ext_rvm      = 1          //! use RV32M extension (multiplication and division)
)(
  input  wire  [31: 0] i_op_a,          //! A operand
  input  wire  [31: 0] i_op_b,          //! B operand
  input  wire  [32: 0] i_op_a_md,       //! A operand from muldiv
  input  wire  [32: 0] i_op_b_md,       //! B operand from muldiv
  input  wire          i_use_md,        //! use muldiv as input
  input  sel_alu_op_e  i_sel_op,        //! operation select
  output logic [31: 0] o_adder_out,     //! full length adder output
  output logic [33: 0] o_adder_fout,    //! full length adder output
  output logic [31: 0] o_out,           //! operation output
  output logic         o_null_out,      //! result is equal to 0
  output logic         o_ops_eq,        //! A = B
  output logic         o_ops_lt,        //! A < B (signed)
  output logic         o_ops_ltu
);

  wire  [ 4: 0] shift_amount = i_op_b[ 4: 0];

  logic [31: 0] adder_out;
  logic [33: 0] adder_fout;
  logic [32: 0] op_a;
  logic [32: 0] op_b;

  logic [31: 0] out;

generate
  if (p_ext_rvm) begin
    always_comb begin: adder
      op_a = i_use_md ? i_op_a_md : { i_op_a, 1'b1 };
      op_b = i_use_md ? i_op_b_md : { i_op_b, 1'b0 };
      adder_fout = op_a + op_b;
      adder_out = adder_fout[32:1];
    end
  end else begin
    always_comb begin: adder
      op_a = 33'd0;
      op_b = 33'd0;
      adder_fout = 34'd0;
      adder_out = i_op_a + i_op_b;
    end
  end
endgenerate

  always_comb begin: opereration_mux
    case (i_sel_op)
      alu_add  : out = adder_out;
      alu_sub  : out = i_op_a - i_op_b;
      alu_and  : out = i_op_a & i_op_b;
      alu_or   : out = i_op_a | i_op_b;
      alu_xor  : out = i_op_a ^ i_op_b;
      alu_slt  : out = ($signed(i_op_a) < $signed(i_op_b)) ? 32'd1: 32'd0; 
      alu_sltu : out = (i_op_a < i_op_b) ? 32'd1 : 32'd0;
      alu_sll  : out = i_op_a << shift_amount;
      alu_srl  : out = i_op_a >> shift_amount;
      alu_sra  : out = $signed(i_op_a) >>> shift_amount;
      alu_cpa  : out = i_op_a;
      alu_cpb  : out = i_op_b;
      default  : out = 32'b0;
    endcase
  end

  assign o_out        = out;
  assign o_null_out   = out == 0 ? 1'b1 : 1'b0;
  assign o_adder_out  = adder_out;
  assign o_adder_fout = adder_fout;
  assign o_ops_eq     = i_op_a == i_op_b;
  assign o_ops_lt     = $signed(i_op_a) < $signed(i_op_b);
  assign o_ops_ltu    = i_op_a < i_op_b;

endmodule

`endif // __CPU_ALU__
