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

`ifndef __CPU_STATIC_BRANCH_PREDICTOR__
`define __CPU_STATIC_BRANCH_PREDICTOR__

`ifdef VIVADO
 `include "packages/pck_control.sv"
 `include "packages/pck_isa.sv"
`else
 `include "core/packages/pck_control.sv"
 `include "core/packages/pck_isa.sv"
`endif

module cpu_static_branch_predictor
  import pck_control::*;
  import pck_isa::*;
(
  input  wire          i_cond_branch,     //! conditionnal branch (inconditionnal branches jal and jalr are excluded)
  input  wire          i_branch_instr,    //! branch instruction
  input  wire          i_jalr_instr,      //! jalr instruction
  input  wire  [31: 0] i_imm,             //! immediate value
  input  wire  [31: 0] i_pc,              //! program counter
  output wire  [31: 0] o_predicted_pc     //! predicted pc value
);
  
  logic [31: 0] predicted_pc;

  always_comb begin: program_counter
    if (i_branch_instr) begin
      if (i_jalr_instr) begin
        predicted_pc = i_pc + 4;
      // if backward branch (i_imm < 0): predict branch is taken 
      end else if (!i_cond_branch || $signed(i_imm) < 0) begin
        predicted_pc = i_pc + $signed(i_imm);
      end else begin
        predicted_pc = i_pc + 4;
      end
    end else begin
      predicted_pc = i_pc + 4;
    end
  end

  assign o_predicted_pc = predicted_pc;

endmodule

`endif // __CPU_STATIC_BRANCH_PREDICTOR__
