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
  input  wire  [31: 0] i_pc_inc,          //! sequential program counter
  output wire  [31: 0] o_predicted_pc,    //! predicted pc value
  output wire          o_predict_taken    //! the control transfer is predicted taken
);
  
  logic [31: 0] predicted_pc;
  logic         predict_taken;

  //! the heuristic itself: an unconditional jump whose target is known from the
  //! instruction alone is always taken, a conditional branch is taken when it
  //! goes backwards (loop), and `jalr` -- whose target needs a register value --
  //! is never predicted.
  always_comb begin: predict
    predict_taken = i_branch_instr & ~i_jalr_instr & (~i_cond_branch | i_imm[31]);
  end

  always_comb begin: program_counter
    if (predict_taken) begin
      predicted_pc = i_pc + $signed(i_imm);
    end else begin
      predicted_pc = i_pc_inc;
    end
  end

  assign o_predicted_pc  = predicted_pc;
  assign o_predict_taken = predict_taken;

endmodule

`endif // __CPU_STATIC_BRANCH_PREDICTOR__
