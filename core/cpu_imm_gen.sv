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

//! Immediate value generator

`ifndef __CPU_IMM_GEN__
`define __CPU_IMM_GEN__



`ifdef VIVADO
 `include "packages/pck_isa.sv"
 `include "packages/pck_sext.sv"
 `include "packages/pck_control.sv"
`else
 `include "core/packages/pck_isa.sv"
 `include "core/packages/pck_sext.sv"
 `include "core/packages/pck_control.sv"
`endif

module cpu_imm_gen
  import pck_isa::*;
  import pck_sext::*;
  import pck_control::*;
#(
  parameter p_branch_imm = 0          //! if 1, decode only immediate values from branch formats 
)(
  input  isa_instr_t  i_instr,        //! instruction containing the immediate value
  input  instr_type_e i_instr_type,   //! instruction format (R, I, S, B, U, J)
  output logic [31:0] o_imm           //! generated immediate value
);

  //! decode immediate value from instruction
  always_comb begin: decode_imm
    o_imm = 32'd0;
    case (i_instr_type)
      instr_b     : o_imm = { sext_19(i_instr), i_instr.b.imm12, i_instr.b.imm11, i_instr.b.imm10_5, i_instr.b.imm4_1, 1'b0   };
      instr_j     : o_imm = { sext_11(i_instr), i_instr.j.imm20, i_instr.j.imm19_12, i_instr.j.imm11, i_instr.j.imm10_1, 1'b0 };
      instr_i     : o_imm = { sext_20(i_instr), i_instr.i.imm                                                                 };
    endcase
    
    if (!p_branch_imm) begin
      case (i_instr_type)
        instr_iu    : o_imm = { 20'd0,            i_instr.i.imm                                                                 }; 
        instr_iucsr : o_imm = { 27'd0,            i_instr.i.rs1                                                                 }; 
        instr_s     : o_imm = { sext_20(i_instr), i_instr.s.imm11_5, i_instr.s.imm4_0                                           };
        instr_u     : o_imm = {                   i_instr.u.imm, 12'd0                                                          };
      endcase
    end
  end

endmodule

`endif // __CPU_IMM_GEN__
