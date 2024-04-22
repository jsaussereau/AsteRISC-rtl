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

//! RISC-V M extension

`ifndef __PCK_ISA_M__
`define __PCK_ISA_M__

package pck_isa_m;

  localparam MUL     = 32'b0000001_?????_?????_000_?????_0110011;
  localparam MULH    = 32'b0000001_?????_?????_001_?????_0110011;
  localparam MULSU   = 32'b0000001_?????_?????_010_?????_0110011;
  localparam MULU    = 32'b0000001_?????_?????_011_?????_0110011;
  localparam DIV     = 32'b0000001_?????_?????_100_?????_0110011;
  localparam DIVU    = 32'b0000001_?????_?????_101_?????_0110011;
  localparam REM     = 32'b0000001_?????_?????_110_?????_0110011;
  localparam REMU    = 32'b0000001_?????_?????_111_?????_0110011;

endpackage

`endif // __PCK_ISA_M__
