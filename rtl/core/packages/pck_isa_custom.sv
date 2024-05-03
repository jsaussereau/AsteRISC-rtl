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

//! RISC-V custom instructions 

`ifndef __PCK_ISA_CUSTOM__
`define __PCK_ISA_CUSTOM__

package pck_isa_custom;

  localparam CUSTOM0 = 32'b0000000_?????_?????_000_?????_1101011; // CRC6

  localparam CUSTOM1 = 32'b0000000_?????_?????_001_?????_1101011; // UNUSED
  localparam CUSTOM2 = 32'b0000000_?????_?????_010_?????_1101011; // UNUSED

endpackage

`endif // __PCK_ISA_CUSTOM__
