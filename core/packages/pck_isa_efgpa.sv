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

//! eFPGA extension

`ifndef __PCK_ISA_EFPGA__
`define __PCK_ISA_EFPGA__

package pck_decode_efpga;

  localparam EFPGA0  = 32'b???????_?????_?????_110_?????_1110011;
  localparam EFPGA1  = 32'b???????_?????_?????_111_?????_1110011;
  localparam EFPGA3  = 32'b???????_?????_?????_111_?????_1110011;

endpackage

`endif // __PCK_ISA_EFPGA__
