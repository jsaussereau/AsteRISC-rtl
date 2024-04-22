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

//! sign extension / zero padding functions

`ifndef __PCK_SEXT__
`define __PCK_SEXT__

package pck_sext;

  //! 24-bit sign extention
  function automatic logic[31:8] sext_24([31:0] instr);
    sext_24 = {24{instr[31]}};
  endfunction

  //! 20-bit sign extention
  function automatic logic[31:12] sext_20([31:0] instr);
    sext_20 = {20{instr[31]}};
  endfunction

  //! 19-bit sign extention
  function automatic logic[31:13] sext_19([31:0] instr);
    sext_19 = {19{instr[31]}};
  endfunction

  //! 16-bit sign extention
  function automatic logic[31:16] sext_16([31:0] instr);
    sext_16 = {16{instr[31]}};
  endfunction

  //! 11-bit sign extention
  function automatic logic[31:21] sext_11([31:0] instr);
    sext_11 = {11{instr[31]}};
  endfunction

endpackage

`endif // __PCK_SEXT__
