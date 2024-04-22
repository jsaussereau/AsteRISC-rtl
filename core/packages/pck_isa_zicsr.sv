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

//! RISC-V Ziscr extension

`ifndef __PCK_ISA_ZICSR__
`define __PCK_ISA_ZICSR__

package pck_isa_zicsr;

  /******************
      Instructions
  ******************/

  localparam CSRRW   = 32'b???????_?????_?????_001_?????_1110011;
  localparam CSRRS   = 32'b???????_?????_?????_010_?????_1110011;
  localparam CSRRC   = 32'b???????_?????_?????_011_?????_1110011;
  localparam CSRRWI  = 32'b???????_?????_?????_101_?????_1110011;
  localparam CSRRSI  = 32'b???????_?????_?????_110_?????_1110011;
  localparam CSRRCI  = 32'b???????_?????_?????_111_?????_1110011;

  /******************
       Addresses
  ******************/

  // Machine Information Registers
  localparam ISA_CSR_MVENDOR_ID     = 12'hf11;
  localparam ISA_CSR_MARCHID        = 12'hf12;
  localparam ISA_CSR_MIMPID         = 12'hf13;
  localparam ISA_CSR_MHARTID        = 12'hf14;
  
  // Machine Trap Setup
  localparam ISA_CSR_MSTATUS        = 12'h300;
  localparam ISA_CSR_ISA            = 12'h301;
  localparam ISA_CSR_MIE            = 12'h304;

  // Machine Trap Handling
  localparam ISA_CSR_MEPC           = 12'h341;
  localparam ISA_CSR_MCAUSE         = 12'h342;
  localparam ISA_CSR_MTVAL          = 12'h343;
  localparam ISA_CSR_MIP            = 12'h344;

  //  User Counter/Timers
  localparam ISA_CSR_CYCLE          = 12'hc00;
  localparam ISA_CSR_TIME           = 12'hc01;
  localparam ISA_CSR_INSTRET        = 12'hc02;
  localparam ISA_CSR_CYCLEH         = 12'hc80;
  localparam ISA_CSR_TIMEH          = 12'hc81;
  localparam ISA_CSR_INSTRETH       = 12'hc82;

  // Machine Counter/Timers
  localparam ISA_CSR_MCYCLE         = 12'hb00;
  localparam ISA_CSR_MINSTRET       = 12'hb02;
  localparam ISA_CSR_MCYCLEH        = 12'hb80;
  localparam ISA_CSR_MINSTRETH      = 12'hb82;

  localparam ISA_CSR_TEST           = 12'hbeb;

endpackage

`endif // __PCK_ISA_ZICSR__
