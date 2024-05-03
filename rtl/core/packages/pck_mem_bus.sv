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

//! memory bus

`ifndef __PCK_MEM_BUS__
`define __PCK_MEM_BUS__

interface mem_bus;

  logic [31: 0] addr;       //! bus address
  logic [ 3: 0] be;         //! bus write byte enable
  logic         wr_en;      //! bus write enable
  logic [31: 0] wr_data;    //! bus write data
  logic         rd_en;      //! bus read enable
  logic [31: 0] rd_data;    //! bus read data
  logic         busy;       //! bus busy
  logic         ack;        //! bus transfer acknowledge

  modport master(
    input  rd_data, busy, ack,
    output addr, be, wr_en, wr_data, rd_en
  );

  modport slave(
    input  addr, be, wr_en, wr_data, rd_en,
    output rd_data, busy, ack
  );

endinterface

`endif // __PCK_MEM_BUS__
