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

//! Single port 32-bit wide RAM with optional .hex init file for simulation

`ifdef VIVADO
  `include "../soc_config.sv"
`else
  `include "soc/soc_config.sv"
`endif

`ifdef TARGET_SIM

`ifndef __WRAP_RAM_IMEM__
`define __WRAP_RAM_IMEM__

/*verilator public_on*/
module wrap_ram_imem #(
  parameter p_addr_base = 32'hf0000000,
  parameter p_addr_mask = 32'hfffff000,
  parameter p_init_file = "init_file.hex",
  parameter p_init_val  = 32'h00000003,
  parameter p_depth_pw2 = 13   //! depth of the RAM in power of two (number of 32-bit words)
)(
  input  wire         i_clk,      //! global clock
  input  wire         i_rst,      //! global reset
  input  wire  [31:2] i_addr,     //! write address
  input  wire  [ 3:0] i_be,       //! write byte enable
  input  wire         i_wr_en,    //! write enable
  input  wire  [31:0] i_wr_data,  //! write data
  input  wire         i_rd_en,    //! read enable
  output logic [31:0] o_rd_data,  //! read data
  output logic        o_busy,     //! memory busy
  output logic        o_ack       //! transfer acknowledge
);

  /*verilator public_on*/
  soc_sp_ram #(
    .p_addr_base    ( p_addr_base                ),
    .p_addr_mask    ( p_addr_mask                ),
    .p_init_mem     ( 1                          ),
    .p_init_file    ( p_init_file                ),
    .p_init_val     ( p_init_val                 ),
    .p_depth_pw2    ( p_depth_pw2                )
  ) imem (
    .i_clk          ( i_clk                      ),
    .i_rst          ( i_rst                      ),
    .i_addr         ( i_addr                     ),
    .i_be           ( i_be                       ),
    .i_wr_en        ( i_wr_en                    ),
    .i_wr_data      ( i_wr_data                  ),
    .i_rd_en        ( i_rd_en                    ),
    .o_rd_data      ( o_rd_data                  ),
    .o_busy         ( o_busy                     ),
    .o_ack          ( o_ack                      )  
  );
  /*verilator public_off*/
  
endmodule

`endif // __WRAP_RAM_IMEM__

`endif // TARGET_SIM
