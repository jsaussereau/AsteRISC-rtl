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


`ifndef __SOC_SP_RAM__
`define __SOC_SP_RAM__

module soc_sp_ram #(
  parameter p_addr_base = 32'h10000000,
  parameter p_addr_mask = 32'hfffff000,
  parameter p_init_mem  = 0,
  parameter p_init_file = "init_file.hex",
  parameter p_init_val  = 32'h00000000,
  parameter p_depth_pw2 = 13   //! depth of the RAM in power of two (number of 32-bit words)
)(
  input  wire         i_clk,      //! global clock
  input  wire         i_rst,      //! global reset
  input  wire  [31:2] i_addr,     //! write address
  input  wire  [ 3:0] i_be,       //! write byte enable
  input  wire         i_wr_en,    //! write enable
  input  wire  [31:0] i_wr_data,  //! write data
  input  wire         i_rd_en,    //! read enable
  output wire  [31:0] o_rd_data,  //! read data
  output wire         o_busy,     //! memory busy
  output wire         o_ack       //! transfer acknowledge
);

  logic [31:0] mem [0:2**p_depth_pw2-1]; //! memory declaration

  logic [31:0] rd_data;

  logic [31:2] masked_addr;
  logic [p_depth_pw2-1:0] addr;

  assign masked_addr = i_addr & ~p_addr_mask[31:2];
  assign addr        = masked_addr[p_depth_pw2-1+2:2];

  always_ff @(posedge i_clk) begin: write_port
    if (i_rst) begin
      rd_data <= 32'd0;
    end else begin
      if (i_wr_en) begin
        if (i_be[0]) mem[addr][ 7: 0] <= i_wr_data[ 7: 0];
        if (i_be[1]) mem[addr][15: 8] <= i_wr_data[15: 8];
        if (i_be[2]) mem[addr][23:16] <= i_wr_data[23:16];
        if (i_be[3]) mem[addr][31:24] <= i_wr_data[31:24];
        rd_data <= rd_data;
      end else begin
        if (i_rd_en) begin
          rd_data <= mem[addr];
        end else begin
          rd_data <= rd_data;
        end
      end
    end
  end

  assign o_rd_data = rd_data;

  assign o_busy = 1'b0;
  assign o_ack = i_wr_en | i_rd_en;

endmodule

`endif // __SOC_SP_RAM__
