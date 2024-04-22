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

`ifndef __REGFILE_ASYNC_2R1W__
`define __REGFILE_ASYNC_2R1W__

`ifdef VIVADO
 `include "packages/pck_regfile.sv"
`else
 `include "core/packages/pck_regfile.sv"
`endif

module cpu_regfile_async_2r1w
  import pck_regfile::*;
#(
  parameter p_half_regfile = 0    //! reduce the register count to 16
)(
  input  logic        i_clk,      //! global clock
  input  logic        i_rst,      //! global reset
  output logic        o_busy,     //! regfile is busy
  output logic        o_addr_oob, //! address out of bounds

  input  logic [ 4:0] i_rd1_addr, //! regfile read address for port 1
  output logic [31:0] o_rd1_data, //! regfile read data for port 1

  input  logic [ 4:0] i_rd2_addr, //! regfile read address for port 2
  output logic [31:0] o_rd2_data, //! regfile read data for port 2

  input  logic        i_wr_en,    //! regfile write enable
  input  logic [ 4:0] i_wr_addr,  //! regfile write address
  input  logic [31:0] i_wr_data   //! regfile write data
);

  logic [ 4:0] wr_addr;
  logic [ 4:0] rd1_addr;
  logic [ 4:0] rd2_addr;
  logic        wr_en;

  localparam depth = (p_half_regfile) ? 15 : 31;

   // using a distributed regfile increases max frequency but also area
  (* ram_style = "distributed" *)
  logic [31:0] regs [0:depth];
  //regfile_t    rf;

  generate
    if (p_half_regfile) begin
      // check for out of bound exception
      always_comb begin
        wr_addr    = { 1'b0, i_wr_addr [3:0] };
        rd1_addr   = { 1'b0, i_rd1_addr[3:0] };
        rd2_addr   = { 1'b0, i_rd2_addr[3:0] };
        o_addr_oob = i_wr_addr[4] | i_rd1_addr[4] | i_rd2_addr[4];
      end
    end else begin
      always_comb begin
        wr_addr    = i_wr_addr;
        rd1_addr   = i_rd1_addr;
        rd2_addr   = i_rd2_addr;
        o_addr_oob = 1'b0;
      end
    end
  endgenerate

  assign wr_en = i_wr_en && wr_addr != 5'd0;

  //! regfile synchronous write port
  always_ff @(posedge i_clk) begin: write
    /*if (i_rst) begin
      regs[0] <= 32'b0;
    end else*/ if (wr_en) begin
      regs[wr_addr] <= i_wr_data;
    end
  end

  //! regfile asynchronous read ports
  always_comb begin: read
    // output hardwired zero if addr=0
    o_rd1_data = (rd1_addr == 5'd0) ? 32'd0 : regs[rd1_addr];
    o_rd2_data = (rd2_addr == 5'd0) ? 32'd0 : regs[rd2_addr];
  end 

  assign o_busy = 1'b0;

endmodule

`endif // __REGFILE_ASYNC_2R1W__
