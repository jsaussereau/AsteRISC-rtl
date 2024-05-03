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

`ifndef SYNTHESIS

`ifndef __TB_REGFILE__
`define __TB_REGFILE__

`timescale 1ns / 1ps


module tb_regfile;
  localparam half_period = 5;
  localparam period = 2*half_period;

  logic clk = 0;
  always #half_period clk = ~clk;

  logic [ 4:0] rd_addr0;
  logic [31:0] rd_data0;
  logic [ 4:0] rd_addr1;
  logic [31:0] rd_data1;
  logic        wr_en;
  logic [ 4:0] wr_addr;
  logic [31:0] wr_data;

  initial begin

    $display($time, " <<< Simulation starts... >>>");

    // write compare 0
    #(1*period)
    wr_addr         = 3;
    wr_data         = 42;
    wr_en           = 1;

    // write compare 1
    #(1*period)
    wr_addr         = 4;
    wr_data         = 512;
    wr_en           = 1;

    // write compare 2
    #(1*period)
    wr_en           = 0;

    #(500*period)
    $display($time, " <<< Simulation ends... >>>");

    $stop;

  end

  regfile_2r1w uut(
    .i_clk      ( clk      ),
    .i_rd_addr0 ( rd_addr0 ),
    .o_rd_data0 ( rd_data0 ),
    .i_rd_addr1 ( rd_addr1 ),
    .o_rd_data1 ( rd_data1 ),
    .i_wr_en    ( wr_en    ),
    .i_wr_addr  ( wr_addr  ),
    .i_wr_data  ( wr_data  )
  );
endmodule

`endif /* __TB_PWM__ */

`endif
