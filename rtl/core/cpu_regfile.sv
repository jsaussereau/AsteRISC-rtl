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
 
// Feel free to use your regfile implementations instead of the ones below 

`ifndef __REGFILE__
`define __REGFILE__

`ifdef VIVADO
  `include "../soc/soc_config.sv"
`else
  `include "soc/soc_config.sv"
`endif

module cpu_regfile #(
  parameter p_ext_rve      = 0,   //! use RV32E extension (reduces the integer register count to 16)
  parameter p_rf_sp        = 0,   //! register file is a single port ram
  parameter p_rf_read_buf  = 0    //! register file has synchronous read
)(
  input  logic        i_clk,      //! global clock
  input  logic        i_rst,      //! global reset
  output logic        o_busy,     //! regfile is busy
  output logic        o_addr_oob, //! address out of bounds

  input  logic        i_rd1_en,   //! regfile read enable for port 1 (used only if p_rf_sp = 1)
  input  logic [ 4:0] i_rd1_addr, //! regfile read address for port 1
  output logic [31:0] o_rd1_data, //! regfile read data for port 1

  input  logic        i_rd2_en,   //! regfile read enable for port 2 (used only if p_rf_sp = 1)
  input  logic [ 4:0] i_rd2_addr, //! regfile read address for port 2
  output logic [31:0] o_rd2_data, //! regfile read data for port 2

  input  logic        i_wr_en,    //! regfile write enable
  input  logic [ 4:0] i_wr_addr,  //! regfile write address
  input  logic [31:0] i_wr_data   //! regfile write data
);

  initial begin
    assert(!p_rf_sp || p_rf_read_buf) else $error("parameter \"p_rf_sp\" cannot be enabled if \"p_rf_read_buf\" is disabled: cannot read asynchronously through two ports on a single port RAM.");
  end

  generate
    if (p_rf_sp) begin
      logic        rd1_en_saved;
      logic        rd2_en_saved;
      logic [ 4:0] rd_addr;
      logic [31:0] rd_data;

      // saved read enables
      always_ff @(posedge i_clk) begin
        rd1_en_saved <= i_rd1_en;
        rd2_en_saved <= i_rd2_en;
      end  

      // select read address 
      always_comb begin
        if (i_rd1_en) begin 
          rd_addr    = i_rd1_addr;
        end else if (i_rd2_en) begin
          rd_addr    = i_rd2_addr;
        end
      end

      // select read data
      always_comb begin
        if (rd1_en_saved) begin 
          o_rd1_data = rd_data;
        end else if (rd2_en_saved) begin
          o_rd2_data = rd_data;
        end
      end  
      
      // single port RAM regfile 
      `KEEP_HIERARCHY
      cpu_regfile_sync_1r1w #(
        .p_half_regfile   ( p_ext_rve       )
      ) regfile (
        .i_clk            ( i_clk           ),
        .i_rst            ( i_rst           ),
        .o_busy           ( o_busy          ),
        .o_addr_oob       ( o_addr_oob      ),
        .i_rd_addr        ( rd_addr         ),
        .o_rd_data        ( rd_data         ),
        .i_wr_en          ( i_wr_en         ),
        .i_wr_addr        ( i_wr_addr       ),
        .i_wr_data        ( i_wr_data       ) 
      );
    end else begin
      if (p_rf_read_buf) begin
        // dual port RAM regfile 
        `KEEP_HIERARCHY
        cpu_regfile_sync_2r1w #(
          .p_half_regfile ( p_ext_rve       )
        ) regfile (
          .i_clk          ( i_clk           ),
          .i_rst          ( i_rst           ),
          .o_busy         ( o_busy          ),
          .o_addr_oob     ( o_addr_oob      ),
          .i_rd1_addr     ( i_rd1_addr      ),
          .o_rd1_data     ( o_rd1_data      ),
          .i_rd2_addr     ( i_rd2_addr      ),
          .o_rd2_data     ( o_rd2_data      ),
          .i_wr_en        ( i_wr_en         ),
          .i_wr_addr      ( i_wr_addr       ),
          .i_wr_data      ( i_wr_data       ) 
        );
      end else begin
        // dual port asynchronous read regfile 
        `KEEP_HIERARCHY
        cpu_regfile_async_2r1w #(
          .p_half_regfile ( p_ext_rve       )
        ) regfile (
          .i_clk          ( i_clk           ),
          .i_rst          ( i_rst           ),
          .o_busy         ( o_busy          ),
          .o_addr_oob     ( o_addr_oob      ),
          .i_rd1_addr     ( i_rd1_addr      ),
          .o_rd1_data     ( o_rd1_data      ),
          .i_rd2_addr     ( i_rd2_addr      ),
          .o_rd2_data     ( o_rd2_data      ),
          .i_wr_en        ( i_wr_en         ),
          .i_wr_addr      ( i_wr_addr       ),
          .i_wr_data      ( i_wr_data       ) 
        );
      end
    end
  endgenerate

endmodule

`endif // __REGFILE__
