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

`ifndef __PERIPHERAL_DEBUG__
`define __PERIPHERAL_DEBUG__

`ifdef VIVADO
  `include "../packages/pck_memory_map.sv"
  `include "../packages/pck_registers.sv"
`else
  `include "soc/packages/pck_memory_map.sv"
  `include "soc/packages/pck_registers.sv"
`endif

module perif_debug 
  import pck_memory_map::*;
(
  input  wire         i_clk,             //! global clock
  input  wire         i_rst,             //! global reset
  input  wire  [ 9:2] i_addr,            //! read/write address
  input  wire  [ 3:0] i_be,              //! write byte enable
  input  wire         i_wr_en,           //! write enable
  input  wire  [31:0] i_wr_data,         //! write data
  input  wire         i_rd_en,           //! read enable
  output logic [31:0] o_rd_data,         //! read data
  output logic        o_busy,            //! busy
  output logic        o_ack              //! transfer acknowledge
);

  debug_registers regs ();

  /******************
     Registers R/W
  ******************/

  logic        wr_ack;
  logic        rd_ack;
  logic [31:0] rd_data;

  assign o_busy = 1'b0; //! module cannot be busy
  assign o_ack  = (i_wr_en & wr_ack) | (i_rd_en & rd_ack);

  always_ff @(posedge i_clk) begin: registers_write
    if (i_rst) begin
      wr_ack <= 1'b0;
    end else begin
      wr_ack <= 1'b1;
      if (i_wr_en) begin
        case (i_addr)
          DEBUG_REG0_OFFSET : begin
            if (i_be[3]) regs.reg0[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.reg0[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.reg0[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.reg0[ 7: 0] <= i_wr_data[ 7: 0];
          end
          DEBUG_REG1_OFFSET : begin
            if (i_be[3]) regs.reg1[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.reg1[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.reg1[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.reg1[ 7: 0] <= i_wr_data[ 7: 0];
          end
          DEBUG_REG2_OFFSET : begin
            if (i_be[3]) regs.reg2[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.reg2[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.reg2[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.reg2[ 7: 0] <= i_wr_data[ 7: 0];
          end
          DEBUG_REG3_OFFSET : begin
            if (i_be[3]) regs.reg3[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.reg3[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.reg3[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.reg3[ 7: 0] <= i_wr_data[ 7: 0];
          end
          DEBUG_REG4_OFFSET : begin
            if (i_be[3]) regs.reg4[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.reg4[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.reg4[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.reg4[ 7: 0] <= i_wr_data[ 7: 0];
          end
          DEBUG_REG5_OFFSET : begin
            if (i_be[3]) regs.reg5[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.reg5[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.reg5[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.reg5[ 7: 0] <= i_wr_data[ 7: 0];
          end
          DEBUG_REG6_OFFSET : begin
            if (i_be[3]) regs.reg6[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.reg6[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.reg6[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.reg6[ 7: 0] <= i_wr_data[ 7: 0];
          end
          DEBUG_REG7_OFFSET : begin
            if (i_be[3]) regs.reg7[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.reg7[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.reg7[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.reg7[ 7: 0] <= i_wr_data[ 7: 0];
          end
          default: wr_ack <= 1'b0;
        endcase     
      end
    end
  end

  always_ff @(posedge i_clk) begin: registers_read
    if (i_rst) begin
      rd_ack  <= 1'b0;
      rd_data <= 0;
    end else begin
      rd_ack  <= 1'b1;
      if (i_rd_en) begin
        rd_data <= 0;
        case (i_addr)
          DEBUG_REG0_OFFSET : rd_data <= regs.reg0;
          DEBUG_REG1_OFFSET : rd_data <= regs.reg1;
          DEBUG_REG2_OFFSET : rd_data <= regs.reg2;
          DEBUG_REG3_OFFSET : rd_data <= regs.reg3;
          DEBUG_REG4_OFFSET : rd_data <= regs.reg4;
          DEBUG_REG5_OFFSET : rd_data <= regs.reg5;
          DEBUG_REG6_OFFSET : rd_data <= regs.reg6;
          DEBUG_REG7_OFFSET : rd_data <= regs.reg7;
          default: rd_ack  <= 1'b0;
        endcase
      end
    end
  end 
  assign o_rd_data = rd_data;

endmodule

`endif // __PERIPHERAL_DEBUG__
