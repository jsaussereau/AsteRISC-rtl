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

//! GPIO periphereal

`ifndef __PERIPHERAL_GPIO__
`define __PERIPHERAL_GPIO__

`ifdef VIVADO
  `include "../packages/pck_memory_map.sv"
  `include "../packages/pck_registers.sv"
`else
  `include "soc/packages/pck_memory_map.sv"
  `include "soc/packages/pck_registers.sv"
`endif

module perif_gpio
  import pck_memory_map::*;
#(
  parameter p_num_gpios = 16
)(
  input  wire                    i_clk,             //! global clock
  input  wire                    i_rst,             //! global reset
  input  wire  [ 9:2]            i_addr,            //! read/write address
  input  wire  [ 3:0]            i_be,              //! write byte enable
  input  wire                    i_wr_en,           //! write enable
  input  wire  [31:0]            i_wr_data,         //! write data
  input  wire                    i_rd_en,           //! read enable
  output logic [31:0]            o_rd_data,         //! read data
  output logic                   o_busy,            //! busy
  output logic                   o_ack,             //! transfer acknowledge
  
  input  logic [p_num_gpios-1:0] i_gpio_in,         //! gpio read 
  output logic [p_num_gpios-1:0] o_gpio_out,        //! gpio write
  output logic [p_num_gpios-1:0] o_gpio_out_en,     //! gpio output enable
  output logic [p_num_gpios-1:0] o_gpio_pullup,     //! gpio pullup enable
  output logic [p_num_gpios-1:0] o_gpio_pulldown    //! gpio pulldown enable
);

  gpio_registers #(
    .p_num_gpios(p_num_gpios)
  ) regs ();

  //! assign input value to read register
  always_comb begin
    regs.gpio_read_value = 0;
    regs.gpio_read_value[p_num_gpios-1:0] = i_gpio_in;
  end

  //! write output if output is enabled
  always_comb begin
    for (integer i = 0 ; i < p_num_gpios ; i++) begin: gpio_assign
      if (regs.gpio_output_en.value[i]) begin
        o_gpio_out[i] = regs.gpio_write_value[i];
      end else begin
        o_gpio_out[i] = 'Z;
      end
    end
  end

  always_comb begin
    o_gpio_out_en   = regs.gpio_output_en.value;
    o_gpio_pullup   = regs.gpio_pullup_en.value;
    o_gpio_pulldown = regs.gpio_pulldown_en.value;
  end


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
      wr_ack                      <= 1'b0;
      regs.gpio_write_value.value <= 0;
      regs.gpio_output_en.value   <= 0;
      regs.gpio_pullup_en.value   <= 0;
      regs.gpio_pulldown_en.value <= 0;
    end else begin 
      wr_ack <= 1'b1;
      if (i_wr_en) begin
        case (i_addr)
          GPIO_WRITE_VALUE_OFFSET: begin
            if (i_be[3]) regs.gpio_write_value [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.gpio_write_value [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.gpio_write_value [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.gpio_write_value [ 7: 0] <= i_wr_data[ 7: 0];
          end
          GPIO_OUTPUT_EN_OFFSET: begin
            if (i_be[3]) regs.gpio_output_en   [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.gpio_output_en   [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.gpio_output_en   [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.gpio_output_en   [ 7: 0] <= i_wr_data[ 7: 0];
          end
          GPIO_PULLUP_EN_OFFSET: begin
            if (i_be[3]) regs.gpio_pullup_en   [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.gpio_pullup_en   [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.gpio_pullup_en   [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.gpio_pullup_en   [ 7: 0] <= i_wr_data[ 7: 0];
          end 
          GPIO_PULLDOWN_EN_OFFSET: begin
            if (i_be[3]) regs.gpio_pulldown_en [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.gpio_pulldown_en [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.gpio_pulldown_en [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.gpio_pulldown_en [ 7: 0] <= i_wr_data[ 7: 0];
          end
          default: wr_ack <= 1'b0;
        endcase
      end
    end
  end

  always_ff @(posedge i_clk) begin: registers_read
    if (i_rst) begin
      rd_ack <= 1'b0;
      rd_data <= 0;
    end else begin
      rd_ack <= 1'b1;
      rd_data <= rd_data;
      if (i_rd_en) begin
        rd_data <= 0;
        case (i_addr)
          GPIO_WRITE_VALUE_OFFSET : rd_data[p_num_gpios-1:0] <= regs.gpio_write_value.value;
          GPIO_READ_VALUE_OFFSET  : rd_data[p_num_gpios-1:0] <= regs.gpio_read_value.value; //read only
          GPIO_OUTPUT_EN_OFFSET   : rd_data[p_num_gpios-1:0] <= regs.gpio_output_en.value;
          GPIO_PULLUP_EN_OFFSET   : rd_data[p_num_gpios-1:0] <= regs.gpio_pullup_en.value;
          GPIO_PULLDOWN_EN_OFFSET : rd_data[p_num_gpios-1:0] <= regs.gpio_pulldown_en.value;
          default: rd_ack <= 1'b0;
        endcase
      end
    end
  end 
  assign o_rd_data = rd_data;

endmodule

`endif // __PERIPHERAL_GPIO__
