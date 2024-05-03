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

`ifndef __PERIPHERAL_IO_FUNCTION_SELECTOR__
`define __PERIPHERAL_IO_FUNCTION_SELECTOR__

`ifdef VIVADO
  `include "../packages/pck_memory_map.sv"
  `include "../packages/pck_registers.sv"
`else
  `include "soc/packages/pck_memory_map.sv"
  `include "soc/packages/pck_registers.sv"
`endif

module perif_io_function_sel
  import pck_memory_map::*;
#(
  parameter p_num_gpios = 24 //32 max
)(
  input  wire         i_clk,             //! global clock
  input  wire         i_rst,             //! global reset
  input  wire  [ 3:2] i_addr,            //! read/write address
  input  wire  [ 3:0] i_be,              //! write byte enable
  input  wire         i_wr_en,           //! write enable
  input  wire  [31:0] i_wr_data,         //! write data
  input  wire         i_rd_en,           //! read enable
  output logic [31:0] o_rd_data,         //! read data
  output logic        o_busy,            //! busy
  output logic        o_ack,             //! transfer acknowledge

  // GPIO peripheral function
  input  logic [p_num_gpios-1:0] i_gpio_out,
  output logic [p_num_gpios-1:0] o_gpio_in,
  input  logic [p_num_gpios-1:0] i_gpio_pullup,
  input  logic [p_num_gpios-1:0] i_gpio_pulldown,
  input  logic [p_num_gpios-1:0] i_gpio_out_en,

  // Hardware IO functions
  input  logic [ 3:0] i_pwm0_out,
  output logic        o_i2c_master0_scl_in,
  input  logic        i_i2c_master0_scl_out,
  input  logic        i_i2c_master0_scl_oen,
  output logic        o_i2c_master0_sda_in,
  input  logic        i_i2c_master0_sda_out,
  input  logic        i_i2c_master0_sda_oen,
  input  logic [ 7:0] i_fpga_io_out,
  output logic [ 7:0] o_fpga_io_in,
  input  logic [ 7:0] i_fpga_io_out_en,
  output logic        o_fpga_config_rx,
  input  wire         i_uart0_tx,
  output logic        o_uart0_rx,

  // GPIO Pads
  output logic [p_num_gpios-1:0] o_gpio_out,
  input  logic [p_num_gpios-1:0] i_gpio_in,
  output logic [p_num_gpios-1:0] o_gpio_pullup,
  output logic [p_num_gpios-1:0] o_gpio_pulldown,
  output logic [p_num_gpios-1:0] o_gpio_out_en
);

  localparam IO_FUNCTION0    = 1'b0;
  localparam IO_FUNCTION1    = 1'b1;
   
  localparam IO_INPUT        = 1'b0;
  localparam IO_OUTPUT       = 1'b1;
   
  localparam IO_PULLUP_OFF   = 1'b0;
  localparam IO_PULLUP_ON    = 1'b1;

  localparam IO_PULLDOWN_OFF = 1'b0;
  localparam IO_PULLDOWN_ON  = 1'b1;

  localparam IO_EMPTY_IN     = 1'b0;
  localparam IO_EMPTY_OUT    = 1'b0;


  iof_registers #(
    .p_num_gpios(p_num_gpios)
  ) regs ();

  /******************
        Select
  ******************/

  always_comb begin
    for (integer i = 0 ; i < p_num_gpios ; i++) begin
      case (i)
        // GPIO0: PWM0_out0
        0 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_pwm0_out[0];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : IO_OUTPUT;
        end 

        // GPIO1: PWM0_out1
        1 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_pwm0_out[1];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : IO_OUTPUT;
        end 

        // GPIO2: PWM0_out2
        2 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_pwm0_out[2];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : IO_OUTPUT;
        end 

        // GPIO3: PWM0_out3
        3 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_pwm0_out[3];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : IO_OUTPUT;
        end 

        // GPIO7: FPGA config rx
        7 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : IO_EMPTY_OUT;
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : IO_INPUT;

          o_fpga_config_rx     = i_gpio_in[i];
        end 

        // GPIO8: FPGA0
        8 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_fpga_io_out[0];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : i_fpga_io_out_en[0];

          o_fpga_io_in[0]      = i_gpio_in[i];
        end 

        // GPIO9: FPGA1
        9 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_fpga_io_out[1];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : i_fpga_io_out_en[1];

          o_fpga_io_in[1]      = i_gpio_in[i];
        end 

        // GPIO10: FPGA2
        10 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_fpga_io_out[2];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : i_fpga_io_out_en[2];

          o_fpga_io_in[2]      = i_gpio_in[i];
        end 

        // GPIO11: FPGA3
        11 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_fpga_io_out[3];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : i_fpga_io_out_en[3];

          o_fpga_io_in[3]      = i_gpio_in[i];
        end 

        // GPIO12: FPGA4
        12 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_fpga_io_out[4];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : i_fpga_io_out_en[4];

          o_fpga_io_in[4]      = i_gpio_in[i];
        end 

        // GPIO13: FPGA5
        13 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_fpga_io_out[5];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : i_fpga_io_out_en[5];

          o_fpga_io_in[5]      = i_gpio_in[i];
        end 

        // GPIO14: FPGA6
        14 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_fpga_io_out[6];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : i_fpga_io_out_en[6];

          o_fpga_io_in[6]      = i_gpio_in[i];
        end 

        // GPIO15: FPGA7
        15 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_fpga_io_out[7];
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : i_fpga_io_out_en[7];

          o_fpga_io_in[7]      = i_gpio_in[i];
        end

        // GPIO16: I²C master 0 SDA
        16 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_i2c_master0_sda_out;
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : i_i2c_master0_sda_oen;

          o_i2c_master0_sda_in = (regs.iof_config.sel[i] == IO_FUNCTION0) ? IO_EMPTY_IN         : i_gpio_in[i];
        end 

        // GPIO17: I²C master 0 SCL
        17 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_i2c_master0_scl_out;
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : i_i2c_master0_scl_oen;

          o_i2c_master0_scl_in = (regs.iof_config.sel[i] == IO_FUNCTION0) ? IO_EMPTY_IN         : i_gpio_in[i];
        end 

        // GPIO18: UART0 RX
        18 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : IO_EMPTY_OUT;
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : IO_INPUT;

          o_uart0_rx           = (regs.iof_config.sel[i] == IO_FUNCTION0) ? IO_EMPTY_IN         : i_gpio_in[i];
        end 

        // GPIO19: UART0 TX
        19 : begin
          o_gpio_out       [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out      [i] : i_uart0_tx;
          o_gpio_in        [i] = i_gpio_in[i];
          o_gpio_pullup    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pullup   [i] : IO_PULLUP_OFF;
          o_gpio_pulldown  [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_pulldown [i] : IO_PULLDOWN_OFF;
          o_gpio_out_en    [i] = (regs.iof_config.sel[i] == IO_FUNCTION0) ? i_gpio_out_en   [i] : IO_OUTPUT;
        end 

        default: begin
          o_gpio_out       [i] = i_gpio_out      [i];
          o_gpio_in        [i] = i_gpio_in       [i];
          o_gpio_pullup    [i] = i_gpio_pullup   [i];
          o_gpio_pulldown  [i] = i_gpio_pulldown [i];
          o_gpio_out_en    [i] = i_gpio_out_en   [i];
        end
      endcase
    end    
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
      wr_ack <= 1'b0;
      regs.iof_config.sel <= 0;
    end else begin
      wr_ack <= 1'b1;
      if (i_wr_en) begin
        case (i_addr)
          IOF_SEL_OFFSET: begin
            if (i_be[3]) regs.iof_config[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.iof_config[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.iof_config[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.iof_config[ 7: 0] <= i_wr_data[ 7: 0];
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
          IOF_SEL_OFFSET : rd_data[p_num_gpios-1: 0] <= regs.iof_config.sel;
          default: rd_ack <= 1'b0;
        endcase
      end
    end
  end
  assign o_rd_data = rd_data;

endmodule

`endif // __PERIPHERAL_IO_FUNCTION_SELECTOR__
