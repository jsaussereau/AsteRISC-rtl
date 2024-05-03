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

`ifndef __PERIPHERAL_PWM__
`define __PERIPHERAL_PWM__

`ifdef VIVADO
  `include "../packages/pck_memory_map.sv"
  `include "../packages/pck_registers.sv"
`else
  `include "soc/packages/pck_memory_map.sv"
  `include "soc/packages/pck_registers.sv"
`endif

module perif_pwm
  import pck_memory_map::*;
#(
  parameter p_cmp_width = 16 // /!\ 16 bits max
)(
  input  wire         i_clk,             //! global clock
  input  wire         i_rst,             //! global reset
  input  wire  [ 9:2] i_addr,            //! read/write address
  input  wire  [ 3:0] i_be,              //! write byte enable
  input  wire         i_wr_en,           //! write enable
  input  wire  [31:0] i_wr_data,         //! write data
  input  wire         i_rd_en,           //! read enable
  output logic [31:0] o_rd_data,         //! read data
  output logic        o_busy,            //! busy
  output logic        o_ack,             //! transfer acknowledge

  output logic [ 3:0] o_pwm_irq,
  output logic [ 3:0] o_pwm_out
);

  initial begin
    assert(p_cmp_width <= 16) else $error("parameter \"p_cmp_width\" of module \"pwm\" must be less than or equal to 16");
  end

  /******************
       Registers
  ******************/

  pwm_registers #(
    .p_cmp_width(p_cmp_width) 
  ) regs ();

  localparam PWM_MAX_SCALE = 2**4 - 1; //pwm_config.scale -> 4 bits wide

  
  /******************
        Signals
  ******************/

  logic reset_counter;
  logic compare_match0;
  logic compare_match1;
  logic compare_match2;
  logic compare_match3;
  logic keep_value;

  logic end_divide;

  logic [3:0] pwm_out;

  
  // reset 
  always_comb begin
    if (regs.pwm_counter.value >= 2**31 - 1 || (regs.pwm_config.zero_cmp && compare_match0 && end_divide)) begin
      reset_counter = 1'b1;
    end else begin
      reset_counter = 1'b0;
    end
  end

  // pwm scaled count
  always_comb begin
    //regs.pwm_scaled_counter.value = regs.pwm_counter.value[regs.pwm_config.scale+p_cmp_width-1 : regs.pwm_config.scale]; // Range selection [A:B] -> A and B must be constants
    regs.pwm_scaled_counter.value = regs.pwm_counter.value[regs.pwm_config.scale +: p_cmp_width]; // part-select addressing
    end_divide = ((regs.pwm_counter.value & 2**regs.pwm_config.scale-1) == 2**regs.pwm_config.scale-1) ? 1'b1 : 1'b0;
  end

  // deglitch and sticky
  always_ff @(posedge i_clk) begin
    keep_value <= regs.pwm_config.sticky | (~reset_counter & regs.pwm_config.deglitch);
  end

  // compare 
  always_comb begin
    compare_match0 = (regs.pwm_scaled_counter.value >= regs.pwm_compare0.value) ? 1'b1 : 1'b0;
    compare_match1 = (regs.pwm_scaled_counter.value >= regs.pwm_compare1.value) ? 1'b1 : 1'b0;
    compare_match2 = (regs.pwm_scaled_counter.value >= regs.pwm_compare2.value) ? 1'b1 : 1'b0;
    compare_match3 = (regs.pwm_scaled_counter.value >= regs.pwm_compare3.value) ? 1'b1 : 1'b0;
  end

  // interrupt pending
  always_ff @(posedge i_clk) begin
    if (i_rst) begin 
      regs.pwm_config.compare0_ip <= 0;
      regs.pwm_config.compare1_ip <= 0;
      regs.pwm_config.compare2_ip <= 0;
      regs.pwm_config.compare3_ip <= 0;
      o_pwm_irq <= 0;
    end else begin
      regs.pwm_config.compare0_ip <= compare_match0 | (keep_value & regs.pwm_config.compare0_ip);
      regs.pwm_config.compare1_ip <= compare_match1 | (keep_value & regs.pwm_config.compare1_ip);
      regs.pwm_config.compare2_ip <= compare_match2 | (keep_value & regs.pwm_config.compare2_ip);
      regs.pwm_config.compare3_ip <= compare_match3 | (keep_value & regs.pwm_config.compare3_ip);
      o_pwm_irq[0] <= compare_match0;
      o_pwm_irq[1] <= compare_match1;
      o_pwm_irq[2] <= compare_match2;
      o_pwm_irq[3] <= compare_match3;
    end
  end

  // map to GPIO with optionnal ganging for arbitrary PWM waveform
  always_comb begin
    pwm_out[0] = regs.pwm_config.compare0_ip & ~(regs.pwm_config.compare0_gang & regs.pwm_config.compare1_ip);
    pwm_out[1] = regs.pwm_config.compare1_ip & ~(regs.pwm_config.compare1_gang & regs.pwm_config.compare2_ip);
    pwm_out[2] = regs.pwm_config.compare2_ip & ~(regs.pwm_config.compare2_gang & regs.pwm_config.compare3_ip);
    pwm_out[3] = regs.pwm_config.compare3_ip & ~(regs.pwm_config.compare3_gang & regs.pwm_config.compare0_ip);
  end

  // complement output
  always_comb begin
    o_pwm_out[0] = regs.pwm_config.compare0_complement ? ~pwm_out[0] : pwm_out[0];
    o_pwm_out[1] = regs.pwm_config.compare1_complement ? ~pwm_out[1] : pwm_out[1];
    o_pwm_out[2] = regs.pwm_config.compare2_complement ? ~pwm_out[2] : pwm_out[2];
    o_pwm_out[3] = regs.pwm_config.compare3_complement ? ~pwm_out[3] : pwm_out[3];
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
      regs.pwm_config.compare3_gang       <= 0;
      regs.pwm_config.compare2_gang       <= 0;
      regs.pwm_config.compare1_gang       <= 0;
      regs.pwm_config.compare0_gang       <= 0;
      regs.pwm_config.compare3_complement <= 0;
      regs.pwm_config.compare2_complement <= 0;
      regs.pwm_config.compare1_complement <= 0;
      regs.pwm_config.compare0_complement <= 0;
      regs.pwm_config.compare3_center     <= 0;
      regs.pwm_config.compare2_center     <= 0;
      regs.pwm_config.compare1_center     <= 0;
      regs.pwm_config.compare0_center     <= 0;
      regs.pwm_config.en_oneshot          <= 0;
      regs.pwm_config.en_always           <= 0;
      regs.pwm_config.deglitch            <= 1;
      regs.pwm_config.zero_cmp            <= 0;
      regs.pwm_config.sticky              <= 1;
      regs.pwm_config.scale               <= 0;
    
      regs.pwm_counter.value              <= 0;
    
      regs.pwm_compare0.value             <= 1;
      regs.pwm_compare1.value             <= 1;
      regs.pwm_compare2.value             <= 1;
      regs.pwm_compare3.value             <= 1;
    end else begin  

      // counter
      if (reset_counter) begin
        regs.pwm_counter.value     <= 0;
        regs.pwm_config.en_oneshot <= 0; // hardware reset to prevent a second cycle
      end else if (regs.pwm_config.en_always || regs.pwm_config.en_oneshot) begin
        regs.pwm_counter.value     <= regs.pwm_counter.value + 1;
      end

      wr_ack <= 1'b1;
      if (i_wr_en) begin
        case (i_addr)
          PWM_CONFIG_OFFSET: begin
            if (i_be[3]) regs.pwm_config  [27:24] <= i_wr_data[27:24];
            if (i_be[2]) regs.pwm_config  [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.pwm_config  [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.pwm_config  [ 7: 0] <= i_wr_data[ 7: 0];
          end
          PWM_COUNTER_OFFSET: begin
            if (i_be[3]) regs.pwm_counter [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.pwm_counter [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.pwm_counter [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.pwm_counter [ 7: 0] <= i_wr_data[ 7: 0];
          end
          PWM_COMPARE0_OFFSET: begin
            if (i_be[3]) regs.pwm_compare0[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.pwm_compare0[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.pwm_compare0[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.pwm_compare0[ 7: 0] <= i_wr_data[ 7: 0];
          end
          PWM_COMPARE1_OFFSET: begin
            if (i_be[3]) regs.pwm_compare1[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.pwm_compare1[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.pwm_compare1[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.pwm_compare1[ 7: 0] <= i_wr_data[ 7: 0];
          end
          PWM_COMPARE2_OFFSET: begin
            if (i_be[3]) regs.pwm_compare2[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.pwm_compare2[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.pwm_compare2[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.pwm_compare2[ 7: 0] <= i_wr_data[ 7: 0];
          end
          PWM_COMPARE3_OFFSET: begin
            if (i_be[3]) regs.pwm_compare3[31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.pwm_compare3[23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.pwm_compare3[15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.pwm_compare3[ 7: 0] <= i_wr_data[ 7: 0];
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
          PWM_CONFIG_OFFSET         : rd_data <= {
                                        regs.pwm_config[31:16],
                                        2'd0, //reserved2
                                        regs.pwm_config[13:12],
                                        1'd0, //reserved1
                                        regs.pwm_config[10:8],
                                        4'd0, //reserved0
                                        regs.pwm_config[3:0]
                                      };
          PWM_COUNTER_OFFSET        : rd_data[p_cmp_width+PWM_MAX_SCALE-1: 0] <= regs.pwm_counter.value;
          PWM_SCALED_COUNTER_OFFSET : rd_data[p_cmp_width-1: 0] <= regs.pwm_scaled_counter.value;
          PWM_COMPARE0_OFFSET       : rd_data[p_cmp_width-1: 0] <= regs.pwm_compare0.value;
          PWM_COMPARE1_OFFSET       : rd_data[p_cmp_width-1: 0] <= regs.pwm_compare1.value;
          PWM_COMPARE2_OFFSET       : rd_data[p_cmp_width-1: 0] <= regs.pwm_compare2.value;
          PWM_COMPARE3_OFFSET       : rd_data[p_cmp_width-1: 0] <= regs.pwm_compare3.value;
          default: rd_ack <= 1'b0;
        endcase
      end
    end
  end
  assign o_rd_data = rd_data;

endmodule

`endif // __PERIPHERAL_PWM
