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

`ifndef __PERIPHERAL_UART_RX__
`define __PERIPHERAL_UART_RX__

module perif_uart_rx #(
  parameter p_fifo_depth = 4
)(
  input  wire         i_clk,             //! global clock
  input  wire         i_rst,             //! global reset
  input  logic        i_en,
  input  logic        i_rd_en,
  input  logic [15:0] i_baudrate,
  input  wire         i_rx_bit,
  output logic        o_empty,
  output logic [ 7:0] o_rx_data
);

  typedef enum { 
    st_idle,
    st_start,
    st_data,
    //st_parity,
    st_stop,
    st_clean
  } state_e;
  
  logic [15:0] baud_counter;
  logic [ 2:0] cycle_cnt; //8 bits total
  logic [ 7:0] rx_data;
  logic        fifo_append;
  state_e      curr_state;

  integer      fifo_pointer;
  logic        shifted;
  logic [ 7:0] fifo_data [0:p_fifo_depth-1];

  always_ff @(posedge i_clk) begin
    if (i_rst) begin
      curr_state   <= st_idle;
      baud_counter <= 0;
      cycle_cnt    <= 0;
      rx_data      <= 0;
      fifo_append  <= 1'b0;
    end else begin
      if (~i_en) begin
        curr_state   <= st_idle;
        baud_counter <= 0;
        cycle_cnt    <= 0;
        rx_data      <= 0;
        fifo_append  <= 1'b0;
      end else begin
        case (curr_state)
          //! wait until the start bit is detected
          st_idle: begin
            fifo_append  <= 1'b0;
            baud_counter <= 0;
            cycle_cnt    <= 0;
            
            if (i_rx_bit == 1'b0) begin
              curr_state <= st_start;
            end else begin
              curr_state <= st_idle;
            end
          end
          
          //! check the middle of start bit to make sure it's still low
          st_start: begin
            if (baud_counter == i_baudrate[15:1]) begin //divide i_baudrate by 2
              if (i_rx_bit == 1'b0) begin
                baud_counter <= 0;  // reset counter, found the middle
                curr_state   <= st_data;
              end else begin
                curr_state <= st_idle;
              end
            end else begin
              baud_counter <= baud_counter + 1;
              curr_state   <= st_start;
            end
          end
          
          // sample serial data 
          st_data: begin
            if (baud_counter <= i_baudrate-1) begin
              baud_counter   <= baud_counter + 1;
              curr_state     <= st_data;
            end else begin
              baud_counter       <= 0;
              rx_data[cycle_cnt] <= i_rx_bit;
              
              if (cycle_cnt < 7) begin
                cycle_cnt  <= cycle_cnt + 1;
                curr_state <= st_data;
              end else begin
                cycle_cnt  <= 0;
                curr_state <= st_stop;
              end
            end
          end
          
          // receive stop bit
          st_stop: begin
            if (baud_counter <= i_baudrate-1) begin
              baud_counter <= baud_counter + 1;
              curr_state   <= st_stop;
            end else begin
              fifo_append  <= 1'b1;///i_rx_bit; // only if it is really a stop bit
              baud_counter <= 0;
              curr_state   <= st_clean;
            end
          end
          
          //! wait 1 clock cycle
          st_clean: begin
            curr_state  <= st_idle;
            fifo_append <= 1'b0;
          end
          
          default: begin
            curr_state <= st_idle;
          end

        endcase
      end
    end
  end
  
  // update fifo
  always_ff @(posedge i_clk) begin
    if (i_rst) begin
      //fifo_data    <= '{default:8'd0};
      fifo_pointer <= 0;
      shifted      <= 0;
    end else begin
      if (fifo_append && fifo_pointer < p_fifo_depth) begin
        fifo_data[fifo_pointer] <= rx_data;
        fifo_pointer <= fifo_pointer + 1;
      end
      if (i_rd_en) begin
        if (shifted == 0) begin
          fifo_data[0] <= fifo_data[1];
          fifo_data[1] <= fifo_data[2];
          fifo_data[2] <= fifo_data[3];
          fifo_data[p_fifo_depth-1] <= 8'h0;
          fifo_pointer <= fifo_pointer - 1;
          shifted      <= 1;
        end
      end else begin
        shifted <= 0;
      end
    end
  end

  assign o_rx_data = fifo_data[0];
  assign o_empty   = fifo_pointer == 0;

endmodule 

`endif //__PERIPHERAL_UART_RX__
