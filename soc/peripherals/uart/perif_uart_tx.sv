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

`ifndef __PERIPHERAL_UART_TX__
`define __PERIPHERAL_UART_TX__

module perif_uart_tx #(
  parameter p_fifo_depth = 4
)(
  input  wire         i_clk,             //! global clock
  input  wire         i_rst,             //! global reset
  input  wire         i_en,
  input  wire         i_wr_en,
  input  wire  [15:0] i_baudrate,
  input  wire  [ 7:0] i_data_tx,  
  output logic        o_tx_full,
  output logic        o_uart_tx
);
  
  typedef enum { 
    st_idle, 
    st_start, 
    st_data, 
    st_parity,
    st_stop
  } state_e;
  
  state_e      curr_state;
  state_e      next_state;

  integer      cycle_counter;
  logic [15:0] baud_counter;
  logic        change_state;
  
  integer      fifo_pointer;
  logic [ 7:0] fifo_data [0:p_fifo_depth-1];
  
  //assign o_tx_full = (fifo_pointer >= p_fifo_depth);
  assign o_tx_full = curr_state != st_idle;
  
  // update fifo
  always_ff @(posedge i_clk) begin
    if (i_rst) begin
      //fifo_data    <= '{default:8'd0};
      fifo_pointer <= 0;
    end else begin
      if (i_wr_en) begin
        if (fifo_pointer < p_fifo_depth) begin
          fifo_data[fifo_pointer] <= i_data_tx;
          fifo_pointer <= fifo_pointer + 1;
        end
      end
      if (curr_state == st_stop && baud_counter == i_baudrate) begin
        fifo_data[0] <= fifo_data[1];
        fifo_data[1] <= fifo_data[2];
        fifo_data[2] <= fifo_data[3];
        fifo_data[3] <= 8'h0;
        fifo_pointer <= fifo_pointer - 1;
      end
    end
  end
  
  //! update the current state every clock cycle
  always_ff @(posedge i_clk) begin
    if (i_rst) begin
      curr_state <= st_idle;
    end else begin
      if (~i_en) begin
        curr_state <= st_idle;
      end else begin
        curr_state <= next_state;
      end
    end
  end
  
  // next_state state
  always_comb begin
    case (curr_state)
      st_idle   : next_state = i_wr_en || |fifo_pointer ? st_start : st_idle;
      st_start  : next_state = (change_state) ? st_data : st_start;
      st_data   : next_state = (change_state) ? st_stop : st_data;
      //st_parity : next_state = (change_state) ? st_stop : st_parity;
      st_stop   : next_state = (change_state) ? st_idle : st_stop;
      default: next_state = st_idle;
    endcase
  end
  
  //TODO: rewrite this mess
  always_ff @(posedge i_clk) begin
    if (i_rst) begin
      baud_counter  <= 0;
      cycle_counter <= 0;
      change_state  <= 0;
    end else begin
      if (~i_en) begin
        baud_counter  <= 0;
        cycle_counter <= 0;
        change_state  <= 0;
      end else begin
        if (curr_state == st_data) begin
          if (baud_counter == i_baudrate) begin
            baud_counter <= 0;
            if (cycle_counter == 7) begin
              change_state <= 1;
              cycle_counter <= cycle_counter;
            end else begin
              cycle_counter <= cycle_counter + 1;
              change_state <= 0;
            end
          end else begin
            baud_counter <= baud_counter + 1;
            cycle_counter <= cycle_counter;
            change_state <= 0;
          end
        end else if (curr_state == st_idle) begin
          cycle_counter <= 0;
          baud_counter <= 0;
          change_state <= 0;
        end else begin
          cycle_counter <= 0;
          if (baud_counter == i_baudrate) begin
            baud_counter <= 0;
            change_state <= 1;
          end else begin
            baud_counter <= baud_counter + 1;
            change_state <= 0;
          end
        end
      end
    end
  end
  
  //! decode output signal
  always_ff @(posedge i_clk) begin
    case (curr_state)
      st_start  : o_uart_tx <= 0;
      st_data   : o_uart_tx <= i_data_tx[cycle_counter];
      st_parity : o_uart_tx <= ~^i_data_tx;
      st_stop   : o_uart_tx <= 1;
      default   : o_uart_tx <= 1;
    endcase
  end
  
endmodule

`endif //__PERIPHERAL_UART_TX__