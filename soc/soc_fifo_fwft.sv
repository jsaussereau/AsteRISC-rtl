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

`ifndef __SOC_FIFO_FWFT__
`define __SOC_FIFO_FWFT__

 // if both i_rd_en and i_rd_en_x4 are at 1, i_rd_en_x4 is ignored 

module soc_fifo_fwft #(
  parameter p_data_width = 8,
  parameter p_fifo_depth = 32,
  parameter p_rd_x4_interface = 0  // 4 words read 
)(
  input  wire                       i_clk,              //! global clock
  input  wire                       i_rst,              //! global reset
  input  wire                       i_wr_en,            //! fifo write enable
  input  wire  [p_data_width-1:0]   i_wr_data,          //! fifo write data
  input  wire                       i_rd_en,            //! fifo read enable
  output wire  [p_data_width-1:0]   o_rd_data,          //! fifo read data
  output wire                       o_empty,            //! fifo is empty
  input  wire                       i_rd_en_x4,         //! fifo read enable (4 words)
  output wire  [p_data_width*4-1:0] o_rd_data_x4,       //! fifo read data (4 words)
  output wire                       o_empty_x4,         //! fifo is empty (4 words)
  output wire                       o_full              //! fifo is full
);
  
  // status signals
  logic looped;
  logic empty;
  logic empty_x4;
  logic full;
  logic [p_data_width-1:0]   rd_data;          //! fifo read data
  logic [p_data_width*4-1:0] rd_data_x4;       //! fifo read data

  // fifo memory
  logic [0:p_fifo_depth-1][p_data_width-1:0] memory;

  // wr_pointer an rd_pointer pointers
  logic [$clog2(p_fifo_depth):0] rd_pointer;
  logic [$clog2(p_fifo_depth):0] wr_pointer;
  wire  [$clog2(p_fifo_depth):0] next_address;
  wire  [$clog2(p_fifo_depth):0] next_address_n1;
  wire  [$clog2(p_fifo_depth):0] next_address_n2;
  wire  [$clog2(p_fifo_depth):0] next_address_n3;

  wire  [$clog2(p_fifo_depth):0] b0_address;
  wire  [$clog2(p_fifo_depth):0] b1_address;
  wire  [$clog2(p_fifo_depth):0] b2_address;
  wire  [$clog2(p_fifo_depth):0] b3_address;


  // get next read address
  assign next_address = (rd_pointer == p_fifo_depth - 1) ? 0 : rd_pointer + 1;

  // get next other 3 read address for x4 interface
  assign next_address_n1 = (rd_pointer + 2 >= p_fifo_depth) ? rd_pointer + 2 - p_fifo_depth : rd_pointer + 2;
  assign next_address_n2 = (rd_pointer + 3 >= p_fifo_depth) ? rd_pointer + 3 - p_fifo_depth : rd_pointer + 3;
  assign next_address_n3 = (rd_pointer + 4 >= p_fifo_depth) ? rd_pointer + 4 - p_fifo_depth : rd_pointer + 4;
  
  assign b0_address = (rd_pointer + 4 >= p_fifo_depth) ? rd_pointer + 4 - p_fifo_depth : rd_pointer + 4;
  assign b1_address = (rd_pointer + 5 >= p_fifo_depth) ? rd_pointer + 5 - p_fifo_depth : rd_pointer + 5;
  assign b2_address = (rd_pointer + 6 >= p_fifo_depth) ? rd_pointer + 6 - p_fifo_depth : rd_pointer + 6;
  assign b3_address = (rd_pointer + 7 >= p_fifo_depth) ? rd_pointer + 7 - p_fifo_depth : rd_pointer + 7;

  always_ff @(posedge i_clk) begin
    if (i_rst) begin
      wr_pointer <= 0;
      rd_pointer <= 0;
      empty <= 1'b1;
      empty_x4 <= 1'b1;
      looped <= 1'b0;
    end else begin
      // fifo read
      if (i_rd_en && (looped || wr_pointer != rd_pointer)) begin

        // update rd_pointer pointer
        if (rd_pointer == p_fifo_depth - 1) begin
          rd_pointer <= 0;
          looped <= 1'b0;
        end else begin
          rd_pointer <= rd_pointer + 1;
        end

        // read the next data
        rd_data <= memory[next_address];

        // check if it was the last data
        if (next_address == wr_pointer) begin
          empty <= 1'b1;
        end else begin
          empty <= 1'b0;
        end
      end else begin

        if (wr_pointer == rd_pointer && !looped) begin
          empty <= 1'b1;
        end else begin
          empty <= 1'b0;
        end

        rd_data <= memory[rd_pointer];
      end

      // fifo read x4
      if (p_rd_x4_interface && !i_rd_en) begin
        if (i_rd_en_x4) begin
          if (rd_pointer + 3 >= p_fifo_depth - 1) begin
            looped <= 1'b0;
          end
          rd_pointer <= b0_address;

          // read the next data
          rd_data_x4 <= { memory[b3_address], memory[b2_address], memory[b1_address], memory[b0_address] };

          // check if it was the last data
          if (b0_address == wr_pointer || b1_address == wr_pointer || b2_address == wr_pointer || b3_address == wr_pointer) begin
            empty_x4 <= 1'b1;
          end else begin
            empty_x4 <= 1'b0;
          end

        end else begin 
          if (wr_pointer == rd_pointer || wr_pointer == next_address || wr_pointer == next_address_n1 || wr_pointer == next_address_n2) begin
            empty_x4 <= 1'b1;
          end else begin
            empty_x4 <= 1'b0;
          end
  
          rd_data_x4 <= { memory[next_address_n2], memory[next_address_n1], memory[next_address], memory[rd_pointer] };
        end
      end
      
      // fifo write
      if (i_wr_en && (!looped || wr_pointer != rd_pointer)) begin
        //write data to memory
        memory[wr_pointer] <= i_wr_data;

        //increment wr_pointer pointer as needed
        if (wr_pointer == p_fifo_depth - 1) begin
          wr_pointer <= 0;
          looped <= 1'b1;
        end else begin
          wr_pointer <= wr_pointer + 1;
        end
      end
    end
  end  
  
  assign full = (wr_pointer == rd_pointer && looped) ? 1'b1 : 1'b0;;

  assign o_rd_data = rd_data;
  assign o_empty   = empty;
  assign o_full    = full;

  if (p_rd_x4_interface) begin
    assign o_rd_data_x4 = rd_data_x4;
    assign o_empty_x4   = empty_x4;
  end else begin
    assign o_rd_data_x4 = 32'b0;
    assign o_empty_x4   = 1'b0;
  end

endmodule

`endif // __SOC_FIFO_FWFT__
