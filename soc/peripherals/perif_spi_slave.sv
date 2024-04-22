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

//! SPI slave periphereal

`ifndef __PERIPHERAL_SPI_SLAVE__
`define __PERIPHERAL_SPI_SLAVE__

`ifdef VIVADO
  `include "../packages/pck_memory_map.sv"
  `include "../packages/pck_registers.sv"
`else
  `include "soc/packages/pck_memory_map.sv"
  `include "soc/packages/pck_registers.sv"
`endif

module perif_spi_slave
  import pck_memory_map::*;
#(
  parameter p_fifo_in_depth  = 256,       //! bytes
  parameter p_fifo_out_depth = 128        //! bytes
)(
  input  wire         i_clk,              //! global clock
  input  wire         i_rst,              //! global reset
  input  wire  [31:2] i_addr,             //! read/write address
  input  wire  [ 3:0] i_be,               //! write byte enable
  input  wire         i_wr_en,            //! write enable
  input  wire  [31:0] i_wr_data,          //! write data
  input  wire         i_rd_en,            //! read enable
  output logic [31:0] o_rd_data,          //! read data
  output logic        o_busy,             //! busy
  output logic        o_ack,              //! transfer acknowledge
  
  input  wire         i_sck,              //! SPI clock
  input  wire         i_csb,              //! SPI chip select
  input  wire         i_sdi,              //! SPI in (MOSI)
  output wire         o_sdo,              //! SPI out (MISO)
  output wire         o_sdo_en            //! SPI out enable
);

  spi_slave_registers regs ();

  logic [63:0] input_reg;
  logic [ 2:0] bit_counter;
  logic [ 8:0] byte_counter;
  logic [ 7:0] msg_size;
  logic        msg_size_stored;
  logic        rst_bit_counter;
  logic        aligned;
  logic        synchronized;
  logic        rd_en_old;

  logic        fifo_in_wr_en;
  logic [ 7:0] fifo_in_wr_data;
  logic        fifo_in_rd_en;
  logic [ 7:0] fifo_in_rd_data;
  logic        fifo_in_empty;
  logic        fifo_in_rd_en_x4;
  logic [31:0] fifo_in_rd_data_x4;
  logic        fifo_in_empty_x4;
  logic        fifo_in_full;

  logic        fifo_out_wr_en;
  logic [ 7:0] fifo_out_wr_data;
  logic        fifo_out_rd_en;
  logic [ 7:0] fifo_out_rd_data;
  logic        fifo_out_empty;
  logic        fifo_out_full;
    
  logic [ 2:0] p2s_bit_counter;
  logic        sdo;

  /******************
       SPI logic
  ******************/

  // Store received data into a register
  always_ff @(posedge i_sck) begin: store_input
    if (i_rst) begin
      input_reg <= 64'b0;
    end else begin
      if (!i_csb) begin
        input_reg <= { input_reg[62:0], i_sdi };
      end
    end
  end

  // Check for start pattern ands count bits
  always_comb begin: find_start_pattern
    if (i_rst) begin
      aligned = 1'b0;
      rst_bit_counter = 1'b0;
    end else begin
      if (!i_csb) begin
        if (regs.spi_config.use_msg_size && msg_size_stored && byte_counter > msg_size) begin
          aligned = 1'b0;
          rst_bit_counter = 1'b1;
        end else if (regs.spi_config.wait_for_msg_start && regs.spi_config.no_current_msg) begin
          if ((input_reg[31:0] & regs.spi_start_pattern_msk) == (regs.spi_start_pattern & regs.spi_start_pattern_msk)) begin
            aligned = 1'b1;
            rst_bit_counter = 1'b1;
          end else begin
            rst_bit_counter = 1'b0;
          end 
        end else begin
          aligned = 1'b0;
          rst_bit_counter = 1'b0;
        end
      end
    end
  end

  // Bit counter
  always_ff @(posedge i_sck) begin: count_bits
    if (i_rst) begin
      bit_counter <= 3'b0;
      byte_counter <= 7'b0;
      msg_size <= 7'b0;
      fifo_in_wr_data <= 8'b0;
      fifo_in_wr_en <= 1'b0;
      synchronized <= 1'b0;
      msg_size_stored <= 1'b0;
    end else begin
      if (!i_csb) begin
        if (rst_bit_counter) begin
          bit_counter <= 0;
          byte_counter <= regs.spi_config.msg_size_init_value;
          msg_size_stored <= 1'b0;
        end else begin
          if (bit_counter == 7) begin
            bit_counter <= 0;
            synchronized <= aligned;
            fifo_in_wr_en <= aligned;
            fifo_in_wr_data <= input_reg[7:0];
            if (byte_counter == regs.spi_config.msg_size_byte && aligned) begin
              msg_size <= input_reg[7:0];
              msg_size_stored <= 1'b1;
            end
            if (aligned) begin
              byte_counter <= byte_counter + 1;
            end
          end else begin
            bit_counter <= bit_counter + 1;
            // write start pattern into fifo
            if (bit_counter < 4 && regs.spi_config.write_start_pattern && aligned && !synchronized) begin
              if (bit_counter == 0) begin
                fifo_in_wr_en <= (regs.spi_start_pattern_msk[31:24] != 8'b0) ? 1'b1 : 1'b0;
                fifo_in_wr_data <= regs.spi_start_pattern[31:24];
              end else if (bit_counter == 1) begin
                fifo_in_wr_en <= (regs.spi_start_pattern_msk[23:16] != 8'b0) ? 1'b1 : 1'b0;
                fifo_in_wr_data <= regs.spi_start_pattern[23:16];
              end else if (bit_counter == 2) begin
                fifo_in_wr_en <= (regs.spi_start_pattern_msk[15:8] != 8'b0) ? 1'b1 : 1'b0;
                fifo_in_wr_data <= regs.spi_start_pattern[15:8];
              end else if (bit_counter == 3) begin
                fifo_in_wr_en <= (regs.spi_start_pattern_msk[7:0] != 8'b0) ? 1'b1 : 1'b0;
                fifo_in_wr_data <= regs.spi_start_pattern[7:0];
              end
            end else begin
              fifo_in_wr_en <= 1'b0;
            end
          end
        end
      end
    end
  end

    
  /******************
         FIFO
  ******************/

  soc_fifo_fwft #(
    .p_data_width       ( 8                   ),
    .p_fifo_depth       ( p_fifo_in_depth     ),
    .p_rd_x4_interface  ( 1                   )
  ) fifo_in ( 
    .i_clk              ( i_clk               ),
    .i_rst              ( i_rst               ),
    .i_wr_en            ( fifo_in_wr_en       ),
    .i_wr_data          ( fifo_in_wr_data     ),
    .i_rd_en            ( fifo_in_rd_en       ),
    .o_rd_data          ( fifo_in_rd_data     ),
    .o_empty            ( fifo_in_empty       ),
    .i_rd_en_x4         ( fifo_in_rd_en_x4    ),
    .o_rd_data_x4       ( fifo_in_rd_data_x4  ),
    .o_empty_x4         ( fifo_in_empty_x4    ),
    .o_full             ( fifo_in_full        )
  );

  soc_fifo_fwft #(
    .p_data_width       ( 8                   ),
    .p_fifo_depth       ( p_fifo_out_depth    ),
    .p_rd_x4_interface  ( 0                   )
  ) fifo_out (    
    .i_clk              ( i_clk               ),
    .i_rst              ( i_rst               ),
    .i_wr_en            ( fifo_out_wr_en      ),
    .i_wr_data          ( fifo_out_wr_data    ),
    .i_rd_en            ( fifo_out_rd_en      ),
    .o_rd_data          ( fifo_out_rd_data    ),
    .o_empty            ( fifo_out_empty      ),
    .i_rd_en_x4         ( 1'b0                ),
    .o_rd_data_x4       (                     ),
    .o_empty_x4         (                     ),
    .o_full             ( fifo_out_full       )
  );

  assign regs.spi_status_rx.fifo_full     = fifo_in_full;
  assign regs.spi_status_rx.fifo_full_x4  = 1'b1;
  assign regs.spi_status_rx.fifo_empty    = fifo_in_empty;
  assign regs.spi_status_rx.fifo_empty_x4 = fifo_in_empty_x4;

  assign regs.spi_status_tx.fifo_full     = fifo_out_full;
  assign regs.spi_status_tx.fifo_full_x4  = 1'b1;
  //assign regs.spi_status_tx.fifo_full_x4  = fifo_out_full_x4;
  assign regs.spi_status_tx.fifo_empty    = fifo_out_empty;
  assign regs.spi_status_tx.fifo_empty_x4 = 1'b1;

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
      regs.spi_config.send_countdown      <= 6'd24;
      regs.spi_config.compare_offset      <= 4'd4;
      regs.spi_config.send_on_compare     <= 1'b1;
      regs.spi_config.write_start_pattern <= 1'b1;
      regs.spi_config.msg_size_byte       <= 4'd4;
      regs.spi_config.msg_size_init_value <= 4'd4;
      regs.spi_config.use_msg_size        <= 1'b1;
      regs.spi_config.no_current_msg      <= 1'b1;
      regs.spi_config.wait_for_msg_start  <= 1'b1;
      regs.spi_start_pattern              <= 32'h00ffa5a5;
      regs.spi_start_pattern_msk          <= 32'h00ffffff;
      regs.spi_data_tx                    <= 32'h00000000;
      fifo_out_wr_en <= 1'b0;
    end else begin 
      wr_ack <= 1'b1;
      fifo_out_wr_en <= 1'b0;
      if (i_wr_en) begin
        case (i_addr)
          SPI_CONFIG_OFFSET: begin
            if (i_be[3]) regs.spi_config            [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.spi_config            [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.spi_config            [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.spi_config            [ 7: 0] <= i_wr_data[ 7: 0];
          end
          SPI_CMP_PATTERN_OFFSET: begin
            if (i_be[3]) regs.spi_cmp_pattern       [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.spi_cmp_pattern       [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.spi_cmp_pattern       [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.spi_cmp_pattern       [ 7: 0] <= i_wr_data[ 7: 0];
          end
          SPI_CMP_PATTERN_MSK_OFFSET: begin
            if (i_be[3]) regs.spi_cmp_pattern_msk   [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.spi_cmp_pattern_msk   [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.spi_cmp_pattern_msk   [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.spi_cmp_pattern_msk   [ 7: 0] <= i_wr_data[ 7: 0];
          end 
          SPI_START_PATTERN_OFFSET: begin
            if (i_be[3]) regs.spi_start_pattern     [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.spi_start_pattern     [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.spi_start_pattern     [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.spi_start_pattern     [ 7: 0] <= i_wr_data[ 7: 0];
          end
          SPI_START_PATTERN_MSK_OFFSET: begin
            if (i_be[3]) regs.spi_start_pattern_msk [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.spi_start_pattern_msk [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.spi_start_pattern_msk [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.spi_start_pattern_msk [ 7: 0] <= i_wr_data[ 7: 0];
          end
          SPI_DATA_TX_OFFSET: begin
            //if (i_be[3]) regs.spi_data_tx           [31:24] <= i_wr_data[31:24];
            //if (i_be[2]) regs.spi_data_tx           [23:16] <= i_wr_data[23:16];
            //if (i_be[1]) regs.spi_data_tx           [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) begin
              regs.spi_data_tx                      [ 7: 0] <= i_wr_data[ 7: 0];
              fifo_out_wr_data                              <= i_wr_data[ 7: 0];
              fifo_out_wr_en                                <= 1'b1;
            end
          end
          SPI_DATA_TX_x4_OFFSET: begin
            if (i_be[3]) regs.spi_data_tx_x4        [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.spi_data_tx_x4        [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.spi_data_tx_x4        [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.spi_data_tx_x4        [ 7: 0] <= i_wr_data[ 7: 0];
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
          SPI_CONFIG_OFFSET            : rd_data <= regs.spi_config;
          SPI_CMP_PATTERN_OFFSET       : rd_data <= regs.spi_cmp_pattern;
          SPI_CMP_PATTERN_MSK_OFFSET   : rd_data <= regs.spi_cmp_pattern_msk;
          SPI_START_PATTERN_OFFSET     : rd_data <= regs.spi_start_pattern;
          SPI_START_PATTERN_MSK_OFFSET : rd_data <= regs.spi_start_pattern_msk;          
          //SPI_STATUS_RX_OFFSET         : rd_data <= regs.spi_status_rx;
          //SPI_STATUS_TX_OFFSET         : rd_data <= regs.spi_status_tx;
          SPI_STATUS_RX_OFFSET         : rd_data <= { 7'b0, regs.spi_status_rx.fifo_empty_x4,  7'b0, regs.spi_status_rx.fifo_empty,  7'b0, regs.spi_status_rx.fifo_full_x4, 7'b0, regs.spi_status_rx.fifo_full }; // read-only
          SPI_STATUS_TX_OFFSET         : rd_data <= { 7'b0, regs.spi_status_tx.fifo_empty_x4,  7'b0, regs.spi_status_tx.fifo_empty,  7'b0, regs.spi_status_tx.fifo_full_x4, 7'b0, regs.spi_status_tx.fifo_full }; // read-only
          //SPI_STATUS_TX_OFFSET         : rd_data <= { 7'b0, 1'b0,             7'b0, fifo_out_empty, 7'b0, 1'b0, 7'b0, fifo_out_full }; // read-only
          //SPI_STATUS_RX_OFFSET         : rd_data <= { 7'b0, fifo_in_empty_x4, 7'b0, fifo_in_empty,  7'b0, 1'b0, 7'b0, fifo_in_full }; // read-only
          SPI_DATA_TX_OFFSET           : rd_data <= regs.spi_data_tx;
          //SPI_DATA_TX_x4_OFFSET        : rd_data <= regs.spi_data_tx_x4;
          SPI_DATA_RX_OFFSET           : rd_data <= { 24'b0, fifo_in_rd_data };
          SPI_DATA_RX_x4_OFFSET        : rd_data <= fifo_in_rd_data_x4;
          default: rd_ack <= 1'b0;
        endcase
      end
    end
  end

  always_ff @(posedge i_clk) begin
    rd_en_old <= i_rd_en;
  end 

  // output parallel to serial
  always_ff @(posedge i_sck) begin: p2s_count_bits
    if (i_rst) begin
      p2s_bit_counter <= 3'b0;
      fifo_out_rd_en <= 1'b0;
    end else begin
      if (!i_csb && !fifo_out_empty) begin
        if (bit_counter == 7) begin
          p2s_bit_counter <= 0;
          fifo_out_rd_en <= 1'b0;
        end else if (bit_counter == 1) begin
          p2s_bit_counter <= p2s_bit_counter + 1;
          fifo_out_rd_en <= 1'b1;
        end else begin
          p2s_bit_counter <= p2s_bit_counter + 1;
          fifo_out_rd_en <= 1'b0;
        end
      end
    end
  end

  // output ff
  always_ff @(posedge i_sck) begin: sdo_out_ff
    if (i_rst) begin
      sdo <= 1'b0;
    end else begin
      if (!i_csb && !fifo_out_empty) begin
        //sdo <= fifo_out_rd_data[p2s_bit_counter+:1];
        case (p2s_bit_counter)
          3'd0: sdo <= fifo_out_rd_data[0];
          3'd1: sdo <= fifo_out_rd_data[1];
          3'd2: sdo <= fifo_out_rd_data[2];
          3'd3: sdo <= fifo_out_rd_data[3];
          3'd4: sdo <= fifo_out_rd_data[4];
          3'd5: sdo <= fifo_out_rd_data[5];
          3'd6: sdo <= fifo_out_rd_data[6];
          3'd7: sdo <= fifo_out_rd_data[7];
          default: sdo <= 1'b0;
        endcase  
      end else begin
        sdo <= 1'b0;
      end
    end
  end

  assign o_sdo = sdo;

  assign fifo_in_rd_en = (i_rd_en && i_rd_en != rd_en_old && i_addr == SPI_DATA_RX_OFFSET) ? 1'b1 : 1'b0;
  assign fifo_in_rd_en_x4 = (i_rd_en && i_rd_en != rd_en_old && i_addr == SPI_DATA_RX_x4_OFFSET) ? 1'b1 : 1'b0;
  
  assign o_rd_data = rd_data;

endmodule

`endif // __PERIPHERAL_SPI_SLAVE__
