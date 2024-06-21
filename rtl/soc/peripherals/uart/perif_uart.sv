/*
 * A silicon-proven RISC-V based SoC
 *
 * Copyright(C) 2022 by Jonathan Saussereau. All rights reserved.
 * 
 * All source codes and documentation contain proprietary confidential
 * information and are distributed under license. It may be used, copied
 * and/or disclosed only pursuant to the terms of a valid license agreement
 * with Jonathan Saussereau. This copyright must be retained at all times.
 *
 * uart.sv
 *
 */


`ifndef __PERIPHERAL_UART__
`define __PERIPHERAL_UART__

`ifdef VIVADO
  `include "../../packages/pck_memory_map.sv"
  `include "../../packages/pck_registers.sv"
`else
  `include "soc/packages/pck_memory_map.sv"
  `include "soc/packages/pck_registers.sv"
`endif

module perif_uart
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
  output logic        o_ack,             //! transfer acknowledge

  input  logic        i_uart_rx,
  output logic        o_uart_tx
);

  logic uart_rd_en;
  logic uart_wr_en;

  uart_registers regs ();

  (* keep_hierarchy = "yes" *)
  perif_uart_tx uart_tx ( 
    .i_clk          ( i_clk                         ), 
    .i_rst          ( i_rst                         ), 
    .i_wr_en        ( uart_wr_en                    ),
    .i_en           ( regs.uart_config.tx_en        ),
    .i_baudrate     ( regs.uart_config.baudrate     ),
    //.i_baudrate     ( 16'd10                        ),
    //.i_nb_stop_bits ( regs.uart_config.nb_stop_bits ),
    //.i_parity       ( regs.uart_config.parity       ),
    .i_data_tx      ( regs.uart_data_tx.value       ),
    .o_tx_full      ( regs.uart_status.tx_full      ),
    .o_uart_tx      ( o_uart_tx                     )
  );

  assign uart_wr_en = i_wr_en && i_addr == UART_DATA_TX_OFFSET && i_be[0];

  (* keep_hierarchy = "yes" *)
  perif_uart_rx uart_rx (
    .i_clk          ( i_clk                         ),
    .i_rst          ( i_rst                         ), 
    .i_rd_en        ( uart_rd_en                    ),
    .i_en           ( regs.uart_config.rx_en        ),
    .i_baudrate     ( regs.uart_config.baudrate     ),
    //.i_baudrate     ( 16'd10                        ),
    .o_rx_data      ( regs.uart_data_rx.value       ),
    .o_empty        ( regs.uart_status.rx_empty     ),
    .i_rx_bit       ( i_uart_rx                     )
  );

  assign uart_rd_en = i_rd_en && i_addr == UART_DATA_RX_OFFSET;


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
      regs.uart_config.baudrate     <= 0;
      regs.uart_config.tx_en        <= 1'b0;
      regs.uart_config.rx_en        <= 1'b0;
      regs.uart_config.parity       <= 1'b0;
      regs.uart_config.nb_stop_bits <= 1'b0;
      regs.uart_data_tx             <= 0;
    end else begin 
      wr_ack <= 1'b1;
      if (i_wr_en) begin
        case (i_addr)
          UART_DATA_TX_OFFSET: begin
            if (i_be[3]) regs.uart_data_tx [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.uart_data_tx [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.uart_data_tx [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.uart_data_tx [ 7: 0] <= i_wr_data[ 7: 0];
          end
          UART_CONFIG_OFFSET: begin
            if (i_be[3]) regs.uart_config  [31:24] <= i_wr_data[31:24];
            if (i_be[2]) regs.uart_config  [23:16] <= i_wr_data[23:16];
            if (i_be[1]) regs.uart_config  [15: 8] <= i_wr_data[15: 8];
            if (i_be[0]) regs.uart_config  [ 7: 0] <= i_wr_data[ 7: 0];
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
      rd_ack <= 1'b1;
      if (i_rd_en) begin
        rd_data <= 32'h0;
        case (i_addr)
          UART_DATA_TX_OFFSET : rd_data[ 7:0] <= regs.uart_data_tx.value;
          UART_DATA_RX_OFFSET : rd_data[ 7:0] <= regs.uart_data_rx.value;
          UART_CONFIG_OFFSET  : rd_data[19:0] <= regs.uart_config[19:0];
          UART_STATUS_OFFSET  : rd_data[ 1:0] <= regs.uart_status;
          default: rd_ack  <= 1'b0;
        endcase
      end
    end
  end
  assign o_rd_data = rd_data;

endmodule

`endif // __PERIPHERAL_UART__
