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

`ifndef __CPU_MEM__
`define __CPU_MEM__

`ifdef VIVADO
 `include "packages/pck_control.sv"
 `include "packages/pck_sext.sv"
`else
 `include "core/packages/pck_control.sv"
 `include "core/packages/pck_sext.sv"
`endif

module cpu_mem
  import pck_control::*;
  import pck_sext::*;
#( 
  parameter p_mem_buf      = 0            //! add buffers to mem stage inputs
)(
  input  logic         i_clk,             //! global clock: triggers with a rising edge

  // memory port
  //mem_bus.cpu_side     dbus,              //! data bus
  output logic [31: 0] dbus_addr,         //! data bus address
  output logic [ 3: 0] dbus_be,           //! data bus write byte enable
  output logic         dbus_wr_en,        //! data bus write enable
  output logic [31: 0] dbus_wr_data,      //! data bus write data
  output logic         dbus_rd_en,        //! data bus read enable
  input  logic [31: 0] dbus_rd_data,      //! data bus read data
  input  logic         dbus_busy,         //! data bus busy
  input  logic         dbus_ack,          //! data bus transfer acknowledge

  // control signals
  input  wire          i_swap_bytes,      //! swap bytes (big endian -> little endian)
  input  wire  [31: 0] i_adder_out,       //! adder output
  input  sel_be_e      i_sel_dmem_be,     //! dmem address mode (word, byte, halfbyte)
  input  wire          i_dmem_sext,       //! sign extension on non-word dmem data
  input  wire          i_en_dmem_wr,      //! write enable
  input  wire  [31: 0] i_rf_rd2_data,     //! write data from register file
  input  wire          i_en_dmem_rd,      //! read enable
  output logic [31: 0] o_dbus_rd_data,    //! read data with sign/zero extension 
  output logic         o_dbus_busy,       //! busy: can't read/write now
  output logic         o_dbus_ack         //! acknowledge
);

  logic [31: 0] addr;
  logic [31: 0] source_data;
  logic [ 1: 0] data_align;
  logic [ 1: 0] rd_data_align;
  logic [31: 0] rd_data;
  logic [31: 0] aligned_wr_data;
  logic [31: 0] aligned_rd_data;
  logic [31: 0] masked_rd_data;
  logic [ 3: 0] be;

  logic         saved_swap_bytes;
  sel_be_e      saved_sel_dmem_be;
  logic         saved_dmem_sext;

  assign addr = { i_adder_out[31:2], 2'd0 };
  assign data_align = i_adder_out[1:0];

  //! connect control signals
  generate
    if (p_mem_buf) begin
      always_ff @(posedge i_clk) begin: mem_bus_inputs
        dbus_addr      <= addr;
        dbus_be        <= be;
        // mux to 0 to avoid having a critical going through here if the memory is not used
        dbus_wr_data   <= (i_en_dmem_wr || i_en_dmem_rd) ? aligned_wr_data : 32'b0;
        dbus_wr_en     <= i_en_dmem_wr;
        dbus_rd_en     <= i_en_dmem_rd;
      end
    end else begin
      always_comb begin: mem_bus_inputs
        dbus_addr      = addr;
        dbus_be        = be;
        // mux to 0 to avoid having a critical going through here if the memory is not used
        dbus_wr_data   = (i_en_dmem_wr || i_en_dmem_rd) ? aligned_wr_data : 32'b0;
        dbus_wr_en     = i_en_dmem_wr;
        dbus_rd_en     = i_en_dmem_rd;
      end
    end
  endgenerate

  always_ff @(posedge i_clk) begin
    rd_data_align     <= data_align;
    saved_sel_dmem_be <= i_sel_dmem_be;
    saved_dmem_sext   <= i_dmem_sext;
    saved_swap_bytes  <= i_swap_bytes;
  end

  // bus ouputs
  always_comb begin: mem_bus_outputs
    rd_data        = dbus_rd_data;
    o_dbus_busy    = dbus_busy;
    o_dbus_ack     = dbus_ack;
  end


  //! byte write enable
  always_comb begin
    case (data_align)
      2'b00: begin
        case (i_sel_dmem_be)
          be_word: be = 4'b1111;
          be_half: be = 4'b0011;
          be_byte: be = 4'b0001;
          default: be = 4'b0000;
        endcase
      end
      2'b01: begin
        case (i_sel_dmem_be)
          be_word: be = 4'b1111;
          be_half: be = 4'b0011;
          be_byte: be = 4'b0010;
          default: be = 4'b0000;
        endcase
      end
      2'b10: begin
        case (i_sel_dmem_be)
          be_word: be = 4'b1111;
          be_half: be = 4'b1100;
          be_byte: be = 4'b0100;
          default: be = 4'b0000;
        endcase
      end
      2'b11: begin
        case (i_sel_dmem_be)
          be_word: be = 4'b1111;
          be_half: be = 4'b1100;
          be_byte: be = 4'b1000;
          default: be = 4'b0000;
        endcase
      end
    endcase
  end

  always_comb begin
    //case (rd_data_align)
      source_data = i_rf_rd2_data;
    //endcase
  end

  //! align write data
  always_comb begin
    case (data_align)
      2'b00 : aligned_wr_data =   source_data[31:0];
      2'b01 : aligned_wr_data = { source_data[23:0],  8'd0 };
      2'b10 : aligned_wr_data = { source_data[15:0], 16'd0 };
      2'b11 : aligned_wr_data = { source_data[ 7:0], 24'd0 };
    endcase
  end

  //! align read data
  always_comb begin
    case (rd_data_align)
      2'b00: aligned_rd_data =           rd_data[31: 0];
      2'b01: aligned_rd_data = {  8'd0 , rd_data[31: 8] };
      2'b10: aligned_rd_data = { 16'd0 , rd_data[31:16] };
      2'b11: aligned_rd_data = { 24'd0 , rd_data[31:24] };
    endcase
  end


  //! sign/zero extension
  always_comb begin: sext
    if (!saved_swap_bytes) begin
      if (saved_dmem_sext) begin // sign extension
        case (saved_sel_dmem_be)
          be_byte : masked_rd_data = { {24{aligned_rd_data[7]}}, aligned_rd_data[ 7:0] };
          be_half : masked_rd_data = { {16{aligned_rd_data[15]}}, aligned_rd_data[15:0] };
          default : masked_rd_data = aligned_rd_data;
        endcase
      end else begin         // zero padding
        case (saved_sel_dmem_be)
          be_byte : masked_rd_data = { 24'd0, aligned_rd_data[ 7:0] };
          be_half : masked_rd_data = { 16'd0, aligned_rd_data[15:0] };
          default : masked_rd_data = aligned_rd_data;
        endcase    
      end
    end else begin
      if (saved_dmem_sext) begin // sign extension
        case (saved_sel_dmem_be)
          be_byte : masked_rd_data = { {24{aligned_rd_data[7]}}, aligned_rd_data[7:0] };
          be_half : masked_rd_data = { {16{aligned_rd_data[7]}}, aligned_rd_data[7:0], aligned_rd_data[15:8] };
          default : masked_rd_data = { aligned_rd_data[7:0], aligned_rd_data[15:8], aligned_rd_data[23:16], aligned_rd_data[31:24]};
        endcase
      end else begin         // zero padding
        case (saved_sel_dmem_be)
          be_byte : masked_rd_data = { 24'd0, aligned_rd_data[7:0] };
          be_half : masked_rd_data = { 16'd0, aligned_rd_data[7:0], aligned_rd_data[15:8] };
          default : masked_rd_data = { aligned_rd_data[7:0], aligned_rd_data[15:8], aligned_rd_data[23:16], aligned_rd_data[31:24]};
        endcase    
      end
    end
  end

  assign o_dbus_rd_data = masked_rd_data;

`ifdef verilator

  function logic get_dbus_wr_en();
    // verilator public
    get_dbus_wr_en = dbus_wr_en;
  endfunction

  function [31:2] get_dbus_addr();
    // verilator public
    get_dbus_addr = dbus_addr;
  endfunction

  function [31:0] get_dbus_wr_data();
    // verilator public
    get_dbus_wr_data = dbus_wr_data;
  endfunction

  function [31:0] get_dbus_rd_data();
    // verilator public
    get_dbus_rd_data = masked_rd_data;
  endfunction
`endif

endmodule

`endif // __CPU_MEM__
