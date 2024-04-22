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

//! Single port 32-bit wide RAM with optional .hex init file for simulation


`ifndef __SOC_BRIDGE__
`define __SOC_BRIDGE__

`ifdef VIVADO
  `include "packages/pck_memory_map.sv"
  `include "../core/packages/pck_mem_bus.sv"
`else
  `include "soc/packages/pck_memory_map.sv"
  `include "core/packages/pck_mem_bus.sv"
`endif

module soc_bridge 
  import pck_memory_map::*;
#(
  parameter p_num_masters = 3
)(
  input  wire         i_clk,                          //! global clock

  //mem_bus.mem_side    dbus,                           //! data bus
  input  logic [31:0] dbus_addr,       //! data bus address
  input  logic [ 3:0] dbus_be,         //! data bus write byte enable
  input  logic        dbus_wr_en,      //! data bus write enable
  input  logic [31:0] dbus_wr_data,    //! data bus write data
  input  logic        dbus_rd_en,      //! data bus read enable
  output logic [31:0] dbus_rd_data,    //! data bus read data
  output logic        dbus_busy,       //! data bus busy
  output logic        dbus_ack,        //! data bus transfer acknowledge

  input  logic [31:0] i_base     [p_num_masters-1:0], //! peripheral write address
  input  logic [31:0] i_mask     [p_num_masters-1:0], //! peripheral write byte enable

  output logic [31:0] o_addr     [p_num_masters-1:0], //! peripheral write address
  output logic [ 3:0] o_be       [p_num_masters-1:0], //! peripheral write byte enable
  output logic        o_wr_en    [p_num_masters-1:0], //! peripheral write enable
  output logic [31:0] o_wr_data  [p_num_masters-1:0], //! peripheral write data
  output logic        o_rd_en    [p_num_masters-1:0], //! peripheral read enable
  input  logic [31:0] i_rd_data  [p_num_masters-1:0], //! peripheral read data
  input  wire         i_busy     [p_num_masters-1:0], //! peripheral busy
  input  wire         i_ack      [p_num_masters-1:0]  //! peripheral transfer acknowledge
);

  //periph_name_t active_name; //! meant only to make understanding easier in simulation
  logic [p_num_masters-1:0] active;
  logic [p_num_masters-1:0] saved_active;
  logic [31:0] saved_dbus_addr;
  logic [31:0] saved_dbus_rd_en;
  //localparam NO_PERIPH = p_num_masters;

  always_ff @(posedge i_clk) begin
    saved_dbus_addr   <= dbus_addr;
    saved_dbus_rd_en  <= dbus_rd_en;
    saved_active      <= active;
  end

  //! determine which peripheral is adressed
  always_comb begin: peripheral_active
    active = 0;
    //active_name.value = NO_PERIPH;
    for (integer i = 0 ; i < p_num_masters ; i++) begin
      if (dbus_wr_en && (dbus_addr & i_mask[i]) == i_base[i] || dbus_rd_en && (dbus_addr & i_mask[i]) == i_base[i]) begin
        active[i] = 1;
        //active_name.value = i;
      end
    end
  end

  //! cpu writes to addressed peripheral
  always_comb begin: bus_write
    for (integer i = 0 ; i < p_num_masters ; i++) begin 
      o_addr   [i] = dbus_addr & ~i_mask[i];
      o_be     [i] = dbus_be;
      o_wr_en  [i] = dbus_wr_en & active[i];
      o_wr_data[i] = dbus_wr_data;
      o_rd_en  [i] = dbus_rd_en & active[i];
    end
  end

  //! cpu reads from addressed peripheral
  always_comb begin: bus_read
    dbus_rd_data = 32'd0;
    dbus_busy    = 1'b0;
    dbus_ack     = 1'b0;
    for (integer i = 0 ; i < p_num_masters ; i++) begin
      if (saved_active[i]) begin
        dbus_rd_data = i_rd_data[i];
        dbus_busy    = i_busy   [i];
        dbus_ack     = i_ack    [i];
        //break;
      end
    end
  end

endmodule

`endif // __SOC_BRIDGE__
