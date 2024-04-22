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

// if p_wb_buf and p_decode_buf are enabled, the decode stage will be used for wb

`ifndef __CPU_WRITE_BACK__
`define __CPU_WRITE_BACK__

`ifdef VIVADO
 `include "packages/pck_control.sv"
`else
 `include "core/packages/pck_control.sv"
`endif

module cpu_write_back
  import pck_control::*;
#(
  parameter p_wb_buf       = 0            //! add buffers to write back stage inputs
)(
  input  wire          i_clk,             //! global clock
  input  wire          i_rst,             //! global reset
  input  wire          i_en_wb,           //! register file write enable. 
  input  sel_wb_e      i_sel_wb,          //! write back source selector
  input  wire  [ 4: 0] i_rf_wr_addr,      //! regfile write address
  input  wire  [31: 0] i_alu_out,         //! ALU result
  input  wire  [31: 0] i_muldiv_out,      //! MUL/DIV result
  input  wire  [31: 0] i_copro_out0,
  input  wire  [31: 0] i_copro_out1,
  input  wire  [31: 0] i_copro_out2,
  input  wire  [31: 0] i_pc_inc,
  input  wire  [31: 0] i_dbus_rd_data,
  input  wire  [31: 0] i_csr_rd_data,
  output logic [31: 0] o_rf_wr_data,      //! regfile write data
  output logic [ 4: 0] o_rf_wr_addr,      //! regfile write address
  output logic         o_rf_wr_en         //! regfile write enable
);
  
  sel_wb_e      sel_wb;
  logic [31: 0] alu_out;
  logic [31: 0] muldiv_out;
  logic [31: 0] copro_out0;
  logic [31: 0] copro_out1;
  logic [31: 0] copro_out2;
  logic [31: 0] pc_inc;
  logic [31: 0] dbus_rd_data;
  logic [31: 0] csr_rd_data;

  //! input synchronization
  generate
    if (p_wb_buf) begin
      always_ff @(posedge i_clk) begin
        sel_wb       <= i_sel_wb;
        alu_out      <= i_alu_out;
        muldiv_out   <= i_muldiv_out;
        copro_out0   <= i_copro_out0;
        copro_out1   <= i_copro_out1;
        copro_out2   <= i_copro_out2;
        pc_inc       <= i_pc_inc;
        dbus_rd_data <= i_dbus_rd_data;
        csr_rd_data  <= i_csr_rd_data;
      end
    end else begin
      always_comb begin
        sel_wb       = i_sel_wb;
        alu_out      = i_alu_out;
        muldiv_out   = i_muldiv_out;
        copro_out0   = i_copro_out0;
        copro_out1   = i_copro_out1;
        copro_out2   = i_copro_out2;
        pc_inc       = i_pc_inc;
        dbus_rd_data = i_dbus_rd_data;
        csr_rd_data  = i_csr_rd_data;
      end
    end
  endgenerate

  //! WB: write back mux
  always_comb begin: write_back
    case (sel_wb)
      wb_alu    : o_rf_wr_data = alu_out;
      wb_muldiv : o_rf_wr_data = muldiv_out;
      wb_pc     : o_rf_wr_data = pc_inc;
      wb_dmem   : o_rf_wr_data = dbus_rd_data;
      wb_csr    : o_rf_wr_data = csr_rd_data;
      wb_copro0 : o_rf_wr_data = copro_out0;
      wb_copro1 : o_rf_wr_data = copro_out1;
      wb_copro2 : o_rf_wr_data = copro_out2;
      default   : o_rf_wr_data = 32'd0;
    endcase
  end

  generate
    if (p_wb_buf) begin
      always_ff @(posedge i_clk) begin
        o_rf_wr_addr <= i_rf_wr_addr;
        o_rf_wr_en   <= i_en_wb;
      end
    end else begin
      always_comb begin
        o_rf_wr_addr = i_rf_wr_addr;
        o_rf_wr_en   = i_en_wb;
      end
    end
  endgenerate
  
endmodule

`endif // __CPU_WRITE_BACK__
