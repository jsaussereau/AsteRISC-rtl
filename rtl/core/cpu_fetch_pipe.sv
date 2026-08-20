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

//! CPU instruction fetch stage (pipeline version)
//!
//! The fetch unit is a two-deep pipeline:
//!
//!   `fetch_addr_q` -> instruction memory (registered read) -> output
//!
//! so the instruction word presented on `o_instr` during cycle `c` is the one
//! addressed by `fetch_addr_q` during cycle `c-1`. `pc_q` shadows that address
//! so that `o_pc` always describes `o_instr`.
//!
//! Flow control is a single enable, `i_en`:
//!
//!  * `i_en = 0` freezes `fetch_addr_q`, `pc_q` **and** the memory read enable.
//!    The RAM holds its output when `i_rd_en` is low, so the output word simply
//!    repeats and no fetched word is ever dropped -- no skid register needed.
//!  * `i_en = 1` advances by four bytes, or to `i_redirect_pc` when
//!    `i_redirect` is set.
//!
//! `i_redirect` is only sampled while the unit advances; the control unit
//! guarantees a redirect is never requested during a freeze (see `cpu_hazard`).
//!
//! Everything else -- squashing wrong-path instructions, counting retired
//! instructions -- belongs to the control unit and to the write back stage, not
//! here.

`ifndef __CPU_FETCH_PIPE__
`define __CPU_FETCH_PIPE__

`ifdef VIVADO
 `include "packages/pck_control.sv"
 `include "packages/pck_isa.sv"
`else
 `include "core/packages/pck_control.sv"
 `include "core/packages/pck_isa.sv"
`endif

module cpu_fetch_pipe 
  import pck_control::*;
  import pck_isa::*;
#(
  parameter p_reset_vector = 32'hf0000000
)(
  input  wire          i_clk,             //! global clock
  input  wire          i_rst,             //! global reset
  input  wire          i_sleep,           //! active high sleep control

  // instruction memory port
  output logic [31: 0] ibus_addr,         //! instruction bus address
  output logic [ 3: 0] ibus_be,           //! instruction bus write byte enable
  output logic         ibus_wr_en,        //! instruction bus write enable
  output logic [31: 0] ibus_wr_data,      //! instruction bus write data
  output logic         ibus_rd_en,        //! instruction bus read enable
  input  logic [31: 0] ibus_rd_data,      //! instruction bus read data
  input  logic         ibus_busy,         //! instruction bus busy
  input  logic         ibus_ack,          //! instruction bus transfer acknowledge

  // flow control
  input  wire          i_en,              //! advance the fetch pipeline
  input  wire          i_redirect,        //! jump to `i_redirect_pc`
  input  wire  [31: 0] i_redirect_pc,     //! redirection target

  // fetched instruction
  output logic [31: 0] o_pc,              //! program counter of `o_instr`
  output logic [31: 0] o_pc_inc,          //! `o_pc` + 4
  output isa_instr_t   o_instr,           //! instruction word
  output logic         o_valid            //! `o_instr` holds a fetched word
);

  logic [31: 0] fetch_addr_q;             //! address presented to the memory
  logic [31: 0] pc_q;                     //! address of the word on the output
  logic         valid_q;                  //! a word has been fetched at least once

  //! the fetch pipeline only moves when it is enabled and the core is awake
  wire          advance = i_en & ~i_sleep;

  wire  [31: 0] next_addr = i_redirect ? i_redirect_pc : (fetch_addr_q + 32'd4);

  always_ff @(posedge i_clk) begin: fetch_pipeline
    if (i_rst) begin
      fetch_addr_q <= p_reset_vector;
      pc_q         <= p_reset_vector - 32'd4;
      valid_q      <= 1'b0;
    end else if (advance) begin
      fetch_addr_q <= next_addr;
      pc_q         <= fetch_addr_q;
      valid_q      <= 1'b1;
    end
  end

  // the instruction memory is mapped at `p_reset_vector`; the bus carries the
  // offset inside that region
  assign ibus_addr    = fetch_addr_q & ~p_reset_vector;
  assign ibus_be      = 4'b0000;
  assign ibus_wr_en   = 1'b0;
  assign ibus_wr_data = 32'd0;
  assign ibus_rd_en   = advance;

  assign o_pc         = pc_q;
  assign o_pc_inc     = pc_q + 32'd4;
  assign o_instr.code = ibus_rd_data;
  assign o_valid      = valid_q;

endmodule

`endif // __CPU_FETCH_PIPE__
