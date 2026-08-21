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

`ifndef __CPU_DYNAMIC_BRANCH_PREDICTOR__
`define __CPU_DYNAMIC_BRANCH_PREDICTOR__

`ifdef VIVADO
 `include "packages/pck_control.sv"
 `include "packages/pck_isa.sv"
`else
 `include "core/packages/pck_control.sv"
 `include "core/packages/pck_isa.sv"
`endif

//! Dynamic direction predictor: a table of saturating counters indexed by the
//! branch address, optionally hashed with a global history register (gshare).
//!
//! The whole point of the parameterisation is to span, with a single piece of
//! RTL, everything from a predictor that costs a handful of flip-flops to one
//! that is genuinely accurate:
//!
//!   p_bp_index_bits : log2 of the number of counters. 2 entries (1 bit) is
//!                     already a usable "last branch" predictor, 8 or 9 bits
//!                     is where a small core stops gaining.
//!   p_bp_ctr_bits   : 1 = last outcome, no hysteresis, one flip-flop per entry
//!                     2 = the classical bimodal counter: one wrong outcome no
//!                         longer flips the prediction, which is what makes a
//!                         loop exit cheap
//!                     3+ = more inertia, rarely worth the area
//!   p_bp_ghr_bits   : 0 = bimodal, the table is indexed by the pc alone
//!                     n = gshare, the n last outcomes are xored into the index
//!                         so that a branch correlated with the ones before it
//!                         gets a counter of its own per history pattern
//!
//! The table is *not* indexed by the update address: the index used for a
//! prediction is handed out on `o_index` and given back on `i_upd_index` when
//! the branch resolves. This keeps the gshare index exact -- the history has
//! moved on by then -- and costs the core only a few bits carried alongside
//! the in-flight instruction.
//!
//! The history itself is updated in resolution order only. There is no
//! speculative history and therefore nothing to repair on a misprediction: the
//! predictor sees a strictly architectural outcome stream, at the cost of the
//! history lagging by the branches still in flight.
module cpu_dynamic_branch_predictor
  import pck_control::*;
  import pck_isa::*;
#(
  parameter p_bp_index_bits = 5,          //! log2 of the number of counters
  parameter p_bp_ctr_bits   = 2,          //! width of a saturating counter
  parameter p_bp_ghr_bits   = 0           //! global history bits (0 = bimodal)
)(
  input  wire          i_clk,             //! global clock
  input  wire          i_rst,             //! global reset

  // lookup
  input  wire          i_cond_branch,     //! conditionnal branch (inconditionnal branches jal and jalr are excluded)
  input  wire          i_branch_instr,    //! branch instruction
  input  wire          i_jalr_instr,      //! jalr instruction
  input  wire  [31: 0] i_imm,             //! immediate value
  input  wire  [31: 0] i_pc,              //! program counter
  input  wire  [31: 0] i_pc_inc,          //! sequential program counter
  output wire  [31: 0] o_predicted_pc,    //! predicted pc value
  output wire          o_predict_taken,   //! the control transfer is predicted taken
  output wire  [p_bp_index_bits-1:0] o_index, //! table index used for this prediction

  // update, when a conditionnal branch resolves
  input  wire          i_upd_valid,       //! a conditionnal branch resolved this cycle
  input  wire          i_upd_taken,       //! ...and this is its actual outcome
  input  wire  [p_bp_index_bits-1:0] i_upd_index //! the index it was predicted with
);

  localparam int IDX_W  = p_bp_index_bits;
  localparam int CTR_W  = p_bp_ctr_bits;
  localparam int GHR_W  = (p_bp_ghr_bits > 0) ? p_bp_ghr_bits : 1;
  localparam int ENTRIES = 1 << IDX_W;
  //! history bits that actually reach the index (a history longer than the
  //! table is folded away rather than silently ignored)
  localparam int GHR_USE = (GHR_W > IDX_W) ? IDX_W : GHR_W;

  //! weakly taken: the reset state of every counter. A never-seen branch is
  //! then predicted the way the static scheme would predict a loop, and a
  //! single outcome is enough to move away from it.
  localparam logic [CTR_W-1:0] CTR_INIT = {1'b1, {(CTR_W-1){1'b0}}};
  localparam logic [CTR_W-1:0] CTR_MAX  = {CTR_W{1'b1}};
  localparam logic [CTR_W-1:0] CTR_MIN  = {CTR_W{1'b0}};

  logic [CTR_W-1:0] bht [ENTRIES];
  logic [GHR_W-1:0] ghr;

  logic [GHR_W:0]   ghr_shift;
  logic [IDX_W-1:0] index;
  logic [IDX_W-1:0] pc_index;
  logic [IDX_W-1:0] ghr_index;
  logic             taken_hint;
  logic             predict_taken;
  logic [31: 0]     predicted_pc;

  //! bit 0 of the pc is always zero and bit 1 only ever varies with `p_ext_rvc`,
  //! so the index starts at bit 1: it keeps two adjacent 32-bit branches in
  //! distinct entries without wasting an index bit on compressed cores either.
  always_comb begin: table_index
    pc_index  = i_pc[IDX_W:1];
    if (p_bp_ghr_bits > 0) begin
      //! gshare: the history is xored into the *low* index bits, where the pc
      //! varies the most, so that the two sources actually mix
      ghr_index                 = '0;
      ghr_index[GHR_USE-1:0]    = ghr[GHR_USE-1:0];
      index                     = pc_index ^ ghr_index;
    end else begin
      ghr_index = '0;
      index     = pc_index;
    end
  end

  //! the history with the newest outcome shifted in, oldest one still on top
  assign ghr_shift = {ghr, i_upd_taken};

  //! the counter msb is the prediction
  assign taken_hint = bht[index][CTR_W-1];

  //! the direction is only ever dynamic for conditionnal branches: `jal` has a
  //! known target and is always taken, `jalr` needs a register value and is
  //! left to the pipeline as in the static scheme.
  always_comb begin: predict
    predict_taken = i_branch_instr & ~i_jalr_instr & (~i_cond_branch | taken_hint);
  end

  always_comb begin: program_counter
    if (predict_taken) begin
      predicted_pc = i_pc + $signed(i_imm);
    end else begin
      predicted_pc = i_pc_inc;
    end
  end

  //! counter update: saturating, one entry per resolved branch
  always_ff @(posedge i_clk) begin: update_bht
    if (i_rst) begin
      for (int k = 0; k < ENTRIES; k++) begin
        bht[k] <= CTR_INIT;
      end
    end else if (i_upd_valid) begin
      if (i_upd_taken) begin
        if (bht[i_upd_index] != CTR_MAX) begin
          bht[i_upd_index] <= bht[i_upd_index] + 1'b1;
        end
      end else begin
        if (bht[i_upd_index] != CTR_MIN) begin
          bht[i_upd_index] <= bht[i_upd_index] - 1'b1;
        end
      end
    end
  end

  //! global history: resolution order, no speculation, hence no recovery
  always_ff @(posedge i_clk) begin: update_ghr
    if (i_rst) begin
      ghr <= '0;
    end else if ((p_bp_ghr_bits > 0) && i_upd_valid) begin
      //! a plain shift left. It is written through a one bit wider value and an
      //! explicit truncation rather than as `{ghr[GHR_W-2:0], ...}` so that a
      //! one bit history stays a legal part select.
      ghr <= ghr_shift[GHR_W-1:0];
    end
  end

  assign o_predicted_pc  = predicted_pc;
  assign o_predict_taken = predict_taken;
  assign o_index         = index;

endmodule

`endif // __CPU_DYNAMIC_BRANCH_PREDICTOR__
