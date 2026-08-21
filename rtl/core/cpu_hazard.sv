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

//! CPU pipeline control unit (hazards, stalls, flushes, redirections)
//!
//! ## Slots and groups
//!
//! `cpu_core_pipe` is parameterised: `p_stage_<X>` is the **number of pipeline
//! barriers** of group X, not a boolean. Rather than reasoning about a variable
//! number of stages, this unit reasons about a fixed number of **slots** -- the
//! places where an instruction is observed by a functional unit:
//!
//! | slot | payload      | functional unit fed by the slot           |
//! |------|--------------|------------------------------------------|
//! | IF   | fetch output | instruction decompressor                  |
//! | IC   | `instr_IC`   | instruction decoder                       |
//! | ID   | `ctrl_ID`    | register file read port                   |
//! | RF   | `ctrl_RF`    | execute unit (ALU, branch resolution)     |
//! | EX   | `ctrl_EX`    | data memory access                        |
//! | MA   | `ctrl_MA`    | write back mux                            |
//! | WB   | `ctrl_WB`    | register file write port                  |
//!
//! Slots always exist, and a slot is the *output* of its group. What the
//! parameters change is how many barriers stand between two consecutive slots:
//! `p_stage_RF = 0` makes the RF slot a combinational alias of the ID slot,
//! `p_stage_EX = 3` puts three barriers between the RF and EX slots. Every slot
//! of a group shares the same `en` and `flush`, so a group behaves as a single
//! elastic element whatever its depth, and every decision below is expressed on
//! slots -- which is what makes the unit correct for any parameter combination
//! instead of for the handful that happened to be tuned.
//!
//! ## Three mechanisms:
//!
//! 1. **Freeze / bubble** (`o_en_*`, `o_flush_*` from a stall). A slot that
//!    cannot issue freezes itself and everything upstream of it, and a bubble is
//!    inserted into the barrier immediately downstream. `freeze` is built by
//!    propagating the per-slot stall conditions upstream, which makes the
//!    pipeline elastic by construction: no combination of stalls can drop or
//!    duplicate an instruction.
//!
//! 2. **Redirection** (`o_redirect`). Every control transfer -- `jal`, `jalr`
//!    and conditional branches alike -- is resolved at *one* point, the RF slot,
//!    where the execute unit produces `branch_taken` and the target. `jal` may
//!    additionally be resolved earlier, at the IC slot straight out of the
//!    decoder (`p_early_jal`), because its target needs no register value.
//!    A redirection is emitted at most once per instruction: the resolving slot
//!    is visited exactly once -- an instruction leaves a slot only when it is
//!    not frozen, which is exactly when the redirection fires -- and `jal` is
//!    skipped at the RF slot when it has already been handled at the IC slot.
//!
//! 3. **Shadow kill** (`o_kill_IF`). After a redirection the front-end keeps
//!    delivering wrong-path instructions until the new target comes out of the
//!    instruction memory. That delay is a static property of the fetch pipeline
//!    (`REDIRECT_SHADOW`) counted in fetch *advances*, not in cycles, so a stall
//!    landing inside the shadow extends it instead of cutting it short. It does
//!    not depend on `p_stage_IF`: extra fetch barriers buffer the wrong-path
//!    words but do not make the instruction memory produce more of them.
//!    Instructions younger than the branch that are already latched are killed
//!    by flushing every front-end barrier from the resolving slot upstream -- a
//!    set that needs no parameter-dependent wording, because a barrier that does
//!    not exist has nothing to flush and ignores the request.
//!
//! ## Branch resolution slot (`p_branch_stage`)
//!
//! Resolving a control transfer costs a path that starts at the branch
//! comparators and ends at the enable and flush of every front-end barrier and
//! at the instruction memory address. That path is very often the critical one,
//! and it is entirely a *placement* choice: nothing forces the verdict to be
//! acted upon in the slot that produced it.
//!
//! `p_branch_stage` moves the acting point one group downstream:
//!
//! | value | verdict acted upon at | cost                                |
//! |-------|-----------------------|-------------------------------------|
//! | 0     | the RF slot           | none (default)                      |
//! | 1     | the EX slot           | one extra bubble per redirection    |
//!
//! The verdict and its target are still *computed* at the RF slot -- they need
//! the register operands -- but they are carried down the EX barrier as
//! `redirect_req` / `redirect_pc` and only acted upon at the EX slot. What the
//! parameter removes from the RF cycle is therefore the whole fan-out of the
//! decision, which is the expensive half, while the comparators themselves keep
//! a full cycle to settle before their result is registered.
//!
//! This is the pipelined counterpart of `p_branch_buf` in the multi-cycle core,
//! and it composes with `p_branch_pred`: with prediction on, the extra bubble is
//! only paid on a mispredict.
//!
//! ## Operand forwarding
//!
//! Forwarding is not written per named slot: `cpu_core_pipe` presents a flat,
//! **youngest-first** list of `N_FWD` in-flight register writes -- one entry per
//! barrier downstream of the RF slot, whatever the group it belongs to -- and
//! this unit picks the youngest entry whose destination matches. An entry that
//! matches but is not `ready` (a load whose data has not come back yet) is the
//! one read-after-write that forwarding cannot cover, hence the one stall. That
//! single rule replaces the hand-written EX/MA/WB/pending-write cases and scales
//! to any group depth for free.
//!
//! `p_fwd_mask` makes each entry *optional*. A masked-out entry is still
//! detected as a producer, it simply cannot be forwarded from, so the consumer
//! stalls until the write has committed instead. That is the canonical CPI /
//! critical path trade: the bypass network is an `N_FWD + 1`-way mux sitting
//! directly in front of the ALU operands, so dropping its deepest inputs
//! shortens the RF path at the cost of stalls whose frequency depends entirely
//! on the code -- which is why it has to be measured rather than reasoned about.
//!
//! Masking is safe for any entry because the stall it substitutes lasts exactly
//! as long as the entry is listed, and an entry is listed until its write is
//! architecturally visible: the register file read address is held during a
//! stall (see `cpu_core_pipe`), so the operands are re-read every stalled cycle
//! and pick the committed value up by themselves. No entry can be masked into a
//! deadlock either, since freezing the RF slot never freezes anything
//! downstream of it -- the producers always drain.
//!
//! Side effects (register write, memory write, redirection, retirement) are
//! gated by the slot `valid` bits in `cpu_core_pipe`, so a killed instruction is
//! architecturally invisible even before its payload is replaced by a bubble.

`ifndef __CPU_HAZARD__
`define __CPU_HAZARD__

`ifdef VIVADO
 `include "packages/pck_control.sv"
`else
 `include "core/packages/pck_control.sv"
`endif


module cpu_hazard
  import pck_control::*;
#(
  parameter p_stage_IF     = 1,           //! number of fetch barriers (>= 1)
  parameter p_stage_IC     = 0,           //! number of decompression barriers
  parameter p_stage_ID     = 1,           //! number of decode barriers
  parameter p_stage_RF     = 0,           //! number of register file barriers (0 or 1)
  parameter p_stage_EX     = 1,           //! number of execute barriers
  parameter p_stage_MA     = 1,           //! number of memory access barriers (>= 1)
  parameter p_stage_WB     = 1,           //! number of write back barriers
  parameter p_early_jal    = 1,           //! resolve `jal` from the decoder instead of the RF slot
  //! branch prediction scheme (shared with the multi-cycle core):
  //!   0 = off, 1 = static (backward taken / forward not taken),
  //!   2 = dynamic (saturating counters, optionally gshare -- see
  //!       `cpu_dynamic_branch_predictor` and the `p_bp_*` parameters).
  parameter p_branch_pred  = 0,           //! branch prediction scheme
  parameter p_redirect_buf = 1,           //! register the redirection target before the fetch stage
  //! slot at which a resolved control transfer is acted upon:
  //!   0 = the RF slot (default), 1 = the EX slot (one more bubble, shorter path)
  parameter p_branch_stage = 0,           //! branch resolution slot
  parameter p_n_fwd        = 3,           //! number of in-flight register writes presented by the core
  //! one bit per forwarding entry, in the order the core presents them. A clear
  //! bit turns that entry into a stall instead of a bypass.
  parameter logic [31: 0] p_fwd_mask = 32'hffff_ffff
)(
  input  wire          i_clk,             //! global clock
  input  wire          i_rst,             //! global reset

  // slot occupancy
  input  wire          i_valid_IC,        //! the IC slot holds a real instruction
  input  wire          i_valid_RF,        //! the RF slot holds a real instruction
  input  wire          i_valid_EX,        //! the EX slot holds a real instruction (`p_branch_stage`)

  // branch prediction (`p_branch_pred`)
  input  wire          i_predict_IC,      //! the IC slot holds a branch predicted taken
  input  wire          i_predicted_RF,    //! the RF slot instruction was predicted taken

  // consumer: source registers read by the instruction in the RF slot
  input  wire  [ 4: 0] i_rd1_addr_RF,     //! rs1 address
  input  wire  [ 4: 0] i_rd2_addr_RF,     //! rs2 address
  input  wire          i_rd2_used_RF,     //! rs2 is actually read

  // producers: in-flight register writes, youngest first (one per barrier
  // downstream of the RF slot). Flattened so that the port list stays portable.
  input  wire  [p_n_fwd-1  : 0] i_fwd_valid, //! the entry holds a real instruction
  input  wire  [p_n_fwd-1  : 0] i_fwd_en_wb, //! the entry writes a register
  input  wire  [p_n_fwd*5-1: 0] i_fwd_addr,  //! destination register of each entry
  input  wire  [p_n_fwd-1  : 0] i_fwd_ready, //! the entry already holds its result

  // control transfer resolution
  input  sel_br_e      i_sel_br_IC,       //! branch class decoded from the IC slot
  input  sel_br_e      i_sel_br_RF,       //! branch class of the RF slot
  input  wire          i_branch_taken_RF, //! the RF slot takes its branch
  //! redirection request resolved at the RF slot and carried down the EX
  //! barrier, used instead of the RF verdict when `p_branch_stage` is set
  input  wire          i_redirect_req_EX, //! the EX slot instruction must redirect the front-end
  input  wire          i_exec_done_RF,    //! the execute unit has a result for the RF slot

  // pipeline flow control
  output logic         o_en_IF,           //! advance the fetch stage
  output logic         o_en_IC,           //! load the IC barriers
  output logic         o_en_ID,           //! load the ID barriers
  output logic         o_en_RF,           //! load the RF barriers
  output logic         o_en_EX,           //! load the EX barriers
  output logic         o_en_MA,           //! load the MA barriers
  output logic         o_en_WB,           //! load the WB barriers
  output logic         o_kill_IF,         //! the fetch output is wrong-path
  output logic         o_flush_IF,        //! load a bubble into the extra fetch barriers
  output logic         o_flush_IC,        //! load a bubble into the IC barriers
  output logic         o_flush_ID,        //! load a bubble into the ID barriers
  output logic         o_flush_RF,        //! load a bubble into the RF barriers
  output logic         o_flush_EX,        //! load a bubble into the EX barriers
  output logic         o_flush_MA,        //! load a bubble into the MA barriers
  output logic         o_flush_WB,        //! load a bubble into the WB barriers
  output logic         o_squash_IC,       //! every IC barrier is younger than a resolved branch
  output logic         o_squash_ID,       //! every ID barrier is younger than a resolved branch
  output logic         o_squash_EX,       //! every EX barrier is younger than a resolved branch (`p_branch_stage`)

  // redirection
  output logic         o_redirect,        //! redirect the fetch stage this cycle
  output logic         o_redirect_early,  //! the target comes from the decoder (early `jal`)

  // operand forwarding towards the RF slot (one-hot over the entry list)
  output logic [p_n_fwd-1: 0] o_fwd_rs1,  //! forward rs1 from this entry
  output logic [p_n_fwd-1: 0] o_fwd_rs2,  //! forward rs2 from this entry

  // debug
  output logic         o_stall_rs1,       //! rs1 caused a load-use stall
  output logic         o_stall_rs2        //! rs2 caused a load-use stall
);

  /*******************************************************
    Static pipeline topology

    Everything below is derived from the `p_stage_*` parameters; nothing in this
    unit hard-codes a particular pipeline shape.
  *******************************************************/

  //! Fetch *advances* the front-end still owes after a redirection before its
  //! output is back on the right path: one for the registered instruction
  //! memory read, one more if the target itself is registered.
  //!
  //! Counting advances rather than cycles is what makes this correct when a
  //! stall lands inside the shadow: a frozen fetch stage repeats its output, so
  //! a wrong-path word repeats with it and the shadow has to wait too.
  localparam int  REDIRECT_SHADOW = 1 + p_redirect_buf;
  localparam int  SHADOW_W        = (REDIRECT_SHADOW < 2) ? 1 : 2;

  //! act upon a resolved control transfer at the EX slot rather than the RF slot
  localparam logic RESOLVE_AT_EX  = (p_branch_stage != 0);

  //! forwarding entries the core is allowed to bypass from
  wire [p_n_fwd-1: 0] fwd_allowed = p_fwd_mask[p_n_fwd-1: 0];

  initial begin
    assert (p_n_fwd <= 32)
      else $error("p_fwd_mask holds one bit per forwarding entry, so p_n_fwd must be <= 32");
    assert ((p_branch_stage == 0) || (p_stage_EX >= 1))
      else $error("p_branch_stage = 1 needs at least one EX barrier, otherwise the EX slot is the RF slot");
  end

  /*******************************************************
    Read after write

    A producer is an in-flight entry that owns a not-yet-committed write to a
    register the RF slot reads. `x0` is never a real destination and an invalid
    entry never produces anything. The entries are ordered youngest first, so
    the correct value is always the one of *lowest* index -- an older write to
    the same register has already been superseded.
  *******************************************************/

  wire          read_rs1 = i_valid_RF & (i_rd1_addr_RF != 5'd0);
  wire          read_rs2 = i_valid_RF & (i_rd2_addr_RF != 5'd0) & i_rd2_used_RF;

  logic [p_n_fwd-1: 0] match_rs1;         //! entries whose destination is rs1
  logic [p_n_fwd-1: 0] match_rs2;         //! entries whose destination is rs2

  always_comb begin: raw_detection
    for (int unsigned i = 0; i < p_n_fwd; i++) begin
      automatic logic [4:0] a = i_fwd_addr[i*5 +: 5];
      automatic logic       w = i_fwd_valid[i] & i_fwd_en_wb[i] & (a != 5'd0);
      match_rs1[i] = w & read_rs1 & (a == i_rd1_addr_RF);
      match_rs2[i] = w & read_rs2 & (a == i_rd2_addr_RF);
    end
  end

  //! youngest match wins: keep the match of lowest index, drop every older one
  logic [p_n_fwd-1: 0] sel_rs1;
  logic [p_n_fwd-1: 0] sel_rs2;

  always_comb begin: raw_priority
    automatic logic taken1 = 1'b0;
    automatic logic taken2 = 1'b0;
    sel_rs1 = '0;
    sel_rs2 = '0;
    for (int unsigned i = 0; i < p_n_fwd; i++) begin
      sel_rs1[i] = match_rs1[i] & ~taken1;
      sel_rs2[i] = match_rs2[i] & ~taken2;
      taken1     = taken1 | match_rs1[i];
      taken2     = taken2 | match_rs2[i];
    end
  end

  //! the selected entry can be forwarded when it already holds its result. A
  //! load still in flight cannot: the data memory is read from the EX slot and
  //! its result only exists at the MA slot, which is the one read-after-write
  //! forwarding cannot cover, hence the one stall.
  //! an entry can be bypassed from when it already holds its result *and* the
  //! configuration keeps its bypass path. Anything else becomes a stall.
  wire [p_n_fwd-1: 0] fwd_usable = i_fwd_ready & fwd_allowed;

  assign o_fwd_rs1 = sel_rs1 &  fwd_usable;
  assign o_fwd_rs2 = sel_rs2 &  fwd_usable;

  wire          stall_rs1 = |(sel_rs1 & ~fwd_usable);
  wire          stall_rs2 = |(sel_rs2 & ~fwd_usable);

  assign o_stall_rs1 = stall_rs1;
  assign o_stall_rs2 = stall_rs2;

  /*******************************************************
    Freeze and bubble

    `freeze_<slot>` is asserted when the instruction in that slot cannot move on
    this cycle. A slot that cannot move also blocks every slot upstream of it,
    which is the whole of the propagation below. Wherever a frozen slot feeds a
    slot that does move, a bubble is inserted so that the downstream instruction
    is not executed twice.
  *******************************************************/

  //! the RF slot is the only slot with a local stall condition today: a
  //! load-use hazard, or a multi-cycle execute unit that has not finished
  wire          stall_RF  = stall_rs1 | stall_rs2 | (i_valid_RF & ~i_exec_done_RF);

  wire          freeze_WB = 1'b0;
  wire          freeze_MA = freeze_WB;
  wire          freeze_EX = freeze_MA;
  wire          freeze_RF = freeze_EX | stall_RF;
  wire          freeze_ID = freeze_RF;
  wire          freeze_IC = freeze_ID;
  wire          freeze_IF = freeze_IC;

  /*******************************************************
    Redirection
  *******************************************************/

  logic [SHADOW_W-1: 0] shadow_q;         //! remaining wrong-path fetch outputs

  wire          in_shadow = |shadow_q;

  //! `jal` needs no register value: its target is `pc + imm`, both available
  //! straight out of the decoder. Resolving it there removes one bubble per
  //! taken jump for each front-end barrier.
  //! a conditional branch predicted taken redirects the front-end from the very
  //! same slot: the predicted target is `pc + imm`, exactly what `jal` uses
  wire          predict_IC  = (p_branch_pred != 0) & i_predict_IC;

  wire          redirect_IC = i_valid_IC
                            & ~freeze_IC
                            & ( ((p_early_jal != 0) & (i_sel_br_IC == br_jal))
                              | predict_IC );

  //! every other control transfer resolves where the execute unit produces its
  //! verdict. `jal` is skipped here when it was already handled above.
  //! with prediction enabled the RF slot no longer redirects on *taken*, but on
  //! *disagreement*: a branch predicted taken and confirmed taken has already
  //! been redirected at the IC slot and costs nothing here, while a branch
  //! predicted taken that turns out not taken has to be undone -- the core
  //! sends it back to `pc_inc`.
  wire          mispredict_RF = (p_branch_pred != 0) ? (i_branch_taken_RF ^ i_predicted_RF)
                                                     :  i_branch_taken_RF;

  //! the verdict as produced by the execute unit, in the slot that produced it
  wire          redirect_RF = i_valid_RF
                            & ~freeze_RF
                            & mispredict_RF
                            & ~((p_early_jal != 0) & (i_sel_br_RF == br_jal));

  //! the same verdict, registered in the EX barrier and acted upon one group
  //! later. `i_redirect_req_EX` already carries the `p_early_jal` and
  //! `p_branch_pred` qualifications, which are resolved where they are cheap.
  wire          redirect_EX = i_valid_EX
                            & ~freeze_EX
                            & i_redirect_req_EX;

  //! whichever of the two the configuration selects. Only one is ever built:
  //! the other side of the mux is a constant at elaboration.
  wire          redirect_late = RESOLVE_AT_EX ? redirect_EX : redirect_RF;

  //! an older instruction always wins: the resolving slot is downstream of the
  //! IC slot, so a late redirection overrides an early one
  assign o_redirect       = redirect_late | redirect_IC;
  assign o_redirect_early = redirect_IC & ~redirect_late;

  always_ff @(posedge i_clk) begin: redirect_shadow
    if (i_rst) begin
      // no shadow out of reset: the fetch stage reports its own output as
      // invalid until it holds a real instruction word
      shadow_q <= '0;
    end else if (o_redirect) begin
      shadow_q <= REDIRECT_SHADOW[SHADOW_W-1:0];
    end else if (in_shadow & ~freeze_IF) begin
      shadow_q <= shadow_q - 1'b1;
    end
  end

  /*******************************************************
    Outputs
  *******************************************************/

  assign o_en_IF = ~freeze_IF;
  assign o_en_IC = ~freeze_IC;
  assign o_en_ID = ~freeze_ID;
  assign o_en_RF = ~freeze_RF;
  assign o_en_EX = ~freeze_EX;
  assign o_en_MA = ~freeze_MA;
  assign o_en_WB = ~freeze_WB;

  //! the fetch output is wrong-path until the redirection target comes out of
  //! the instruction memory
  assign o_kill_IF = in_shadow;

  //! A barrier receives a bubble when the slot feeding it is frozen while it
  //! moves on -- otherwise the instruction it holds would be executed twice --
  //! or when the payload it is about to latch is younger than a control
  //! transfer that just resolved. The latter is every front-end barrier from
  //! the resolving slot upstream: barriers that do not exist simply ignore the
  //! request, which is exactly right since nothing is latched there.
  //!
  //! The two reasons do *not* reach the same barriers when a group is deeper
  //! than one, which is the whole difficulty of a deep group:
  //!
  //!  - a bubble is inserted at the *head* of the group only. The instructions
  //!    already inside it are older than the stall and must keep shifting;
  //!    replacing them by bubbles would simply lose them.
  //!  - a squash kills *every* barrier of the group. They are all younger than
  //!    the instruction that resolved the control transfer -- which has already
  //!    been captured by the group downstream -- so they all have to die.
  //!
  //! `o_flush_<X>` is the head request (both reasons), `o_squash_<X>` the one
  //! the deeper barriers obey. With a single barrier the two coincide, which is
  //! why this distinction was invisible until now.
  //! Everything strictly younger than the resolving instruction dies. Moving
  //! the resolution to the EX slot simply adds one group to that set: the RF
  //! group, whose barriers hold instructions fetched after the branch, and the
  //! EX group itself -- squashing an EX barrier kills what it is *about* to
  //! latch, and the resolving instruction has already left for the MA barrier
  //! on the same edge, so it survives and still retires.
  assign o_squash_IC = redirect_late | redirect_IC;
  assign o_squash_ID = redirect_late;
  assign o_squash_EX = redirect_late & RESOLVE_AT_EX;

  assign o_flush_IF = redirect_late | redirect_IC;
  assign o_flush_IC = (freeze_IF & ~freeze_IC) | o_squash_IC;
  assign o_flush_ID = (freeze_IC & ~freeze_ID) | o_squash_ID;
  assign o_flush_RF = (freeze_ID & ~freeze_RF) | redirect_late;
  //! When a group is empty, its slot is a combinational alias of the slot
  //! upstream: the flush request it receives goes nowhere, and the freeze it
  //! would have reported never happens. The next barrier downstream must
  //! therefore look past it, at the freeze of the closest slot that really is
  //! barriered, or it would latch the same payload every stalled cycle.
  wire          freeze_ex_eff = (p_stage_EX != 0) ? freeze_EX : freeze_RF;

  assign o_flush_EX = (freeze_RF     & ~freeze_EX) | o_squash_EX;
  assign o_flush_MA = (freeze_ex_eff & ~freeze_MA);
  assign o_flush_WB = (freeze_MA     & ~freeze_WB);

endmodule

`endif // __CPU_HAZARD__
