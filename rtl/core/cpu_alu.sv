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

//! Arithmetic Logic Unit: arithmetic and bitwise operations on integers.
//! Immediates are muxed into rs2 (op_b)
//!
//! ## Micro-architecture parameters
//!
//! The `p_stage_*` / `p_*_buf` parameters of the cores decide *where* the
//! datapath is cut by a register barrier. They do not change what stands
//! between two barriers, so at a given cut the critical path is still set by
//! the combinational structure of this unit. The two parameters below expose
//! that structure:
//!
//!  - `p_alu_share_adder` trades a mux for two 32-bit operators. It moves area
//!    only: the number of cycles per instruction is unchanged.
//!  - `p_alu_shift_bits` trades cycles for logic depth, and is the one knob
//!    here that moves *both* axes -- which is exactly the CPI / critical path
//!    trade-off the exploration is about.
//!
//! Both are worth sweeping because their pay-off is target dependent: a barrel
//! shifter is nearly free on a LUT6 fabric with wide muxes and expensive on a
//! standard cell library, while a shared adder saves a carry chain that a small
//! FPGA charges dearly and a large one does not.
//!
//! ### Shared adder (`p_alu_share_adder`)
//!
//! `sub`, `slt`, `sltu` and the three branch comparators all need `a - b`. By
//! default each gets its own operator. When set, they all read a single
//! adder run in two's complement mode (`a + ~b + 1`), and the comparison
//! results are decoded from its sum and carry out. This is safe because the
//! comparator outputs and the operation output are never both used by the same
//! instruction: a conditional branch ignores `o_out`, and every instruction
//! that uses `o_out` ignores `o_ops_*`.
//!
//! ### Sequential shifter (`p_alu_shift_bits`)
//!
//! A 32-bit barrel shifter is five cascaded 2:1 mux layers and is very often
//! *the* critical path of a small RV32 core. `p_alu_shift_bits` = K replaces it
//! by a shifter that moves K bits per cycle:
//!
//!  - the first cycle applies the residual `shift_amount % K` through a small
//!    barrel of `log2(K)` layers,
//!  - each following cycle applies a *constant* shift of K, which is pure
//!    wiring and costs one 2:1 mux layer.
//!
//! Logic depth drops from 5 layers to `log2(K) + 1`, at up to `31/K` extra
//! cycles on shift instructions only. K = 32 keeps the plain barrel shifter and
//! is the default, so the parameter is inert unless the exploration sets it.
//!
//! Multi-cycle shifts report through `o_done`, the same handshake the multiply
//! and divide unit already uses (`o_exec_done` in `cpu_exec`): the multi-cycle
//! core waits in `st_execute` and the pipelined core freezes its RF slot.

`ifndef __CPU_ALU__
`define __CPU_ALU__

`ifdef VIVADO
 `include "packages/pck_control.sv"
`else
 `include "core/packages/pck_control.sv"
`endif
 
module cpu_alu 
  import pck_control::*;
#(
  parameter p_ext_rvm         = 1,      //! use RV32M extension (multiplication and division)
  parameter p_alu_share_adder = 0,      //! sub/slt/sltu and the branch comparators reuse the main adder
  parameter p_alu_shift_bits  = 32      //! bits shifted per cycle: 32 = barrel shifter, 1/2/4/8/16 = sequential
)(
  input  wire          i_clk,           //! global clock
  input  wire          i_rst,           //! global reset
  input  wire          i_start,         //! the operands presented this cycle belong to a new instruction
  input  wire  [31: 0] i_op_a,          //! A operand
  input  wire  [31: 0] i_op_b,          //! B operand
  input  wire  [32: 0] i_op_a_md,       //! A operand from muldiv
  input  wire  [32: 0] i_op_b_md,       //! B operand from muldiv
  input  wire          i_use_md,        //! use muldiv as input
  input  sel_alu_op_e  i_sel_op,        //! operation select
  output logic [31: 0] o_adder_out,     //! full length adder output
  output logic [33: 0] o_adder_fout,    //! full length adder output
  output logic [31: 0] o_out,           //! operation output
  output logic         o_done,          //! the operation output is valid
  output logic         o_null_out,      //! result is equal to 0
  output logic         o_ops_eq,        //! A = B
  output logic         o_ops_lt,        //! A < B (signed)
  output logic         o_ops_ltu
);

  /*******************************************************
    Static configuration
  *******************************************************/

  //! a barrel shifter is anything that does not need a sequencer
  localparam logic SH_BARREL = (p_alu_shift_bits >= 32);
  //! bits shifted per sequencer cycle
  localparam int  SH_K      = SH_BARREL ? 32 : p_alu_shift_bits;
  //! layers of the residual barrel: 0 for K = 1, 4 for K = 16
  localparam int  SH_RW     = (SH_K <= 1) ? 0 : $clog2(SH_K);
  //! width of the step counter: 5 - log2(K), at least 1 so the type is legal
  localparam int  SH_CW     = SH_BARREL ? 1 : (5 - SH_RW);

  initial begin
    assert (p_alu_shift_bits inside {1, 2, 4, 8, 16, 32})
      else $error("p_alu_shift_bits must be a power of two in [1:32]");
  end

  wire  [ 4: 0] shift_amount = i_op_b[ 4: 0];

  logic [31: 0] adder_out;
  logic [33: 0] adder_fout;
  logic [32: 0] op_a;
  logic [32: 0] op_b;

  logic [31: 0] out;

  /*******************************************************
    Adder

    The 33-bit form keeps a free carry-in slot in bit 0, which is what lets the
    same adder serve the multiply/divide unit (`i_use_md`) and, when
    `p_alu_share_adder` is set, the subtraction and the comparators.
  *******************************************************/

  //! operations that need `a - b` rather than `a + b`
  wire          sub_op = (i_sel_op == alu_sub) | (i_sel_op == alu_slt) | (i_sel_op == alu_sltu);

  //! the shared adder only switches to two's complement mode for the core's own
  //! operands: a multiply/divide step drives the adder itself
  wire          sub_mode = (p_alu_share_adder != 0) & sub_op & ~i_use_md;

generate
  if (p_ext_rvm || p_alu_share_adder) begin: g_adder
    always_comb begin: adder
      op_a = i_use_md ? i_op_a_md : { i_op_a, 1'b1 };
      //! `a + ~b + 1`: the `+1` rides in the carry-in slot of both operands
      op_b = i_use_md ? i_op_b_md
                      : (sub_mode ? { ~i_op_b, 1'b1 } : { i_op_b, 1'b0 });
      adder_fout = op_a + op_b;
      adder_out = adder_fout[32:1];
    end
  end else begin: g_adder
    always_comb begin: adder
      op_a = 33'd0;
      op_b = 33'd0;
      adder_fout = 34'd0;
      adder_out = i_op_a + i_op_b;
    end
  end
endgenerate

  /*******************************************************
    Comparison

    In shared mode every verdict is decoded from the subtraction above:
    `a >= b` unsigned is exactly the carry out, `a == b` is a null difference,
    and the signed comparison only differs from the sign of the difference when
    the operands have opposite signs -- where the negative one is the smaller.
  *******************************************************/

  wire          diff_null  = (adder_out == 32'd0);
  wire          diff_carry = adder_fout[33];
  wire          opposite   = i_op_a[31] ^ i_op_b[31];

  logic         ops_eq;
  logic         ops_lt;
  logic         ops_ltu;

generate
  if (p_alu_share_adder) begin: g_shared_cmp
    always_comb begin
      ops_eq  = diff_null;
      ops_ltu = ~diff_carry;
      ops_lt  = opposite ? i_op_a[31] : adder_out[31];
    end
  end else begin: g_shared_cmp
    always_comb begin
      ops_eq  = (i_op_a == i_op_b);
      ops_ltu = (i_op_a < i_op_b);
      ops_lt  = ($signed(i_op_a) < $signed(i_op_b));
    end
  end
endgenerate

  /*******************************************************
    Shifter
  *******************************************************/

  wire          shift_op = (i_sel_op == alu_sll) | (i_sel_op == alu_srl) | (i_sel_op == alu_sra);

  logic [31: 0] shift_out;                //! shifter result
  logic         shift_done;               //! the shifter result is valid

generate
  if (SH_BARREL) begin: g_shifter
    //! plain barrel shifter: five mux layers, one cycle
    always_comb begin
      case (i_sel_op)
        alu_sll : shift_out = i_op_a <<  shift_amount;
        alu_srl : shift_out = i_op_a >>  shift_amount;
        alu_sra : shift_out = $signed(i_op_a) >>> shift_amount;
        default : shift_out = 32'd0;
      endcase
      shift_done = 1'b1;
    end
  end else begin: g_shifter
    //! residual applied in the launch cycle: `shift_amount % K`, `log2(K)` layers
    wire  [ 4: 0] resid  = shift_amount & 5'(SH_K - 1);
    //! number of constant K-bit steps still owed after the residual
    wire  [SH_CW-1: 0] nsteps = shift_amount[4 : SH_RW];

    logic [31: 0] sh_first;               //! operand after the residual shift
    logic [31: 0] sh_step;                //! current value after one K-bit step
    logic [31: 0] sh_q;                   //! sequencer accumulator
    logic [SH_CW-1: 0] sh_cnt;            //! steps still owed
    logic [ 1: 0] sh_state;               //! sequencer state

    //! Three states, for the same reason the multiply/divide unit needs them:
    //! `i_start` is a level, not a pulse -- it stays asserted for as long as the
    //! consumer holds the instruction, and with two shifts back to back it never
    //! falls between them. A run that ended therefore cannot be distinguished
    //! from a run that should begin by looking at `i_start` alone. `SH_FIN` is
    //! the cycle that publishes the result: the sequencer refuses to relaunch
    //! there, and leaves for `SH_IDLE` unconditionally, so the next instruction
    //! starts from a clean state whether or not `i_start` ever fell.
    localparam logic [1:0] SH_IDLE = 2'd0;
    localparam logic [1:0] SH_RUN  = 2'd1;
    localparam logic [1:0] SH_FIN  = 2'd2;

    always_comb begin
      case (i_sel_op)
        alu_sll : sh_first = i_op_a <<  resid;
        alu_srl : sh_first = i_op_a >>  resid;
        alu_sra : sh_first = $signed(i_op_a) >>> resid;
        default : sh_first = 32'd0;
      endcase
      //! a constant shift is pure wiring: only the direction mux costs anything
      case (i_sel_op)
        alu_sll : sh_step = sh_q <<  SH_K;
        alu_srl : sh_step = sh_q >>  SH_K;
        alu_sra : sh_step = $signed(sh_q) >>> SH_K;
        default : sh_step = 32'd0;
      endcase
    end

    //! a shift long enough to need the sequencer at all. Anything shorter is
    //! covered by the residual alone and stays a one-cycle operation.
    wire          seq_needed = shift_op & (nsteps != {SH_CW{1'b0}});
    wire          launch     = (sh_state == SH_IDLE) & i_start & seq_needed;

    always_ff @(posedge i_clk) begin: shift_sequencer
      if (i_rst) begin
        sh_q     <= 32'd0;
        sh_cnt   <= {SH_CW{1'b0}};
        sh_state <= SH_IDLE;
      end else begin
        case (sh_state)
          SH_IDLE: begin
            if (launch) begin
              sh_q     <= sh_first;
              sh_cnt   <= nsteps;
              sh_state <= SH_RUN;
            end
          end
          SH_RUN: begin
            sh_q     <= sh_step;
            sh_cnt   <= sh_cnt - 1'b1;
            if (sh_cnt == {{(SH_CW-1){1'b0}}, 1'b1}) begin
              sh_state <= SH_FIN;
            end
          end
          default: begin
            sh_state <= SH_IDLE;
          end
        endcase
      end
    end

    always_comb begin
      //! a shift that skipped the sequencer reads the residual path directly;
      //! every other case reads the accumulator, which still holds the result
      //! of the last run during `SH_FIN`
      shift_out  = ((sh_state == SH_IDLE) & (nsteps == {SH_CW{1'b0}})) ? sh_first : sh_q;
      shift_done = (sh_state != SH_RUN) & ~launch;
    end
  end
endgenerate

  /*******************************************************
    Operation mux
  *******************************************************/

  always_comb begin: opereration_mux
    case (i_sel_op)
      alu_add  : out = adder_out;
      alu_sub  : out = (p_alu_share_adder != 0) ? adder_out : (i_op_a - i_op_b);
      alu_and  : out = i_op_a & i_op_b;
      alu_or   : out = i_op_a | i_op_b;
      alu_xor  : out = i_op_a ^ i_op_b;
      alu_slt  : out = { 31'd0, ops_lt  };
      alu_sltu : out = { 31'd0, ops_ltu };
      alu_sll  ,
      alu_srl  ,
      alu_sra  : out = shift_out;
      alu_cpa  : out = i_op_a;
      alu_cpb  : out = i_op_b;
      default  : out = 32'b0;
    endcase
  end

  assign o_out        = out;
  assign o_done       = shift_done;
  assign o_null_out   = out == 0 ? 1'b1 : 1'b0;
  assign o_adder_out  = adder_out;
  assign o_adder_fout = adder_fout;
  assign o_ops_eq     = ops_eq;
  assign o_ops_lt     = ops_lt;
  assign o_ops_ltu    = ops_ltu;

endmodule

`endif // __CPU_ALU__
