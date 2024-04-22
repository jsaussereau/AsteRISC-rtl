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


`ifndef __CPU_FETCH__
`define __CPU_FETCH__

`ifdef VIVADO
 `include "packages/pck_control.sv"
 `include "packages/pck_isa.sv"
`else
 `include "core/packages/pck_control.sv"
 `include "core/packages/pck_isa.sv"
`endif

//FIXME: handle instruction cut between two addresses

module cpu_fetch 
  import pck_control::*;
  import pck_isa::*;
#(
  parameter p_reset_vector = 32'hf0000000,
  parameter p_prefetch_buf = 0,           //! use a prefetch buffer
  parameter p_branch_buf   = 0,           //! add buffers to alu comp outputs (+1 cycle for conditionnal branches)
  parameter p_counters     = 0            //! use minstret counter
)(
  input  wire          i_clk,             //! global clock
  input  wire          i_rst,             //! global reset
  input  wire          i_sleep,           //! active high sleep control

  // memory port
  //mem_bus.cpu_side     ibus,              //! instruction bus
  output logic [31: 0] ibus_addr,         //! data bus address
  output logic [ 3: 0] ibus_be,           //! data bus write byte enable
  output logic         ibus_wr_en,        //! data bus write enable
  output logic [31: 0] ibus_wr_data,      //! data bus write data
  output logic         ibus_rd_en,        //! data bus read enable
  input  logic [31: 0] ibus_rd_data,      //! data bus read data
  input  logic         ibus_busy,         //! data bus busy
  input  logic         ibus_ack,          //! data bus transfer acknowledge
 
  // control signals
  input  wire          i_en_fetch,        //! fetch enable
  input  wire          i_update_pc,       //! update program counter
  input  wire          i_refetch,         //! 
  input  wire          i_freeze_pc,       //! 
  input  wire          i_compressed,      //! instruction is compressed
  input  wire          i_offset_pc,       //! force fetching next pc
  input  wire          i_wb_state,        //! currently in write back state
  input  sel_pc_e      i_sel_pc,          //! program counter select
  input  wire  [31: 0] i_alu_out,         //! ALU output
  input  wire  [31: 0] i_imm,             //! immediate value
  input  wire  [31: 0] i_rf_rd1_data,     //! regfile data read on port 1
  input  wire          i_cond_br_bp,      //! conditionnal branch (inconditionnal branches jal and jalr are excluded) (unregistered)
  input  wire          i_br_instr_bp,     //! branch instruction (unregistered)
  input  wire          i_jalr_instr_bp,   //! jalr instruction (unregistered)
  input  wire  [31: 0] i_imm_bp,          //! immediate value (unregistered)
  output wire  [31: 0] o_pc,              //! program counter
  output wire  [31: 0] o_pc_inc,          //! program counter +4 / +2

  // fectched instruction 
  output isa_instr_t   o_instr,           //! instruction to 'decode' stage
  output wire          o_bad_predict,     //! bad branch prediction
  output wire          o_bypass_decomp,   //! bypass decompressor
  output wire          o_half_pc_addr,    //! high when the pc is not a multiple of 4
  output wire          o_half_npc_addr,   //! high when the next pc is not a multiple of 4
  output wire  [63: 0] o_minstret         //! 
);

  logic         half_pc_addr;
  logic         half_npc_addr;
  logic         bypass_decomp;

  logic [31: 0] alu_out;                  //! ALU output
  logic [31: 0] alu_out_reg;              //! ALU output

  logic [31: 0] fetch_addr;               //! fetch address
  logic [31: 0] pc;                       //! program counter
  logic [31: 0] pc_inc;                   //! program counter +4 / +2
  logic [31: 0] next_pc;                  //! next program counter value
  logic [31: 0] computed_pc;              //! computed pc value
  logic [31: 0] predicted_pc;             //! predicted pc value
  logic         bad_predict;
  wire          bad_predict_gate;
  wire          bad_predict_pulse;

  logic         branch_instr;             //!
  logic         fetch_alternate;          //!
  logic [31: 0] ibus_rd_data_saved;       //!

  sel_pc_e      sel_pc_reg;               //!
  logic         en_fetch_reg;             //!
  logic [31: 0] imm_reg;                  //!
  logic [31: 0] rf_rd1_data_reg;          //!
  logic [31: 0] pc_reg;                   //!
  logic [31: 0] pc_inc_reg;               //!
  logic [31: 0] pc_reg_out;               //!
  logic [31: 0] pc_inc_reg_out;           //!
  logic [63: 0] minstret_reg_out;         //! 
  logic [31: 0] computed_pc_reg;          //! computed pc value
  logic [31: 0] predicted_pc_reg;         //! predicted pc value
  logic [31: 0] ibus_rd_data_reg;         //! buffered intruction from memory
  logic         bad_predict_reg;          //!
  logic         fetch_alternate_reg;      //!
  logic         bad_predict_gate_reg;

  isa_instr_t   instr;                    //! instruction to 'decode' stage

  
  logic         debug_flag;
  assign debug_flag = pc == 32'hf00000dc;

  logic [63: 0] minstret;

  //! PC: program counter
  //TODO: add a condition to counters
  always_ff @(posedge i_clk) begin: program_counter
    if (i_rst) begin
      pc <= p_reset_vector - 4;
      minstret <= 64'd0;
    end else begin
      if (i_update_pc && !i_sleep) begin
        pc <= next_pc;
        minstret <= minstret + 1;
      end
      if (bad_predict_pulse && branch_instr) begin
        pc <= next_pc;
      end
    end
  end

  always_comb begin
    half_npc_addr = next_pc[1];
    half_pc_addr  = pc[1];
    alu_out       = (p_branch_buf && i_sel_pc == pc_alu) ? alu_out_reg : i_alu_out;
    bypass_decomp = (pc == p_reset_vector - 4) ? 1'b1 : 1'b0;
    pc_inc        = i_compressed ? pc + 2 : pc + 4;
  end

  always_ff @(posedge i_clk) begin
    if (!(fetch_alternate && bad_predict)) begin
      pc_reg_out        <= pc;
      pc_inc_reg_out    <= pc_inc;
      minstret_reg_out  <= minstret;
    end
    pc_reg              <= pc;
    pc_inc_reg          <= pc_inc;
    alu_out_reg         <= i_alu_out;
    imm_reg             <= i_imm;
    rf_rd1_data_reg     <= i_rf_rd1_data;
    sel_pc_reg          <= i_sel_pc;
    en_fetch_reg        <= i_en_fetch;
    ibus_rd_data_reg    <= ibus_rd_data;
    //computed_pc_reg     <= computed_pc;
    predicted_pc_reg    <= predicted_pc;
    bad_predict_gate_reg<= bad_predict_gate;
    fetch_alternate_reg <= fetch_alternate;
  end

  if (p_prefetch_buf) begin
    //! compute the next program counter value
    always_comb begin: compute_pc_value
      if (i_freeze_pc) begin
        computed_pc_reg = pc_reg;
      end else begin
        case (sel_pc_reg)
          pc_none   : computed_pc_reg = pc_reg;
          pc_plus_4 : computed_pc_reg = pc_inc_reg;
          pc_alu    : computed_pc_reg = alu_out_reg & 32'hfffffffe;
          pc_imm    : computed_pc_reg = pc_reg + $signed(imm_reg);
          pc_rf     : computed_pc_reg = rf_rd1_data_reg;
          default   : computed_pc_reg = pc_inc_reg;
        endcase
      end
    end
  end else begin
    always_comb begin: compute_pc_value
      if (i_freeze_pc) begin
        computed_pc = pc;
      end else begin
        case (i_sel_pc)
          pc_none   : computed_pc = pc;
          pc_plus_4 : computed_pc = pc_inc;
          pc_alu    : computed_pc = alu_out & 32'hfffffffe;
          pc_imm    : computed_pc = pc + $signed(i_imm);
          pc_rf     : computed_pc = i_rf_rd1_data;
          default   : computed_pc = pc_inc;
        endcase
      end
    end
  end

  always_comb begin: select_next_pc
    if (p_prefetch_buf) begin
      /*if (!branch_instr) begin
        next_pc = computed_pc;
      end else */if (fetch_alternate && bad_predict) begin
        next_pc = computed_pc_reg;
      end else begin
        next_pc = predicted_pc_reg;
      end
    end else begin
      next_pc = computed_pc;
    end
  end

  // optionnal prefetch buffer
  if (p_prefetch_buf) begin
    cpu_branch_predictor #(
      .p_reset_vector ( p_reset_vector  ),
      .p_branch_buf   ( p_branch_buf    ),
      .p_mini_decoder ( 0               )
    ) branch_predictor ( 
      .i_clk          ( i_clk           ),
      .i_rst          ( i_rst           ),
      .i_pc           ( pc              ),
      .i_instr        ( ibus_rd_data    ),
      .i_en           ( i_en_fetch      ),
      .i_cond_br_bp   ( i_cond_br_bp    ),
      .i_br_instr_bp  ( i_br_instr_bp   ),
      .i_jalr_instr_bp( i_jalr_instr_bp ),
      .i_imm_bp       ( i_imm_bp        ),
      .o_predicted_pc ( predicted_pc    ),
      .o_branch_instr ( branch_instr    )
    );
  end else begin
    always_comb begin
      predicted_pc = 32'b0;
      branch_instr = 1'b0;
      /*bad_predict  =  1'b0;*/
    end
  end

  // select the fetch address
  always_comb begin: select_fetch_addr
    fetch_addr = next_pc;
  end

  always_comb begin
    fetch_alternate = p_prefetch_buf & en_fetch_reg & branch_instr;
    /*if (fetch_alternate_reg) begin
      ibus_rd_data_saved = ibus_rd_data;
    end*/
  end

  always_ff @(posedge i_clk) begin
    if (fetch_alternate) begin
      ibus_rd_data_saved <= ibus_rd_data;
    end
  end

  //! instruction fetch from imem
  always_ff @(posedge i_clk) begin: fetch
    ibus_rd_en   <= i_en_fetch | (fetch_alternate & bad_predict);
    ibus_wr_data <= 32'd0;
    ibus_wr_en   <= 1'b0;
    ibus_be      <= 4'b0000;
    if (i_en_fetch || fetch_alternate) begin
      if (i_offset_pc) begin
        ibus_addr  <= (fetch_addr + 2) & ~p_reset_vector;
      end else begin
        ibus_addr  <= fetch_addr & ~p_reset_vector;
      end
    end
  end

  // check if prediction was right
  always_comb begin
    if (predicted_pc_reg != computed_pc_reg) begin
      bad_predict = p_prefetch_buf; // only a bad prediction if prediction is enabled
    end else begin
      bad_predict = 1'b0;
    end
  end

  // output buffered instruction if prefetch is enabled
  always_comb begin
    if (p_prefetch_buf) begin
      if (fetch_alternate_reg) begin
        instr.code = bad_predict ? ibus_rd_data_saved : ibus_rd_data;
      end else begin
        instr.code = ibus_rd_data;
      end
    end else begin
      instr.code = ibus_rd_data;
    end
  end

  assign bad_predict_gate = bad_predict & i_wb_state;
  assign bad_predict_pulse = bad_predict_gate & (bad_predict_gate^bad_predict_gate_reg);

  // assign outputs
  assign o_instr.code    = instr;
  assign o_pc            = pc_reg_out;
  assign o_pc_inc        = pc_inc_reg_out;
  assign o_minstret      = p_counters ? minstret_reg_out : 64'b0;
  assign o_bad_predict   = bad_predict_pulse;
  assign o_bypass_decomp = bypass_decomp;
  assign o_half_npc_addr = half_npc_addr;
  assign o_half_pc_addr  = half_pc_addr;

endmodule

`endif // __CPU_FETCH__
