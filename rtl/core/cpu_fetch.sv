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
  //! branch prediction scheme: 0 = off, 1 = static (backward taken / forward
  //! not taken). Higher values are reserved for the dynamic predictors to come.
  parameter p_branch_pred  = 0,           //! branch prediction scheme
  parameter p_fetch_buf    = 0,           //! add buffers to fetch stage output
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
  input  wire          i_speculate_branch,//! launch a speculative conditional branch fetch
  input  wire          i_resolve_branch,  //! resolve a speculative conditional branch fetch
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
  logic [31: 0] resolved_pc;              //! resolved pc value for a speculative branch
  logic         bad_predict;
  wire          start_prediction;

  logic [31: 0] pc_reg_out;               //!
  logic [31: 0] pc_inc_reg_out;           //!
  logic [63: 0] minstret_reg_out;         //! 
  logic [31: 0] predicted_pc_reg;         //! predicted pc value
  logic [31: 0] predicted_src_pc_reg;     //! source pc of a speculative branch
  logic [31: 0] predicted_src_pc_inc_reg; //! source sequential pc of a speculative branch
  logic [31: 0] predicted_src_imm_reg;    //! source immediate of a speculative branch
  logic [31: 0] predicted_src_alu_out_reg;//! source alu result of a speculative branch
  logic [31: 0] predicted_src_rf_rd1_data_reg; //! source register data of a speculative branch
  logic         predicted_valid;          //! a speculative conditional branch is in flight
  logic [31: 0] instr_code;               //! instruction word selected from the bus
  logic [31: 0] ibus_rd_data_reg;         //! buffered intruction from memory

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
      if ((i_update_pc || start_prediction) && !i_sleep) begin
        if (!i_resolve_branch || bad_predict) begin
          pc <= next_pc;
        end
        if (i_update_pc) begin
          minstret <= minstret + 1;
        end
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
    if (!bad_predict) begin
      pc_reg_out        <= pc;
      pc_inc_reg_out    <= pc_inc;
      minstret_reg_out  <= minstret;
    end
    alu_out_reg         <= i_alu_out;
    ibus_rd_data_reg    <= instr_code;
  end

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

  always_comb begin: compute_resolved_pc
    if (i_freeze_pc) begin
      resolved_pc = predicted_src_pc_reg;
    end else begin
      case (i_sel_pc)
        pc_none   : resolved_pc = predicted_src_pc_reg;
        pc_plus_4 : resolved_pc = predicted_src_pc_inc_reg;
        pc_alu    : resolved_pc = predicted_src_alu_out_reg & 32'hfffffffe;
        pc_imm    : resolved_pc = predicted_src_pc_reg + $signed(predicted_src_imm_reg);
        pc_rf     : resolved_pc = predicted_src_rf_rd1_data_reg;
        default   : resolved_pc = predicted_src_pc_inc_reg;
      endcase
    end
  end

  assign start_prediction = (p_branch_pred == 1) && i_speculate_branch;

  always_ff @(posedge i_clk) begin
    if (i_rst) begin
      predicted_valid             <= 1'b0;
      predicted_pc_reg            <= 32'd0;
      predicted_src_pc_reg        <= 32'd0;
      predicted_src_pc_inc_reg    <= 32'd0;
      predicted_src_imm_reg       <= 32'd0;
      predicted_src_alu_out_reg   <= 32'd0;
      predicted_src_rf_rd1_data_reg <= 32'd0;
    end else begin
      if (i_resolve_branch && predicted_valid) begin
        predicted_valid <= 1'b0;
      end

      if (start_prediction) begin
        predicted_valid               <= 1'b1;
        predicted_pc_reg              <= predicted_pc;
        predicted_src_pc_reg          <= pc;
        predicted_src_pc_inc_reg      <= pc_inc;
        predicted_src_imm_reg         <= i_imm;
        predicted_src_alu_out_reg     <= alu_out;
        predicted_src_rf_rd1_data_reg <= i_rf_rd1_data;
      end
    end
  end

  always_comb begin: select_next_pc
    if (bad_predict) begin
      next_pc = resolved_pc;
    end else if (start_prediction) begin
      next_pc = predicted_pc;
    end else begin
      next_pc = computed_pc;
    end
  end

  // optionnal branch predictor (p_branch_pred == 1: static scheme)
  if (p_branch_pred == 1) begin
    cpu_branch_predictor #(
      .p_reset_vector ( p_reset_vector  ),
      .p_branch_buf   ( p_branch_buf    ),
      .p_mini_decoder ( 0               )
    ) branch_predictor ( 
      .i_clk          ( i_clk           ),
      .i_rst          ( i_rst           ),
      .i_pc           ( pc              ),
      .i_pc_inc       ( pc_inc          ),
      .i_instr        ( ibus_rd_data    ),
      .i_en           ( i_en_fetch      ),
      .i_cond_br_bp   ( i_cond_br_bp    ),
      .i_br_instr_bp  ( i_br_instr_bp   ),
      .i_jalr_instr_bp( i_jalr_instr_bp ),
      .i_imm_bp       ( i_imm_bp        ),
      .o_predicted_pc ( predicted_pc    ),
      .o_predict_taken(                 ),
      .o_branch_instr (                 )
    );
  end else begin
    always_comb begin
      predicted_pc = 32'b0;
    end
  end

  // select the fetch address
  always_comb begin: select_fetch_addr
    fetch_addr = next_pc;
  end

  //! instruction fetch from imem
  always_ff @(posedge i_clk) begin: fetch
    ibus_rd_en   <= i_en_fetch | bad_predict;
    ibus_wr_data <= 32'd0;
    ibus_wr_en   <= 1'b0;
    ibus_be      <= 4'b0000;
    if (i_en_fetch || bad_predict) begin
      if (i_offset_pc) begin
        ibus_addr  <= (fetch_addr + 2) & ~p_reset_vector;
      end else begin
        ibus_addr  <= fetch_addr & ~p_reset_vector;
      end
    end
  end

  // check if prediction was right
  always_comb begin
    if (predicted_valid && i_resolve_branch &&
        (predicted_pc_reg != resolved_pc)) begin
      bad_predict = 1'b1;
    end else begin
      bad_predict = 1'b0;
    end
  end

  // The wrong-path instruction is never presented to the decompressor.
  always_comb begin
    instr_code = ibus_rd_data;
  end

  //! optionnal fetch stage output buffer: cuts the imem read data path at the
  //! cost of one extra cycle per instruction (Fmax vs IPC trade-off)
  assign instr.code = p_fetch_buf ? ibus_rd_data_reg : instr_code;

  // assign outputs
  assign o_instr.code    = instr;
  assign o_pc            = pc_reg_out;
  assign o_pc_inc        = pc_inc_reg_out;
  assign o_minstret      = p_counters ? minstret_reg_out : 64'b0;
  assign o_bad_predict   = bad_predict;
  assign o_bypass_decomp = bypass_decomp;
  assign o_half_npc_addr = half_npc_addr;
  assign o_half_pc_addr  = half_pc_addr;

endmodule

`endif // __CPU_FETCH__
