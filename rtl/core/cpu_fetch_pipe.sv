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


`ifndef __CPU_FETCH_PIPE__
`define __CPU_FETCH_PIPE__

`ifdef VIVADO
 `include "packages/pck_control.sv"
 `include "packages/pck_isa.sv"
`else
 `include "core/packages/pck_control.sv"
 `include "core/packages/pck_isa.sv"
`endif

//FIXME: handle instruction cut between two addresses

module cpu_fetch_pipe 
  import pck_control::*;
  import pck_isa::*;
#(
  parameter p_reset_vector = 32'hf0000000,
  parameter p_branch_buf   = 0            //! add buffers to alu comp outputs (+1 cycle for conditionnal branches)
)(
  input  wire          i_clk,             //! global clock
  input  wire          i_rst,             //! global reset
  input  wire          i_sleep,           //! active high sleep control

  // memory port
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
  input  wire          i_update_instret,  //! update instret
  input  wire          i_refetch,         //! 
  input  wire          i_freeze_pc,       //! 
  input  wire          i_freeze_pc_test,  //! 
  input  wire          i_compressed,      //! instruction is compressed
  input  wire          i_offset_pc,       //! force fetching next pc
  input  sel_pc_e      i_sel_pc,          //! program counter select
  input  wire  [31: 0] i_alu_out,         //! ALU output
  input  wire  [31: 0] i_imm,             //! immediate value
  input  wire  [31: 0] i_rf_rd1_data,     //! regfile data read on port 1
  input  wire  [31: 0] i_jump_pc,         //! program counter (used for sel_pc == pc_imm)
  input  wire          i_force_instret,   //! force instret value to i_instret
  input  wire  [63: 0] i_minstret,        //! instret value to be forced
  /*
  input  wire          i_bp_pc_en,        //! enable pc bypass
  input  wire  [31: 0] i_bp_pc_addr,      //! pc bypass address
  */
  output logic [31: 0] o_pc,              //! program counter
  output logic [31: 0] o_pc_inc,          //! program counter +4 / +2

  // fectched instruction 
  output isa_instr_t   o_instr,           //! instruction to 'decode' stage
  output logic         o_bypass_decomp,   //! bypass decompressor
  output logic         o_half_pc_addr,    //! high when the pc is not a multiple of 4
  output logic         o_half_npc_addr,   //! high when the next pc is not a multiple of 4
  output logic [63: 0] o_minstret       
);

  logic         half_pc_addr;
  logic         half_npc_addr;

  wire  [31: 0] alu_out;                  //! ALU output
  logic [31: 0] alu_out_sync;             //! ALU output

  logic [31: 0] fetch_addr;               //! fetch address
  logic [31: 0] pc;                       //! program counter
  logic [31: 0] pc_old;                   //! 
  logic [31: 0] pc_inc;                   //! program counter +4 / +2
  logic [31: 0] next_pc;                  //! next program counter value
  logic [31: 0] next_pc_tmp;

  logic         debug_flag;
  assign debug_flag = pc == 32'hf0000930;

  logic [63: 0] minstret;

  isa_instr_t   instr;
  isa_instr_t   instr_last;
  logic         freeze_pc_reg;

  //! PC: program counter
  always_ff @(posedge i_clk) begin: program_counter
    if (i_rst) begin
      pc <= p_reset_vector - 4;
      pc_old <= p_reset_vector - 4;
    end else begin
      if (i_update_pc && !i_sleep) begin
        pc <= next_pc_tmp;
        pc_old <= pc;
      end
    end
  end
  //assign o_pc = pc;

 //! instruction counter
  always_ff @(posedge i_clk) begin: instr_counter
    if (i_rst) begin
      minstret <= 64'd0;
    end else begin
      if (i_force_instret) begin
        minstret <= i_minstret;
      end else if (i_update_instret) begin
        minstret <= minstret + 1;
      end
    end
  end

  // assign outputs
  assign half_npc_addr = next_pc[1];
  assign half_pc_addr  = pc[1];
  always_comb begin
    o_half_npc_addr = half_npc_addr;
    o_half_pc_addr  = half_pc_addr;
    o_bypass_decomp = (pc == p_reset_vector - 4) ? 1'b1 : 1'b0;
  end

  assign pc_inc       = i_compressed ? pc + 2 : pc + 4;
  /*always_ff @(posedge i_clk) begin
    o_pc_inc      <= pc_inc;
    o_pc          <= pc;
    o_minstret    <= minstret;
  end*/
  always_comb begin
    o_pc_inc      = pc/*_inc*/;
    o_pc          = pc_old;
    o_minstret    = minstret;
  end

  always_ff @(posedge i_clk) begin
    alu_out_sync <= i_alu_out;
  end

  assign alu_out = (p_branch_buf && i_sel_pc == pc_alu) ? alu_out_sync : i_alu_out;

  always_comb begin: next_pc_value
    if (i_freeze_pc) begin
      next_pc_tmp = pc;
    end else begin
      case (i_sel_pc)
        pc_none   : next_pc_tmp = pc;
        pc_plus_4 : next_pc_tmp = pc_inc;
        pc_alu    : next_pc_tmp = alu_out & 32'hfffffffe;
        pc_imm    : next_pc_tmp = i_jump_pc + $signed(i_imm);
        pc_rf     : next_pc_tmp = i_rf_rd1_data;
        default   : next_pc_tmp = pc_inc;
      endcase
    end
  end

  always_comb begin
      case (i_sel_pc)
        pc_none   : next_pc = pc;
        pc_plus_4 : next_pc = pc_inc;
        pc_alu    : next_pc = alu_out & 32'hfffffffe;
        pc_imm    : next_pc = i_jump_pc + $signed(i_imm);
        pc_rf     : next_pc = i_rf_rd1_data;
        default   : next_pc = pc_inc;
      endcase
  end

  //! instruction fetch in imem
  always_ff @(posedge i_clk) begin: fetch
    ibus_rd_en     <= 1'b1;
    //if (i_en_fetch) begin
      ibus_wr_en   <= 1'b0;
      ibus_be      <= 4'b0000;
      if (i_offset_pc) begin
        fetch_addr  <= (next_pc + 2) & ~p_reset_vector;
      end else begin
        fetch_addr  <= next_pc & ~p_reset_vector;
      end
      ibus_wr_data <= 32'd0;
    //end 
  end 

  always_ff @(posedge i_clk) begin
    freeze_pc_reg <= i_freeze_pc_test;
    //if (!i_freeze_pc_test) begin
      instr_last <= instr;
    //end
  end 


  always_comb begin/*
    if (!i_freeze_pc_test) begin*/
      ibus_addr = fetch_addr;
    /*end else begin
      ibus_addr = pc;
    end*/
  end

  always_comb begin
    if (i_rst) begin
      instr = ibus_rd_data;
    end else begin
      instr = (i_en_fetch && !freeze_pc_reg) ? ibus_rd_data : instr_last;
      //instr = (i_en_fetch) ? ibus_rd_data : instr;
    end
  end

  always_comb begin
    o_instr.code = instr;
  end

endmodule

`endif // __CPU_FETCH_PIPE__
