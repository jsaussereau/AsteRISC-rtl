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

//! CPU hazard unit

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
  parameter p_stage_IF = 1,
  parameter p_stage_IC = 0,
  parameter p_stage_ID = 1,
  parameter p_stage_RF = 0,
  parameter p_stage_EX = 1,
  parameter p_stage_MA = 1,
  parameter p_stage_WB = 1
)(
  input  wire          i_clk,             //! global clock
  input  wire          i_rst,             //! global reset
  input  wire  [ 4: 0] i_rf_rd1_addr,     //! register file read address 1 (rs1) (IF stage)
  input  wire  [ 4: 0] i_rf_rd2_addr,     //! register file read address 2 (rs2) (IF stage)
  input  wire          i_rf_rd2_used,     //! register file read port 2 is used  (IF stage)
  input  wire  [ 4: 0] i_rf_rd1_addr_ID,  //! register file read address 1 (rs1) (ID stage)
  input  wire  [ 4: 0] i_rf_rd2_addr_ID,  //! register file read address 2 (rs2) (ID stage)
  input  wire          i_rf_rd2_used_ID,  //! register file read port 2 is used  (ID stage)
  input  wire  [ 4: 0] i_rf_rd1_addr_RF,  //! register file read address 1 (rs1) (RF stage)
  input  wire  [ 4: 0] i_rf_rd2_addr_RF,  //! register file read address 2 (rs2) (RF stage)
  input  wire          i_rf_rd2_used_RF,  //! register file read port 2 is used  (RF stage)
  input  wire  [ 4: 0] i_rf_rd1_addr_EX,  //! register file read address 1 (rs1) (EX stage)
  input  wire  [ 4: 0] i_rf_rd2_addr_EX,  //! register file read address 2 (rs2) (EX stage)
  input  wire          i_rf_rd2_used_EX,  //! register file read port 2 is used  (EX stage)
  input  wire  [ 4: 0] i_rf_rd1_addr_MA,  //! register file read address 1 (rs1) (MA stage)
  input  wire  [ 4: 0] i_rf_rd2_addr_MA,  //! register file read address 2 (rs2) (MA stage)
  input  wire          i_rf_rd2_used_MA,  //! register file read port 2 is used  (MA stage)
  input  wire  [ 4: 0] i_wb_addr_ID,      //! register file write address (rd) (IF stage)
  input  wire  [ 4: 0] i_wb_addr_RF,      //! register file write address (rd) (RF stage)
  input  wire  [ 4: 0] i_wb_addr_EX,      //! register file write address (rd) (EX stage)
  input  wire  [ 4: 0] i_wb_addr_MA,      //! register file write address (rd) (MA stage)
  input  wire  [ 4: 0] i_wb_addr_WB,      //! register file write address (rd) (WB stage)
  input  wire  [31: 0] i_pc,              //! program counter (IF stage)
  input  sel_br_e      i_sel_br,          //! program counter mux selector
  input  sel_br_e      i_sel_br_ID,       //! program counter mux selector
  input  sel_br_e      i_sel_br_RF,       //! program counter mux selector
  input  sel_br_e      i_sel_br_EX,       //! program counter mux selector
  input  sel_br_e      i_sel_br_MA,       //! program counter mux selector
  input  sel_wb_e      i_sel_wb,          //! write back source selector (IF stage)
  input  sel_wb_e      i_sel_wb_ID,       //! write back source selector (ID stage)
  input  sel_wb_e      i_sel_wb_RF,       //! write back source selector (RF stage)
  input  sel_wb_e      i_sel_wb_EX,       //! write back source selector (EX stage)
  input  sel_wb_e      i_sel_wb_MA,       //! write back source selector (MA stage)
  input  sel_wb_e      i_sel_wb_WB,       //! write back source selector (WB stage)
  input  wire          i_branch_taken,
  input  wire          i_branch_taken_EX,
  input  wire          i_branch_taken_MA,
  input  wire          i_valid_IF,        //! the current instruction in stage IF is meant to be executed
  input  wire          i_valid_IC,        //! the current instruction in stage IC is meant to be executed
  input  wire          i_valid_ID,        //! the current instruction in stage ID is meant to be executed
  input  wire          i_valid_RF,        //! the current instruction in stage RF is meant to be executed
  input  wire          i_valid_EX,        //! the current instruction in stage EX is meant to be executed
  input  wire          i_valid_MA,        //! the current instruction in stage MA is meant to be executed
  input  wire          i_valid_WB,        //! the current instruction in stage WB is meant to be executed
  output logic         o_squash_IF,
  output logic         o_squash_IC,
  output logic         o_squash_ID,
  output logic         o_squash_RF,
  output logic         o_squash_EX,
  output logic         o_squash_MA,
  output logic         o_squash_WB,
  output logic         o_update_pc,
  output logic         o_update_pc_test,
  output logic         o_update_instret,
  output logic         o_force_instret,
  output logic         o_en_IF,
  output logic         o_en_IC,
  output logic         o_en_ID,
  output logic         o_en_ID_instr,
  output logic         o_en_RF,
  output logic         o_en_RF_addr,
  output logic         o_en_EX,
  output logic         o_en_MA,
  output logic         o_en_WB,
  output logic         o_bp_rs1_rf,       //! bypass rs1 from stage RF
  output logic         o_bp_rs1_ex,       //! bypass rs1 from stage EX
  output logic         o_bp_rs1_ma,       //! bypass rs1 from stage MA
  output logic         o_bp_rs1_wb,       //! bypass rs1 from stage WB
  output logic         o_bp_rs1_pw,       //! bypass rs1 from stage PW (pending write)
  output logic         o_bp_rs2_rf,       //! bypass rs2 from stage RF
  output logic         o_bp_rs2_ex,       //! bypass rs2 from stage EX
  output logic         o_bp_rs2_ma,       //! bypass rs2 from stage MA
  output logic         o_bp_rs2_wb,       //! bypass rs2 from stage WB
  output logic         o_bp_rs2_pw,       //! bypass rs2 from stage PW (pending write)
  output logic         o_bp_jal,          //! bypass for jal
  output logic         o_bp_jalr,         //! bypass for jalr
  output logic         o_bp_branch,       //! bypass for other branches
  output logic         o_stall_rs1,       //! a stall needed for the instruction in ID stage
  output logic         o_stall_rs2        //! a stall needed for the instruction in ID stage
);

logic rst_p1; 

logic pipoutest;

logic hazard_unhandled; 

logic hazard_jal_if_c1; // 1st cycle
logic hazard_jal_if_fixed_c1;
logic hazard_jal_id_c1; // 1st cycle
logic hazard_jal_id_fixed_c1;
logic hazard_jal_id_c2; // 2nd cycle (the fetch stage registers the input address, adding a one-cycle latency) 
logic hazard_jal_id_c3; // 3nd cycle (the fetch stage registers the input address, adding a one-cycle latency) 

logic hazard_jalr_if_c1;
logic hazard_jalr_if_fixed_c1;
logic hazard_jalr_id_c1;
logic hazard_jalr_id_fixed_c1;
logic hazard_jalr_rf_c1;
logic hazard_jalr_rf_fixed_c1;
logic hazard_jalr_ex_c1; // 1st cycle
logic hazard_jalr_ex_c2; // 2nd cycle (the fetch stage registers the input address, adding a one-cycle latency)

logic branch_if;

logic hazard_branch;
logic hazard_branch_id_c1; // 1st cycle
logic hazard_branch_id_c2; // 2nd cycle
logic hazard_branch_rf_c1; // 1st cycle
logic hazard_branch_rf_c2; // 2nd cycle
logic hazard_branch_ex_c1; // 1st cycle
logic hazard_branch_ex_c2; // 2nd cycle (the fetch stage registers the input address, adding a one-cycle latency)
logic hazard_branch_ex_c3; // 3rd cycle (the fetch stage registers the input address, adding a one-cycle latency)

logic hazard_raw;
logic hazard_raw_rs1_id;
logic hazard_raw_rs1_rf;
logic hazard_raw_rs1_ex;
logic hazard_raw_rs1_ma;
logic hazard_raw_rs1_wb;
logic hazard_raw_rs1_pw;
logic hazard_raw_rs2_id;
logic hazard_raw_rs2_rf;
logic hazard_raw_rs2_ex;
logic hazard_raw_rs2_ma;
logic hazard_raw_rs2_wb;
logic hazard_raw_rs2_pw;

logic wb_from_dmem_id; // write back source for instruction in ID stage is DMEM
logic wb_from_dmem_rf; // write back source for instruction in RF stage is DMEM
logic wb_from_dmem_ex; // write back source for instruction in EX stage is DMEM

logic hazard_raw_rs1_stall_id; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs2_stall_id; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs1_stall_rf; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs2_stall_rf; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs1_stall_ex; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs2_stall_ex; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs1_stall_rf_c2; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs2_stall_rf_c2; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs1_stall_ex_c2; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs2_stall_ex_c2; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs1_stall_ex_c3; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs2_stall_ex_c3; // raw with bypass from memory output -> stall needed
logic hazard_raw_rs1_stall_id_c2;
logic hazard_raw_rs2_stall_id_c2;
logic hazard_raw_rs1_stall_id_c3;
logic hazard_raw_rs2_stall_id_c3;

logic squash_IF;
logic squash_IC;
logic squash_ID;
logic squash_RF;
logic squash_EX;
logic squash_MA;
logic squash_WB;

logic [ 4: 0] wb_addr_PW;      //! register file write address (rd) (pending write back)
sel_wb_e      sel_wb_PW;       //! write back source selector (pending write back)

logic branch_taken_ID;

  assign branch_taken_ID = i_branch_taken & ~hazard_branch_ex_c1 & ~squash_EX;
  assign wb_from_dmem_id = (i_sel_wb_ID == wb_dmem) ? 1'b1 : 1'b0;
  assign wb_from_dmem_rf = (i_sel_wb_RF == wb_dmem) ? 1'b1 : 1'b0;
  assign wb_from_dmem_ex = (i_sel_wb_EX == wb_dmem) ? 1'b1 : 1'b0;

  // JAL 
  assign hazard_jal_if_c1 = i_sel_br    == br_jal;
  assign hazard_jal_if_fixed_c1 = hazard_jal_if_c1 
    // make sure the jal instruction must be executed
    & ~hazard_branch_id_c1 
    & (~hazard_branch_id_c2 | p_stage_RF)
    & (~hazard_branch_ex_c1 | ~p_stage_RF)
    & (~hazard_branch_ex_c2 | ~p_stage_RF)
    & (~hazard_branch_ex_c2 | p_stage_ID)
  ;
  assign hazard_jal_id_c1 = i_sel_br_ID == br_jal;
  assign hazard_jal_id_fixed_c1 = hazard_jal_id_c1
    & (~hazard_branch_ex_c2 | p_stage_ID)
  ;

  // JALR
  assign hazard_jalr_if_c1 = i_sel_br    == br_jalr;
  assign hazard_jalr_if_fixed_c1 = hazard_jalr_if_c1 
    // make sure the jalr instruction must be executed
    & ~hazard_branch_id_c1 
    & ~hazard_branch_rf_c1 
    //& (~hazard_branch_id_c2 | p_stage_RF)
    & (~hazard_branch_ex_c1 )
    & (~hazard_branch_ex_c2 )
  ;
  assign hazard_jalr_id_c1 = i_sel_br_ID == br_jalr;
  assign hazard_jalr_id_fixed_c1 = hazard_jalr_id_c1
    & (~hazard_branch_rf_c1 | ~p_stage_RF)
    & (~hazard_branch_ex_c1 | ~p_stage_RF)
    & (~hazard_branch_ex_c2 | p_stage_ID)
  ;
  assign hazard_jalr_rf_c1 = i_sel_br_RF == br_jalr;
  assign hazard_jalr_rf_fixed_c1 = hazard_jalr_rf_c1
    & (~hazard_branch_ex_c1 | ~p_stage_RF)
    & (~hazard_branch_ex_c2 | p_stage_ID)
  ;
  assign hazard_jalr_ex_c1 = i_sel_br_EX == br_jalr;

  // Other branches 
  assign hazard_branch_id_c1 = branch_taken_ID   && i_sel_br_ID != br_none && i_sel_br_ID != br_jal && i_sel_br_ID != br_jalr;
  assign hazard_branch_rf_c1 = branch_taken_ID   && i_sel_br_RF != br_none && i_sel_br_RF != br_jal && i_sel_br_RF != br_jalr;
  assign hazard_branch_ex_c1 = i_branch_taken_EX && i_sel_br_EX != br_none && i_sel_br_EX != br_jal && i_sel_br_EX != br_jalr;
  assign hazard_branch_ma_c1 = i_branch_taken_MA && i_sel_br_MA != br_none && i_sel_br_MA != br_jal && i_sel_br_MA != br_jalr;
  assign hazard_branch       = hazard_branch_id_c1 | (hazard_branch_rf_c1 | hazard_branch_ex_c1) & p_stage_ID;

  // Read after write
  assign hazard_raw_rs1_id  = (i_rf_rd1_addr    == i_wb_addr_ID && i_sel_wb_ID != wb_none && i_rf_rd1_addr    != 5'd0);
  assign hazard_raw_rs1_rf  = (i_rf_rd1_addr_ID == i_wb_addr_RF && i_sel_wb_RF != wb_none && i_rf_rd1_addr_ID != 5'd0);
  assign hazard_raw_rs1_ex  = (i_rf_rd1_addr_RF == i_wb_addr_EX && i_sel_wb_EX != wb_none && i_rf_rd1_addr_RF != 5'd0);
  assign hazard_raw_rs1_ma  = (i_rf_rd1_addr_RF == i_wb_addr_MA && i_sel_wb_MA != wb_none && i_rf_rd1_addr_RF != 5'd0);
  assign hazard_raw_rs1_wb  = (i_rf_rd1_addr_RF == i_wb_addr_WB && i_sel_wb_WB != wb_none && i_rf_rd1_addr_RF != 5'd0);
  assign hazard_raw_rs1_pw  = (i_rf_rd1_addr_RF ==   wb_addr_PW &&   sel_wb_PW != wb_none && i_rf_rd1_addr_RF != 5'd0);

  assign hazard_raw_rs1_stall_id = (hazard_raw_rs1_id && wb_from_dmem_id && ~hazard_branch_rf_c1 && ~hazard_branch_ex_c1);
  assign hazard_raw_rs1_stall_rf = (hazard_raw_rs1_rf && wb_from_dmem_rf);
  assign hazard_raw_rs1_stall_ex = (hazard_raw_rs1_ex && wb_from_dmem_ex);

  assign hazard_raw_rs2_id  = (i_rf_rd2_addr    == i_wb_addr_ID && i_sel_wb_ID != wb_none && i_rf_rd2_addr    != 5'd0 && i_rf_rd2_used);
  assign hazard_raw_rs2_rf  = (i_rf_rd2_addr_ID == i_wb_addr_RF && i_sel_wb_RF != wb_none && i_rf_rd2_addr_ID != 5'd0 && i_rf_rd2_used_ID);
  assign hazard_raw_rs2_ex  = (i_rf_rd2_addr_RF == i_wb_addr_EX && i_sel_wb_EX != wb_none && i_rf_rd2_addr_RF != 5'd0 && i_rf_rd2_used_RF);
  assign hazard_raw_rs2_ma  = (i_rf_rd2_addr_RF == i_wb_addr_MA && i_sel_wb_MA != wb_none && i_rf_rd2_addr_RF != 5'd0 && i_rf_rd2_used_RF);
  assign hazard_raw_rs2_wb  = (i_rf_rd2_addr_RF == i_wb_addr_WB && i_sel_wb_WB != wb_none && i_rf_rd2_addr_RF != 5'd0 && i_rf_rd2_used_RF);
  assign hazard_raw_rs2_pw  = (i_rf_rd2_addr_RF ==   wb_addr_PW &&   sel_wb_PW != wb_none && i_rf_rd2_addr_RF != 5'd0 && i_rf_rd2_used_RF);
  
  assign hazard_raw_rs2_stall_id = (hazard_raw_rs2_id && wb_from_dmem_id);
  assign hazard_raw_rs2_stall_rf = (hazard_raw_rs2_rf && wb_from_dmem_rf);
  assign hazard_raw_rs2_stall_ex = (hazard_raw_rs2_ex && wb_from_dmem_ex);

  assign hazard_raw         = hazard_raw_rs1_rf | hazard_raw_rs1_ex | hazard_raw_rs1_ma | hazard_raw_rs1_wb | hazard_raw_rs2_rf | hazard_raw_rs2_ex | hazard_raw_rs2_ma | hazard_raw_rs2_wb;

  assign hazard_unhandled   = 0;

  assign pipoutest = hazard_branch_id_c1 & (hazard_raw_rs1_stall_ex_c2 | hazard_raw_rs2_stall_ex_c2);
  //assign poupitest = hazard_branch_id_c1 & (hazard_raw_rs1_stall_rf_c3 | hazard_raw_rs2_stall_rf_c3);

  // reset register
  always_ff @(posedge i_clk) begin
    rst_p1               <= i_rst;
  end
  
  // pending write (RAM regfile)
  always_ff @(posedge i_clk) begin
    wb_addr_PW <= i_wb_addr_WB;
    sel_wb_PW  <= i_sel_wb_WB;
  end

  // add a cycle if the fetch stage registers the input address
  always_ff @(posedge i_clk) begin
    if (i_rst) begin
      hazard_jal_id_c2           <= 0;
      hazard_jal_id_c3           <= 0;
      hazard_jalr_ex_c2          <= 0;
      hazard_branch_id_c2        <= 0;
      hazard_branch_rf_c2        <= 0;
      hazard_branch_ex_c2        <= 0;
      hazard_branch_ex_c3        <= 0;
      hazard_raw_rs1_stall_id_c2 <= 0;
      hazard_raw_rs2_stall_id_c2 <= 0;
      hazard_raw_rs1_stall_id_c3 <= 0;
      hazard_raw_rs2_stall_id_c3 <= 0;
      hazard_raw_rs1_stall_rf_c2 <= 0;
      hazard_raw_rs2_stall_rf_c2 <= 0;
      hazard_raw_rs1_stall_ex_c2 <= 0;
      hazard_raw_rs2_stall_ex_c2 <= 0;
      hazard_raw_rs1_stall_ex_c3 <= 0;
      hazard_raw_rs2_stall_ex_c3 <= 0;
    end else begin
      hazard_jal_id_c2           <= hazard_jal_id_fixed_c1;
      hazard_jal_id_c3           <= hazard_jal_id_c2;
      hazard_jalr_ex_c2          <= hazard_jalr_ex_c1;
      hazard_branch_id_c2        <= hazard_branch_id_c1;
      hazard_branch_rf_c2        <= hazard_branch_rf_c1;
      hazard_branch_ex_c2        <= hazard_branch_ex_c1;
      hazard_branch_ex_c3        <= hazard_branch_ex_c2;
      hazard_raw_rs1_stall_id_c2 <= hazard_raw_rs1_stall_id;
      hazard_raw_rs2_stall_id_c2 <= hazard_raw_rs2_stall_id;
      hazard_raw_rs1_stall_id_c3 <= hazard_raw_rs1_stall_id_c2;
      hazard_raw_rs2_stall_id_c3 <= hazard_raw_rs2_stall_id_c2;
      hazard_raw_rs1_stall_rf_c2 <= hazard_raw_rs1_stall_rf;
      hazard_raw_rs2_stall_rf_c2 <= hazard_raw_rs2_stall_rf;
      hazard_raw_rs1_stall_ex_c2 <= hazard_raw_rs1_stall_ex;
      hazard_raw_rs2_stall_ex_c2 <= hazard_raw_rs2_stall_ex;
      hazard_raw_rs1_stall_ex_c3 <= hazard_raw_rs1_stall_ex_c2;
      hazard_raw_rs2_stall_ex_c3 <= hazard_raw_rs2_stall_ex_c2;
    end
  end
  //TODO: make these registers togglable

  assign squash_IF = 0
    | (p_stage_ID & hazard_jal_id_c1) 
    | hazard_jal_id_c2 
    | (~p_stage_ID & hazard_jal_id_c3) 
    | (p_stage_ID & hazard_jalr_id_fixed_c1) 
    | (p_stage_ID & hazard_jalr_rf_fixed_c1) 
    | hazard_jalr_ex_c1 
    | hazard_jalr_ex_c2
  ;
  assign squash_IC = 0;
  assign squash_ID = p_stage_ID & (hazard_branch_ex_c1 | hazard_branch_ex_c2);
  assign squash_RF = p_stage_RF & (hazard_branch_ex_c1 | hazard_branch_ex_c2);
  assign squash_EX = 0
    | hazard_branch_ex_c1
    | hazard_branch_ex_c2 & ~p_stage_ID
    | ((hazard_raw_rs1_stall_ex | hazard_raw_rs2_stall_ex) )
    | ((hazard_raw_rs1_stall_ex_c2 | hazard_raw_rs2_stall_ex_c2) & p_stage_RF)
  ; 
  assign squash_MA = 0;
  assign squash_WB = 0;

  assign o_squash_IF = squash_IF;
  assign o_squash_IC = squash_IC;
  assign o_squash_ID = squash_ID;
  assign o_squash_RF = squash_RF;
  assign o_squash_EX = squash_EX;
  assign o_squash_MA = squash_MA;
  assign o_squash_WB = squash_WB;

  assign o_update_instret = 1
    & ~rst_p1
    & ~hazard_jal_if_fixed_c1
    & ~hazard_jal_id_fixed_c1
    & (~hazard_jal_id_c2 | p_stage_ID)
    & ~hazard_jalr_if_fixed_c1
    & ~hazard_jalr_id_fixed_c1 
    & ~hazard_jalr_rf_fixed_c1 
    & ~hazard_jalr_ex_c1
    & (~hazard_branch_id_c1 | p_stage_EX)
    & ~hazard_raw_rs1_stall_id
    & ~hazard_raw_rs2_stall_id
    & (~hazard_raw_rs1_stall_rf | ~p_stage_RF)
    & (~hazard_raw_rs2_stall_rf | ~p_stage_RF)
    & (~hazard_raw_rs1_stall_ex | hazard_raw_rs1_stall_rf_c2 | p_stage_ID)
    & (~hazard_raw_rs2_stall_ex | hazard_raw_rs1_stall_rf_c2 | p_stage_ID)
  ;

  assign o_update_pc = 1
    & ~hazard_jal_if_fixed_c1
    & ~hazard_jalr_if_fixed_c1
    & ~hazard_jalr_id_fixed_c1 
    & ~hazard_jalr_rf_fixed_c1 
    & (~hazard_branch_id_c1 | p_stage_EX)
    & ~hazard_raw_rs1_stall_id
    & ~hazard_raw_rs2_stall_id
    & (~hazard_raw_rs1_stall_rf | ~p_stage_RF)
    & (~hazard_raw_rs2_stall_rf | ~p_stage_RF)
    & (~hazard_raw_rs1_stall_ex | hazard_raw_rs1_stall_rf_c2 | p_stage_ID)
    & (~hazard_raw_rs2_stall_ex | hazard_raw_rs1_stall_rf_c2 | p_stage_ID)
  ;

  assign o_update_pc_test = 1
    & ~hazard_jal_id_fixed_c1
    & ~hazard_jalr_id_fixed_c1
    & ~hazard_jalr_rf_fixed_c1
    & ~hazard_jalr_ex_c1 
    & ~hazard_branch_ex_c1
    & (~hazard_raw_rs1_stall_rf | ~p_stage_RF)
    & (~hazard_raw_rs2_stall_rf | ~p_stage_RF)
    & (~hazard_raw_rs1_stall_ex | ~p_stage_ID)
    & (~hazard_raw_rs2_stall_ex | ~p_stage_ID)
  ;

  assign o_en_IF = 1;
  assign o_en_IC = 1;
  assign o_en_ID = 1
    & (~hazard_raw_rs1_stall_rf | ~p_stage_RF) 
    & (~hazard_raw_rs2_stall_rf | ~p_stage_RF) 
    & (~hazard_raw_rs1_stall_ex)
    & (~hazard_raw_rs2_stall_ex)
    & (~hazard_branch_id_c1 | ~p_stage_ID)
    & (~hazard_branch_rf_c1 | ~p_stage_ID)
    & (~hazard_branch_ex_c1)
  ;
  assign o_en_ID_instr = 1
    & (~hazard_raw_rs1_stall_ex_c2 | p_stage_ID)
    & (~hazard_raw_rs2_stall_ex_c2 | p_stage_ID)
    & (~hazard_raw_rs1_stall_ex_c3 | p_stage_ID)
    & (~hazard_raw_rs2_stall_ex_c3 | p_stage_ID)
    & (~hazard_raw_rs1_stall_id_c3 | p_stage_ID)
    & (~hazard_raw_rs2_stall_id_c3 | p_stage_ID)
  ;
  assign o_en_RF_addr = 1
    & (~hazard_raw_rs1_stall_rf | ~p_stage_RF)
    & (~hazard_raw_rs2_stall_rf | ~p_stage_RF)
    & (~hazard_raw_rs1_stall_ex | ~p_stage_RF)
    & (~hazard_raw_rs2_stall_ex | ~p_stage_RF)
  ;
  assign o_en_RF = 1
    & ~hazard_raw_rs1_stall_ex 
    & ~hazard_raw_rs2_stall_ex 
    & ~hazard_raw_rs1_stall_ex_c2 
    & ~hazard_raw_rs2_stall_ex_c2 
  ;
  assign o_en_EX = 1;
  assign o_en_MA = 1;
  assign o_en_WB = 1;

  assign o_bp_jal = hazard_jal_if_fixed_c1;
  assign o_bp_jalr = hazard_jalr_rf_fixed_c1;
  assign o_bp_branch = hazard_branch_rf_c1;

  assign o_force_instret = hazard_branch_ex_c2;
  
  assign o_bp_rs1_rf = hazard_raw_rs1_rf;
  assign o_bp_rs1_ex = hazard_raw_rs1_ex & ~wb_from_dmem_ex;
  assign o_bp_rs1_ma = hazard_raw_rs1_ma;
  assign o_bp_rs1_wb = hazard_raw_rs1_wb;
  assign o_bp_rs1_pw = hazard_raw_rs1_pw;
  assign o_bp_rs2_rf = hazard_raw_rs2_rf;
  assign o_bp_rs2_ex = hazard_raw_rs2_ex & ~wb_from_dmem_ex;
  assign o_bp_rs2_ma = hazard_raw_rs2_ma;
  assign o_bp_rs2_wb = hazard_raw_rs2_wb;
  assign o_bp_rs2_pw = hazard_raw_rs2_pw;

  assign o_stall_rs1 = hazard_raw_rs1_stall_id | hazard_raw_rs1_stall_rf;
  assign o_stall_rs2 = hazard_raw_rs2_stall_id | hazard_raw_rs2_stall_rf;

endmodule

`endif // __CPU_HAZARD__
