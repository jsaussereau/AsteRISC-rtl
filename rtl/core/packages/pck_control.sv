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

//! Control enums


`ifndef __PCK_CONTROL__
`define __PCK_CONTROL__


package pck_control;

  `define YES 1'd1
  `define NO  1'd0

  typedef enum logic [2:0] {
    instr_r,
    instr_i,
    instr_iu,   //i with unsigned imm
    instr_iucsr, //i with uimm instead of rs1
    instr_s,
    instr_b,
    instr_u,
    instr_j
  } instr_type_e;

  typedef enum logic [3:0] {
    br_none,
    br_bne,
    br_beq,
    br_blt,
    br_bge,
    br_bgeu,
    br_bltu,
    br_jal,
    br_jalr
  } sel_br_e;

  typedef enum logic [2:0] {
    pc_none,
    pc_plus_4,
    pc_alu,
    pc_imm,
    pc_rf
  } sel_pc_e;

  typedef enum logic [3:0] {
    alu_nop,
    alu_add,
    alu_sub,
    alu_and,
    alu_or,
    alu_xor,
    alu_slt,
    alu_sltu,
    alu_sll,
    alu_srl,
    alu_sra,
    alu_cpa,
    alu_cpb
  } sel_alu_op_e;

  typedef enum logic [1:0] {
    opa_none,
    opa_rf,
    opa_pc,
    opa_muldiv
  } sel_alu_opa_e;

  typedef enum logic [2:0] {
    opb_none,
    opb_rf,
    opb_imm,
    opb_muldiv,
    opb_csrr
  } sel_alu_opb_e;

  typedef enum logic [2:0] {
    muldiv_nop,
    muldiv_mull,
    muldiv_mulh,
    muldiv_div,
    muldiv_rem
  } sel_md_op_e;

  typedef enum logic [3:0] {
    wb_none,
    wb_alu,
    wb_muldiv,
    wb_pc,
    wb_dmem,
    wb_csr,
    wb_copro0,
    wb_copro1,
    wb_copro2
  } sel_wb_e;

  typedef enum logic [1:0] {
    csr_wr_none,
    csr_wr_rf,
    csr_wr_imm
  } sel_csr_wr_e;

  typedef enum logic [1:0] {
    csr_op_none,
    csr_op_set,
    csr_op_swap,
    csr_op_clear
  } sel_csr_op_e;

  typedef enum logic [1:0] {
    be_none,
    be_byte,
    be_half,
    be_word
  } sel_be_e;
/*
  typedef struct {
    sel_br_e      sel_br;         //! program counter mux selector
    sel_alu_op_e  sel_alu_op;     //! ALU operation selector
    sel_alu_opb_e sel_alu_opa;    //! ALU operator A selector
    sel_alu_opb_e sel_alu_opb;    //! ALU operator B selector
    sel_wb_e      sel_wb;         //! write back source selector
    logic         wen_rf;         //! register file write enable
    logic         wen_csr;        //! csr write enable
    logic         ren_csr;        //! csr read enable
  } ctrl_sigs_t;

  typedef struct {
    logic [31: 0] instr;          //! instruction output
    logic         br_eq;          //! ALU branch equal
    logic         br_lt;          //! ALU branch less than
    logic         br_ltu;         //! ALU branch less than (unsigned)
  } status_sigs_t;
*/
endpackage

`endif // __PCK_CONTROL__
