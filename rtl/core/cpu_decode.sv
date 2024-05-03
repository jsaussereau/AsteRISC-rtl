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

//! CPU decode

`ifndef __CPU_DECODE__
`define __CPU_DECODE__

//TODO: add rf_rd2_used : { = `YES pour alu_opb = opb_rf (reg-reg et sauts conditionnels) et memory store } 
// => 1 cycle en plus pour la lecture du 2eme registre avec 

`ifdef VIVADO
 `include "../soc/soc_config.sv"
 `include "packages/pck_control.sv"
 `include "packages/pck_isa.sv"
 `include "packages/pck_isa_i.sv"
 `include "packages/pck_isa_m.sv"
 `include "packages/pck_isa_custom.sv"
`else
 `include "soc/soc_config.sv"
 `include "core/packages/pck_control.sv"
 `include "core/packages/pck_isa.sv"
 `include "core/packages/pck_isa_i.sv"
 `include "core/packages/pck_isa_m.sv"
 `include "core/packages/pck_isa_custom.sv"
`endif
 

module cpu_decode
  import pck_isa::*;
  import pck_isa_i::*;
  import pck_isa_m::*;
  import pck_isa_ziswap::*;
  import pck_isa_custom::*;
  import pck_control::*;
#(
  parameter p_ext_rvm      = 0,
  parameter p_ext_rvzicsr  = 0,
  parameter p_ext_rvziswap = 1,
  parameter p_ext_custom   = 0,
  parameter p_decode_buf   = 0
)( 
  input  wire          i_clk,             //! global clock
  input  wire          i_en_decode,       //! enable decode
  input  isa_instr_t   i_instr,           //! instruction from 'fetch' stage
  output isa_instr_e   o_instr_name,      //! instruction name (debug)
  output logic [ 4: 0] o_rf_wr_addr,      //! register file write address (rd)
  output logic [ 4: 0] o_rf_rd1_addr,     //! register file read address 1 (rs1)
  output logic [ 4: 0] o_rf_rd2_addr,     //! register file read address 2 (rs2)
  output logic         o_rf_rd2_used,     //! register file read port 2 is used
  output logic [11: 0] o_csr_addr,        //! 
  output logic [31: 0] o_imm,             //! immediate value
  output sel_br_e      o_sel_br,          //! program counter mux selector
  output sel_alu_op_e  o_sel_alu_op,      //! ALU operation selector
  output sel_alu_opa_e o_sel_alu_opa,     //! ALU operator A selector
  output sel_alu_opb_e o_sel_alu_opb,     //! ALU operator B selector
  output sel_md_op_e   o_sel_md_op,       //! MULDIV operation selector
  output sel_csr_wr_e  o_sel_csr_wr,      //! csr write data selector
  output sel_csr_op_e  o_sel_csr_op,      //! csr write operation selector
  output logic         o_opa_signed,      //! MULDIV operand A is signed
  output logic         o_opb_signed,      //! MULDIV operand B is signed
  output sel_wb_e      o_sel_wb,          //! write back source selector
  output sel_be_e      o_sel_dmem_be,     //! dmem address mode (word, byte, halfbyte)
  output logic         o_dmem_sext,       //! sign extension on non-word dmem data
  output logic         o_dmem_wr,         //! data memory write
  output logic         o_dmem_rd,         //! data memory read
  output logic         o_swap_bytes,      //! swap bytes (big endian -> little endian)
  output logic         o_en_wb,           //! register file write enable
  output logic         o_wen_csr,         //! csr write enable
  output logic         o_ren_csr,         //! csr read enable
  output logic         o_cond_branch,     //! conditionnal branch (inconditionnal branches jal and jalr are excluded)
  output logic         o_cond_br_bp,      //! conditionnal branch (inconditionnal branches jal and jalr are excluded) (unregistered)
  output logic         o_br_instr_bp,     //! branch instruction (unregistered)
  output logic         o_jalr_instr_bp,   //! jalr instruction (unregistered)
  output logic [31: 0] o_imm_bp,          //! immediate value (unregistered)
  output logic         o_jump_reg         //! jump register
);

  instr_type_e  instr_type;
  isa_instr_e   instr_name;

  logic [31: 0] imm;
  sel_br_e      sel_br;
  sel_alu_op_e  sel_alu_op;
  sel_alu_opa_e sel_alu_opa;
  sel_alu_opb_e sel_alu_opb; 
  sel_wb_e      sel_wb;
  sel_be_e      sel_dmem_be;
  sel_csr_wr_e  sel_csr_wr;
  sel_csr_op_e  sel_csr_op;
  sel_md_op_e   sel_md_op;
  logic         rf_rd2_used;
  logic         opa_signed;
  logic         opb_signed;
  logic         dmem_sext;
  logic         dmem_wr;
  logic         dmem_rd;
  logic         swap_bytes;
  logic         en_wb;
  logic         wen_csr;
  logic         ren_csr;
  logic         branch_instr;
  logic         jalr_instr;
  logic         cond_branch;
  logic         jump_reg;

  logic         rvi_instr;
  logic         rvm_instr;
  logic         rvzicsr_instr;
  logic         rvziswap_instr;
  logic         custom_instr;
  logic         illegal;

  always_comb begin
  //always_ff @(posedge i_clk) begin
    // default values:
    if (i_en_decode) begin //TODO: find the best place to do it
      
    instr_name       = isa_other;
    instr_type       = instr_r;
    sel_br           = br_none;
    sel_alu_op       = alu_nop;
    sel_md_op        = muldiv_nop;
    sel_alu_opa      = opa_none;
    sel_alu_opb      = opb_none;
    sel_wb           = wb_none;
    sel_csr_wr       = csr_wr_none;
    sel_csr_op       = csr_op_none;
    sel_dmem_be      = be_none;
    rf_rd2_used      = `NO;
    dmem_sext        = `NO;
    dmem_wr          = `NO;
    dmem_rd          = `NO;
    wen_csr          = `NO;
    ren_csr          = `NO;
    opa_signed       = `NO;
    opb_signed       = `NO;
    
    rvi_instr        = `YES;
    casez (i_instr.code)
      pck_isa_i::NOP     : begin 
        instr_name       = isa_nop;
        instr_type       = instr_r;
        sel_alu_op       = alu_nop;
        sel_alu_opa      = opa_none;
        sel_alu_opb      = opb_none;
        sel_wb           = wb_none;
      end

      // reg-reg
      pck_isa_i::ADD     :  begin 
        instr_name       = isa_add;
        instr_type       = instr_r;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
        sel_wb           = wb_alu;
      end
      pck_isa_i::SUB     : begin
        instr_name       = isa_sub;
        instr_type       = instr_r;
        sel_alu_op       = alu_sub;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
        sel_wb           = wb_alu;
      end
      pck_isa_i::AND     : begin
        instr_name       = isa_and;
        instr_type       = instr_r;
        sel_alu_op       = alu_and;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
        sel_wb           = wb_alu;
      end
      pck_isa_i::OR      : begin
        instr_name       = isa_or;
        instr_type       = instr_r;
        sel_alu_op       = alu_or;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
        sel_wb           = wb_alu;
      end
      pck_isa_i::XOR     : begin
        instr_name       = isa_xor;
        instr_type       = instr_r;
        sel_alu_op       = alu_xor;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
        sel_wb           = wb_alu;
      end
      pck_isa_i::SLT     : begin
        instr_name       = isa_slt;
        instr_type       = instr_r;
        sel_alu_op       = alu_slt;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
        sel_wb           = wb_alu;
      end
      pck_isa_i::SLTU    : begin
        instr_name       = isa_sltu;
        instr_type       = instr_r;
        sel_alu_op       = alu_sltu;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
        sel_wb           = wb_alu;
      end
      pck_isa_i::SLL     : begin
        instr_name       = isa_sll;
        instr_type       = instr_r;
        sel_alu_op       = alu_sll;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
        sel_wb           = wb_alu;
      end
      pck_isa_i::SRL     : begin
        instr_name       = isa_srl;
        instr_type       = instr_r;
        sel_alu_op       = alu_srl;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
        sel_wb           = wb_alu;
      end
      pck_isa_i::SRA     : begin
        instr_name       = isa_sra;
        instr_type       = instr_r;
        sel_alu_op       = alu_sra;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
        sel_wb           = wb_alu;
      end

      // reg-imm
      pck_isa_i::ADDI    : begin
        instr_name       = isa_addi;
        instr_type       = instr_i;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_alu;
      end
      pck_isa_i::ANDI    : begin
        instr_name       = isa_andi;
        instr_type       = instr_i;
        sel_alu_op       = alu_and;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_alu;
      end
      pck_isa_i::ORI     : begin
        instr_name       = isa_ori;
        instr_type       = instr_i;
        sel_alu_op       = alu_or;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_alu;
      end
      pck_isa_i::XORI    : begin
        instr_name       = isa_xori;
        instr_type       = instr_i;
        sel_alu_op       = alu_xor;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_alu;
      end
      pck_isa_i::SLTI    : begin
        instr_name       = isa_slti;
        instr_type       = instr_i;
        sel_alu_op       = alu_slt;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_alu;
      end
      pck_isa_i::SLTIU   : begin
        instr_name       = isa_sltiu;
        instr_type       = instr_iu;
        sel_alu_op       = alu_sltu;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_alu;
      end
      pck_isa_i::SLLI    : begin
        instr_name       = isa_slli;
        instr_type       = instr_i;
        sel_alu_op       = alu_sll;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_alu;
      end
      pck_isa_i::SRLI    : begin
        instr_name       = isa_srli;
        instr_type       = instr_i;
        sel_alu_op       = alu_srl;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_alu;
      end
      pck_isa_i::SRAI    : begin
        instr_name       = isa_srai;
        instr_type       = instr_i;
        sel_alu_op       = alu_sra;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_alu;
      end

      // upper-immediate 
      pck_isa_i::LUI     : begin
        instr_name       = isa_lui;
        instr_type       = instr_u;
        sel_alu_op       = alu_cpb;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_alu;
      end
      pck_isa_i::AUIPC   : begin
        instr_name       = isa_auipc;
        instr_type       = instr_u;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_pc;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_alu;
      end

      // branch and jump 
      pck_isa_i::BEQ     : begin
        instr_name       = isa_beq;
        instr_type       = instr_b;
        sel_br           = br_beq;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
      end
      pck_isa_i::BNE     : begin
        instr_name       = isa_bne;
        instr_type       = instr_b;
        sel_br           = br_bne;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
      end
      pck_isa_i::BLT     : begin
        instr_name       = isa_blt;
        instr_type       = instr_b;
        sel_br           = br_blt;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
      end
      pck_isa_i::BGE     : begin
        instr_name       = isa_bge;
        instr_type       = instr_b;
        sel_br           = br_bge;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
      end
      pck_isa_i::BLTU    : begin
        instr_name       = isa_bltu;
        instr_type       = instr_b;
        sel_br           = br_bltu;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
      end
      pck_isa_i::BGEU    : begin
        instr_name       = isa_bgeu;
        instr_type       = instr_b;
        sel_br           = br_bgeu;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_rf;
        rf_rd2_used      = `YES;
      end
      pck_isa_i::JAL     : begin
        instr_name       = isa_jal;
        instr_type       = instr_j;
        sel_br           = br_jal;
        sel_alu_op       = alu_nop;
        sel_alu_opa      = opa_none;
        sel_alu_opb      = opb_none;
        sel_wb           = wb_pc;
      end
      pck_isa_i::JALR    : begin
        instr_name       = isa_jalr;
        instr_type       = instr_i;
        sel_br           = br_jalr;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        sel_wb           = wb_pc;
      end

      // memory load and store
      pck_isa_i::LW      : begin
        instr_name       = isa_lw;
        instr_type       = instr_i;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        rf_rd2_used      = `YES;
        sel_wb           = wb_dmem;
        sel_dmem_be      = be_word;
        dmem_sext        = `YES;
        dmem_wr          = `NO;
        dmem_rd          = `YES;
      end
      pck_isa_i::LB      : begin
        instr_name       = isa_lb;
        instr_type       = instr_i;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        rf_rd2_used      = `YES;
        sel_wb           = wb_dmem;
        sel_dmem_be      = be_byte;
        dmem_sext        = `YES;
        dmem_wr          = `NO;
        dmem_rd          = `YES;
      end
      pck_isa_i::LH      : begin
        instr_name       = isa_lh;
        instr_type       = instr_i;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        rf_rd2_used      = `YES;
        sel_wb           = wb_dmem;
        sel_dmem_be      = be_half;
        dmem_sext        = `YES;
        dmem_wr          = `NO;
        dmem_rd          = `YES;
      end
      pck_isa_i::LBU     : begin
        instr_name       = isa_lbu;
        instr_type       = instr_i;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        rf_rd2_used      = `YES;
        sel_wb           = wb_dmem;
        sel_dmem_be      = be_byte;
        dmem_sext        = `NO;
        dmem_wr          = `NO;
        dmem_rd          = `YES;
      end
      pck_isa_i::LHU     : begin
        instr_name       = isa_lhu;
        instr_type       = instr_i;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        rf_rd2_used      = `YES;
        sel_wb           = wb_dmem;
        sel_dmem_be      = be_half;
        dmem_sext        = `NO;
        dmem_wr          = `NO;
        dmem_rd          = `YES;
      end
      pck_isa_i::SW      : begin
        instr_name       = isa_sw;
        instr_type       = instr_s;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        rf_rd2_used      = `YES;
        sel_wb           = wb_none;
        sel_dmem_be      = be_word;
        dmem_sext        = `YES;
        dmem_wr          = `YES;
        dmem_rd          = `NO;
      end
      pck_isa_i::SB      : begin
        instr_name       = isa_sb;
        instr_type       = instr_s;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        rf_rd2_used      = `YES;
        sel_wb           = wb_none;
        sel_dmem_be      = be_byte;
        dmem_sext        = `YES;
        dmem_wr          = `YES;
        dmem_rd          = `NO;
      end
      pck_isa_i::SH      : begin
        instr_name       = isa_sh;
        instr_type       = instr_s;
        sel_alu_op       = alu_add;
        sel_alu_opa      = opa_rf;
        sel_alu_opb      = opb_imm;
        rf_rd2_used      = `YES;
        sel_wb           = wb_none;
        sel_dmem_be      = be_half;
        dmem_sext        = `YES;
        dmem_wr          = `YES;
        dmem_rd          = `NO;
      end

      default            : begin
        rvi_instr        = `NO;
      end
    endcase

    // RV32M Extension (multiplication and division)
    if (p_ext_rvm) begin
      rvm_instr = `YES;
      casez (i_instr.code)
        pck_isa_m::MUL     : begin
          instr_name       = isa_mul;
          instr_type       = instr_r;
          sel_alu_op       = alu_add;
          sel_md_op        = muldiv_mull;
          opa_signed       = `YES;
          opb_signed       = `YES;
          rf_rd2_used      = `YES;
          sel_wb           = wb_muldiv;
        end
        pck_isa_m::MULH    : begin
          instr_name       = isa_mulh;
          instr_type       = instr_r;
          sel_alu_op       = alu_add;
          sel_md_op        = muldiv_mulh;
          opa_signed       = `YES;
          opb_signed       = `YES;
          rf_rd2_used      = `YES;
          sel_wb           = wb_muldiv;
        end
        pck_isa_m::MULSU   : begin
          instr_name       = isa_mulsu;
          instr_type       = instr_r;
          sel_alu_op       = alu_add;
          sel_md_op        = muldiv_mull;
          opa_signed       = `YES;
          opb_signed       = `NO;
          rf_rd2_used      = `YES;
          sel_wb           = wb_muldiv;
        end
        pck_isa_m::MULU    : begin
          instr_name       = isa_mulu;
          instr_type       = instr_r;
          sel_alu_op       = alu_add;
          sel_md_op        = muldiv_mull;
          opa_signed       = `NO;
          opb_signed       = `NO;
          rf_rd2_used      = `YES;
          sel_wb           = wb_muldiv;
        end
        pck_isa_m::DIV     : begin
          instr_name       = isa_div;
          instr_type       = instr_r;
          sel_alu_op       = alu_add;
          sel_md_op        = muldiv_div;
          opa_signed       = `YES;
          opb_signed       = `YES;
          rf_rd2_used      = `YES;
          sel_wb           = wb_muldiv;
        end
        pck_isa_m::DIVU    : begin
          instr_name       = isa_divu;
          instr_type       = instr_r;
          sel_alu_op       = alu_add;
          sel_md_op        = muldiv_div;
          opa_signed       = `NO;
          opb_signed       = `NO;
          rf_rd2_used      = `YES;
          sel_wb           = wb_muldiv;
        end
        pck_isa_m::REM     : begin
          instr_name       = isa_rem;
          instr_type       = instr_r;
          sel_alu_op       = alu_add;
          sel_md_op        = muldiv_rem;
          opa_signed       = `YES;
          opb_signed       = `YES;
          rf_rd2_used      = `YES;
          sel_wb           = wb_muldiv;
        end
        pck_isa_m::REMU    : begin
          instr_name       = isa_remu;
          instr_type       = instr_r;
          sel_alu_op       = alu_add;
          sel_md_op        = muldiv_rem;
          opa_signed       = `NO;
          opb_signed       = `NO;
          rf_rd2_used      = `YES;
          sel_wb           = wb_muldiv;
        end
        default            : begin
          rvm_instr        = `NO;
        end
      endcase
    end else begin
      rvm_instr = `NO;
    end

    // Ziscr Extension
    rvzicsr_instr = `YES;
    if (p_ext_rvzicsr) begin
      casez (i_instr.code)
        pck_isa_zicsr::CSRRW : begin
          instr_name       = isa_csrrw;
          instr_type       = instr_iucsr;
          sel_csr_op       = csr_op_swap;
          sel_csr_wr       = csr_wr_rf;
          wen_csr          = `YES;
          sel_wb           = (i_instr.i.rd == 5'd0) ? wb_none : wb_csr;
        end
        pck_isa_zicsr::CSRRS : begin
          instr_name       = isa_csrrs;
          instr_type       = instr_iucsr;
          sel_csr_op       = csr_op_set;
          sel_csr_wr       = csr_wr_rf;
          wen_csr          = (i_instr.i.rs1 == 5'd0) ? `NO : `YES;
          sel_wb           = wb_csr;
        end
        pck_isa_zicsr::CSRRC : begin
          instr_name       = isa_csrrc;
          instr_type       = instr_iucsr;
          sel_csr_op       = csr_op_clear;
          sel_csr_wr       = csr_wr_rf;
          wen_csr          = (i_instr.i.rs1 == 5'd0) ? `NO : `YES;
          sel_wb           = wb_csr;
        end
        pck_isa_zicsr::CSRRWI : begin
          instr_name       = isa_csrrwi;
          instr_type       = instr_iucsr;
          sel_csr_op       = csr_op_swap;
          sel_csr_wr       = csr_wr_imm;
          wen_csr          = `YES;
          sel_wb           = (i_instr.i.rd == 5'd0) ? wb_none : wb_csr;
        end
        pck_isa_zicsr::CSRRSI : begin
          instr_name       = isa_csrrsi;
          instr_type       = instr_iucsr;
          sel_csr_op       = csr_op_set;
          sel_csr_wr       = csr_wr_imm;
          wen_csr          = (i_instr.i.rs1 == 5'd0) ? `NO : `YES;
          sel_wb           = wb_csr;
        end
        pck_isa_zicsr::CSRRCI : begin
          instr_name       = isa_csrrci;
          instr_type       = instr_iucsr;
          sel_csr_op       = csr_op_clear;
          sel_csr_wr       = csr_wr_imm;
          wen_csr          = (i_instr.i.rs1 == 5'd0) ? `NO : `YES;
          sel_wb           = wb_csr;
        end
        default            : begin
          rvzicsr_instr    = `NO;
        end
      endcase
    end else begin
      rvzicsr_instr = `NO;
    end

    // Custom instructions extension
    if (p_ext_rvziswap) begin
      rvziswap_instr = `YES;
      swap_bytes     = `YES;
      casez (i_instr.code)
        pck_isa_ziswap::SWAP_LW : begin
          instr_name       = isa_swap_lw;
          instr_type       = instr_i;
          sel_alu_op       = alu_add;
          sel_alu_opa      = opa_rf;
          sel_alu_opb      = opb_imm;
          rf_rd2_used      = `YES;
          sel_wb           = wb_dmem;
          sel_dmem_be      = be_word;
          dmem_sext        = `YES;
          dmem_wr          = `NO;
          dmem_rd          = `YES;
        end
        pck_isa_ziswap::SWAP_LH : begin
          instr_name       = isa_swap_lh;
          instr_type       = instr_i;
          sel_alu_op       = alu_add;
          sel_alu_opa      = opa_rf;
          sel_alu_opb      = opb_imm;
          rf_rd2_used      = `YES;
          sel_wb           = wb_dmem;
          sel_dmem_be      = be_half;
          dmem_sext        = `YES;
          dmem_wr          = `NO;
          dmem_rd          = `YES;
        end
        pck_isa_ziswap::SWAP_LHU : begin
          instr_name       = isa_swap_lhu;
          instr_type       = instr_i;
          sel_alu_op       = alu_add;
          sel_alu_opa      = opa_rf;
          sel_alu_opb      = opb_imm;
          rf_rd2_used      = `YES;
          sel_wb           = wb_dmem;
          sel_dmem_be      = be_half;
          dmem_sext        = `NO;
          dmem_wr          = `NO;
          dmem_rd          = `YES;
        end
        default            : begin
          rvziswap_instr   = `NO;
          swap_bytes       = `NO;
        end
      endcase
    end else begin
      rvziswap_instr = `NO;
      swap_bytes     = `NO;
    end

    // Custom instructions extension
    if (p_ext_custom) begin
      custom_instr = `YES;
      casez (i_instr.code)
        pck_isa_custom::CUSTOM0: begin // crc6
          instr_name       = isa_custom0;
          instr_type       = instr_r;
          sel_alu_op       = alu_nop;
          sel_alu_opa      = opa_rf;
          sel_alu_opb      = opb_rf;
          rf_rd2_used      = `YES;
          sel_wb           = wb_copro0;
        end
        /*pck_isa_custom::CUSTOM1: begin // sb_crc6
          instr_name       = isa_custom1;
          instr_type       = instr_r;
          sel_alu_op       = alu_add;
          sel_alu_opa      = opa_rf;
          sel_alu_opb      = opb_none;
          rf_rd2_used      = `YES;
          sel_wb           = wb_copro0;
          sel_dmem_be      = be_byte;
          dmem_sext        = `YES;
          dmem_wr          = `YES;
          dmem_rd          = `NO;
        end*/
        default            : begin
          custom_instr     = `NO;
        end
      endcase
    end else begin
      custom_instr = `NO;
    end
    
    end
  end

  assign illegal = ~rvi_instr & ~rvm_instr & ~rvzicsr_instr & ~rvziswap_instr & ~custom_instr;

  assign en_wb = (sel_wb != wb_none) ? 1'b1 : 1'b0;
  assign cond_branch = (sel_br == br_bne || sel_br == br_beq || sel_br == br_blt || sel_br == br_bge || sel_br == br_bgeu || sel_br == br_bltu) ? 1'b1 : 1'b0;
  assign branch_instr = (cond_branch || sel_br == br_jal || sel_br == br_jalr) ? 1'b1 : 1'b0;
  assign jalr_instr = (sel_br == br_jalr) ? 1'b1 : 1'b0;
  assign jump_reg = jalr_instr;

  //! Unpack instruction
generate
  if (p_decode_buf) begin
    always_ff @(posedge i_clk) begin: unpack
      //if (i_en_decode) begin // registers
        o_rf_wr_addr  <= i_instr.common.rd;
        o_rf_rd1_addr <= i_instr.common.rs1;
        o_rf_rd2_addr <= i_instr.common.rs2;
        o_csr_addr    <= i_instr.i.imm;

        o_sel_br      <= sel_br;
        o_sel_alu_op  <= sel_alu_op;
        o_sel_alu_opa <= sel_alu_opa;
        o_sel_alu_opb <= sel_alu_opb; 
        o_sel_wb      <= sel_wb;
        o_sel_dmem_be <= sel_dmem_be;
        o_sel_md_op   <= sel_md_op;
        o_sel_csr_wr  <= sel_csr_wr;
        o_sel_csr_op  <= sel_csr_op;
        o_opa_signed  <= opa_signed;
        o_opb_signed  <= opb_signed;
        o_rf_rd2_used <= rf_rd2_used;
        o_dmem_sext   <= dmem_sext;
        o_dmem_wr     <= dmem_wr;
        o_dmem_rd     <= dmem_rd;
        o_swap_bytes  <= swap_bytes;
        o_en_wb       <= en_wb;
        o_wen_csr     <= wen_csr;
        o_ren_csr     <= ren_csr;
        o_imm         <= imm;
        o_cond_branch <= cond_branch;
        o_jump_reg    <= jump_reg;
      //end 
    end 
  end else begin
    always_comb begin: unpack
      //if (i_en_decode) begin // latches
        o_rf_wr_addr  = i_instr.common.rd;
        o_rf_rd1_addr = i_instr.common.rs1;
        o_rf_rd2_addr = i_instr.common.rs2;
        o_csr_addr    = i_instr.i.imm;

        o_sel_br      = sel_br;
        o_sel_alu_op  = sel_alu_op;
        o_sel_alu_opa = sel_alu_opa;
        o_sel_alu_opb = sel_alu_opb; 
        o_sel_wb      = sel_wb;
        o_sel_dmem_be = sel_dmem_be;
        o_sel_md_op   = sel_md_op;
        o_sel_csr_wr  = sel_csr_wr;
        o_sel_csr_op  = sel_csr_op;
        o_opa_signed  = opa_signed;
        o_opb_signed  = opb_signed;
        o_rf_rd2_used = rf_rd2_used;
        o_dmem_sext   = dmem_sext;
        o_dmem_wr     = dmem_wr;
        o_dmem_rd     = dmem_rd;
        o_swap_bytes  = swap_bytes;
        o_en_wb       = en_wb;
        o_wen_csr     = wen_csr;
        o_ren_csr     = ren_csr;
        o_imm         = imm;
        o_cond_branch = cond_branch;
        o_jump_reg    = jump_reg;
      end
    //end 
  end
endgenerate

  //! IMM GEN: immediate value generator
  cpu_imm_gen #(
    .p_branch_imm ( 0          )
  ) imm_gen (
    .i_instr      ( i_instr    ),
    .i_instr_type ( instr_type ),
    .o_imm        ( imm        )
  );

  assign o_instr_name = instr_name;

  // branch prediction outputs
  assign o_br_instr_bp   = branch_instr;
  assign o_cond_br_bp    = cond_branch;
  assign o_jalr_instr_bp = jalr_instr;
  assign o_imm_bp        = imm;

endmodule

`endif // __CPU_DECODE__
