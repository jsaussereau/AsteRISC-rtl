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

//! RISC-V ISA

`ifndef __PCK_ISA__
`define __PCK_ISA__

package pck_isa;

  typedef struct packed {
    logic [31:25] funct7;
    logic [24:20] rs2;
    logic [19:15] rs1;
    logic [14:12] funct3;
    logic [11: 7] rd;
    logic [ 6: 0] opcode;
  } isa_common_t;

  typedef struct packed {
    logic [31:25] funct7;
    logic [24:20] rs2;
    logic [19:15] rs1;
    logic [14:12] funct3;
    logic [11: 7] rd;
    logic [ 6: 0] opcode;
  } isa_r_type_t;

  typedef struct packed {
    logic [31:20] imm;
    logic [19:15] rs1;
    logic [14:12] funct3;
    logic [11: 7] rd;
    logic [ 6: 0] opcode;
  } isa_i_type_t;

  typedef struct packed {
    logic [31:25] imm11_5;
    logic [24:20] rs2;
    logic [19:15] rs1;
    logic [14:12] funct3;
    logic [11: 7] imm4_0;
    logic [ 6: 0] opcode;
  } isa_s_type_t;

  typedef struct packed {
    logic         imm12;
    logic [30:25] imm10_5;
    logic [24:20] rs2;
    logic [19:15] rs1;
    logic [14:12] funct3;
    logic [11: 8] imm4_1;
    logic         imm11;
    logic [ 6: 0] opcode;
  } isa_b_type_t;

  typedef struct packed {
    logic [31:12] imm;
    logic [11: 7] rd;
    logic [ 6: 0] opcode;
  } isa_u_type_t;

  typedef struct packed {
    logic         imm20;
    logic [30:21] imm10_1;
    logic         imm11;
    logic [19:12] imm19_12;
    logic [11: 7] rd;
    logic [ 6: 0] opcode;
  } isa_j_type_t;

  typedef union packed {
    isa_r_type_t r;
    isa_i_type_t i;
    isa_s_type_t s;
    isa_b_type_t b;
    isa_u_type_t u;
    isa_j_type_t j;
    isa_common_t common;
    logic [31:0] code;
  } isa_instr_t;

  typedef enum { 
    // i baseline
    isa_nop,
    isa_add,
    isa_sub,
    isa_and,
    isa_or,
    isa_xor,
    isa_slt,
    isa_sltu,
    isa_sll,
    isa_srl,
    isa_sra,
    isa_addi,
    isa_andi,
    isa_ori,
    isa_xori,
    isa_slti,
    isa_sltiu,
    isa_slli,
    isa_srli,
    isa_srai,
    isa_lui,
    isa_auipc,
    isa_beq,
    isa_bne,
    isa_blt,
    isa_bge,
    isa_bltu,
    isa_bgeu,
    isa_jal,
    isa_jalr,
    isa_lw,
    isa_lb,
    isa_lh,
    isa_lbu,
    isa_lhu,
    isa_sw,
    isa_sb,
    isa_sh,

    // m extension
    isa_mul,
    isa_mulh,
    isa_mulsu,
    isa_mulu,
    isa_div,
    isa_divu,
    isa_rem,
    isa_remu,

    // zicsr extension
    isa_csrrw,
    isa_csrrs,
    isa_csrrc,
    isa_csrrwi,
    isa_csrrsi,
    isa_csrrci,

    // ziswap extension
    isa_swap_lw,
    isa_swap_lh,
    isa_swap_lhu,

    // custom extension
    isa_custom0,
    isa_custom1,
    isa_custom2,

    isa_other
  } isa_instr_e;

endpackage

`endif // __PCK_ISA__
