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

//! RISC-V C extension

// Warning: The order of the instructions matters when use in a 'casez'. 
//          The order used below should be used.
//          The reason for this is that the code of some instructions 
//          correspond to specific case of others instruction.
//          Examples: C.LUI with rd=x2 => C.ADDI16SP, C.MV with rs2=x0 => C.JR.

`ifndef __PCK_ISA_C__
`define __PCK_ISA_C__

package pck_isa_c;

  localparam C_NOP       = 16'b000_0_00000_00000_01;
  localparam C_ZERO      = 16'b000_0_00000_00000_00;
  localparam C_EBREAK    = 16'b1001_00000_00000_10;

  // Stack-Pointer-Based Loads and Stores
  localparam C_LWSP      = 16'b010_?_?????_?????_10;
  localparam C_SWSP      = 16'b110_??????_?????_10;

  // Register-Based Loads and Stores
  localparam C_LW        = 16'b010_???_???_??_???_00;
  localparam C_SW        = 16'b110_???_???_??_???_00;

  // Control Transfer Instructions
  localparam C_J         = 16'b101_???????????_01;
  localparam C_JAL       = 16'b001_???????????_01;
  localparam C_JR        = 16'b1000_?????_00000_10;
  localparam C_JALR      = 16'b1001_?????_00000_10;
  localparam C_BEQZ      = 16'b110_???_???_?????_01;
  localparam C_BNEZ      = 16'b111_???_???_?????_01;

  // Integer Register-Immediate Operations
  localparam C_ADDI      = 16'b000_?_?????_?????_01;
  localparam C_ADDI16SP  = 16'b011_?_00010_?????_01;
  localparam C_ADDI4SPN  = 16'b000_????????_???_00;
  //localparam C_SLLI      = 16'b000_0_?????_?????_10;  // cf. HINTs
  localparam C_SLLI      = 16'b000_?_?????_?????_10; 
  localparam C_SRLI      = 16'b100_?_00_???_?????_01;
  localparam C_SRAI      = 16'b100_?_01_???_?????_01;
  localparam C_ANDI      = 16'b100_?_10_???_?????_01;

   // Integer Constant-Generation Instructions
  localparam C_LI        = 16'b010_?_?????_?????_01;
  localparam C_LUI       = 16'b011_?_?????_?????_01;

  // Integer Register-Register Operations
  localparam C_MV        = 16'b1000_?????_?????_10;
  localparam C_ADD       = 16'b1001_?????_?????_10;
  localparam C_AND       = 16'b100011_???_11_???_01;
  localparam C_OR        = 16'b100011_???_10_???_01;
  localparam C_XOR       = 16'b100011_???_01_???_01;
  localparam C_SUB       = 16'b100011_???_00_???_01;


  // RV32FC only:
  localparam C_FLWSP     = 16'b011_?_?????_?????_10;
  localparam C_FSWSP     = 16'b111_?_?????_?????_10;
  localparam C_FLW       = 16'b011_???_???_??_???_00;
  localparam C_FSW       = 16'b111_???_???_??_???_00;

  // RV32DC only:
  localparam C_FLDSP     = 16'b001_?_?????_?????_10;
  localparam C_FSDSP     = 16'b101_?_?????_?????_10;
  localparam C_FLD       = 16'b001_???_???_??_???_00;
  localparam C_FSD       = 16'b101_???_???_??_???_00;

  localparam OPCODE_NC = 2'b11; // not compressed


  typedef struct packed {
    logic [15:12] funct4;
    logic [11: 7] rd_rs1;
    logic [ 6: 2] rs2;
    logic [ 1: 0] opcode;
  } isa_cr_type_t;

  typedef struct packed {
    logic [15:13] funct3;
    logic         imm5;
    logic [11: 7] rd_rs1;
    logic [ 6: 2] imm4_0;
    logic [ 1: 0] opcode;
  } isa_ci_type_t;

  typedef struct packed {
    logic [15:13] funct3;
    logic [12: 7] imm;
    logic [ 6: 2] rs2;
    logic [ 1: 0] opcode;
  } isa_css_type_t;

  typedef struct packed {
    logic [15:13] funct3;
    logic [12: 5] imm;
    logic [ 4: 2] rd_;
    logic [ 1: 0] opcode;
  } isa_ciw_type_t;

  typedef struct packed {
    logic [15:13] funct3;
    logic [12:10] imm4_2;
    logic [ 9: 7] rs1_;
    logic [ 6: 5] imm1_0;
    logic [ 4: 2] rd_;
    logic [ 1: 0] opcode;
  } isa_cl_type_t;

  typedef struct packed {
    logic [15:13] funct3;
    logic [12:10] imm4_2;
    logic [ 9: 7] rs1_;
    logic [ 6: 5] imm1_0;
    logic [ 4: 2] rs2_;
    logic [ 1: 0] opcode;
  } isa_cs_type_t;

  typedef struct packed {
    logic [15:10] funct6;
    logic [ 9: 7] rd_rs1_;
    logic [ 6: 5] funct2;
    logic [ 4: 2] rs2_;
    logic [ 1: 0] opcode;
  } isa_ca_type_t;

  typedef struct packed {
    logic [15:13] funct3;
    logic [12:10] imm7_5;
    logic [ 9: 7] rd_rs1_;
    logic [ 6: 2] imm4_0;
    logic [ 1: 0] opcode;
  } isa_cb_type_t;

  typedef struct packed {
    logic [15:13] funct3;
    logic         imm5;
    logic [11:10] funct2;
    logic [ 9: 7] rd_rs1_;
    logic [ 6: 2] imm4_0;
    logic [ 1: 0] opcode;
  } isa_cb2_type_t;

  typedef struct packed {
    logic [15:13] funct3;
    logic [12: 2] imm;
    logic [ 1: 0] opcode;
  } isa_cj_type_t;

  typedef union packed {
    isa_cr_type_t  cr;
    isa_ci_type_t  ci;
    isa_css_type_t css;
    isa_ciw_type_t ciw;
    isa_cl_type_t  cl;
    isa_cs_type_t  cs;
    isa_ca_type_t  ca;
    isa_cb_type_t  cb;
    isa_cb2_type_t cb2;
    isa_cj_type_t  cj;
    logic [15:0]   code;
  } isa_instr_c_t;

  typedef struct packed {
    logic [31:28] upper_funct4;
    logic [27:23] upper_rd_rs1;
    logic [22:18] upper_rs2;
    logic [17:16] upper_opcode;
    logic [15:12] lower_funct4;
    logic [11: 7] lower_rd_rs1;
    logic [ 6: 2] lower_rs2;
    logic [ 1: 0] lower_opcode;
  } isa_pcompr_t;

  typedef enum { 
    isa_c_nop,
    isa_c_zero,
    isa_c_ebreak,
    isa_c_lwsp,
    isa_c_swsp,
    isa_c_lw,
    isa_c_sw,
    isa_c_j,
    isa_c_jal,
    isa_c_jr,
    isa_c_jalr,
    isa_c_beqz,
    isa_c_bnez,
    isa_c_addi,
    isa_c_addi16sp,
    isa_c_addi4spn,
    isa_c_slli,
    isa_c_srli,
    isa_c_srai,
    isa_c_andi,
    isa_c_li,
    isa_c_lui,
    isa_c_mv,
    isa_c_add,
    isa_c_and,
    isa_c_or,
    isa_c_xor,
    isa_c_sub,
    isa_c_flwsp,
    isa_c_fswsp,
    isa_c_flw,
    isa_c_fsw,
    isa_c_fldsp,
    isa_c_fsdsp,
    isa_c_fld,
    isa_c_fsd,
    isa_c_other
  } isa_instr_c_e;

  function automatic logic[4:0] uncompress_reg_addr(input logic [2:0] compressed_reg_addr);
    uncompress_reg_addr = compressed_reg_addr + 8; // 3'b000 -> x8 (s0)
  endfunction

endpackage

`endif // __PCK_ISA_C__
