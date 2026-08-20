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

//! Pipeline payload bundles
//!
//! The pipeline of `cpu_core_pipe` used to repeat the same ~35-signal list four
//! times per stage (reset values, squash values, enable transfer, combinational
//! feed-through when the stage barrier is disabled). Grouping the payload into
//! packed structs collapses each of those lists to a handful of lines, and the
//! `*_NOP` constants below replace the reset and squash lists entirely: adding a
//! new datapath signal now means adding one struct field, not editing 28 places.
//!
//! The bundles are split into sub-groups that match the actual lifetime of each
//! signal, so a stage only carries the groups it needs and no extra flip-flop is
//! inferred compared to the previous flat coding.

`ifndef __PCK_PIPE__
`define __PCK_PIPE__

`ifdef VIVADO
  `include "packages/pck_control.sv"
  `include "packages/pck_isa.sv"
  `include "packages/pck_isa_i.sv"
`else
  `include "core/packages/pck_control.sv"
  `include "core/packages/pck_isa.sv"
  `include "core/packages/pck_isa_i.sv"
`endif

package pck_pipe;

  import pck_control::*;
  import pck_isa::*;

  /*****************************************************
                      Payload groups
  *****************************************************/

  //! instruction identity and immediate: carried by every stage
  typedef struct packed {
    logic [31: 0] pc;                     //! program counter
    logic [31: 0] pc_inc;                 //! program counter +4 / +2
    logic [31: 0] imm;                    //! immediate value
    isa_instr_e   instr_name;             //! instruction name (debug)
    isa_instr_t   instr_decomp;           //! decompressed instruction
  } pipe_com_t;

  //! source register addresses: gated by a dedicated enable in the ID stage
  //! (`en_RF_addr`), hence kept in a group of their own
  typedef struct packed {
    logic [ 4: 0] rd1_addr;               //! regfile read address for port 1
    logic [ 4: 0] rd2_addr;               //! regfile read address for port 2
    logic         rd2_used;               //! regfile read port 2 is used
  } pipe_rsa_t;

  //! source register information: carried from ID down to WB (hazard + debug)
  typedef struct packed {
    pipe_rsa_t    a;                      //! read addresses (separately enabled)
    logic         stall_rs1;              //! a stall is needed on rs1
    logic         stall_rs2;              //! a stall is needed on rs2
  } pipe_rs_t;

  //! write back control: carried from ID down to MA
  typedef struct packed {
    logic [ 4: 0] wb_addr;                //! write back address
    sel_wb_e      sel_wb;                 //! write back source selector
    logic         en_wb;                  //! register file write enable
  } pipe_wbc_t;

  //! decode control consumed by EX: carried from ID down to RF only
  typedef struct packed {
    sel_alu_op_e  sel_alu_op;             //! ALU operation selector
    sel_alu_opa_e sel_alu_opa;            //! ALU operand A selector
    sel_alu_opb_e sel_alu_opb;            //! ALU operand B selector
    sel_md_op_e   sel_md_op;              //! MULDIV operation selector
    sel_csr_wr_e  sel_csr_wr;             //! csr write data selector
    sel_csr_op_e  sel_csr_op;             //! csr write operation selector
    logic         opa_signed;             //! MULDIV operand A is signed
    logic         opb_signed;             //! MULDIV operand B is signed
    logic [11: 0] csr_addr;               //! csr address
    logic         wen_csr;                //! csr write enable
    logic         ren_csr;                //! csr read enable
  } pipe_dec_t;

  //! data memory control: carried from ID down to EX only
  typedef struct packed {
    sel_be_e      sel_dmem_be;            //! dmem address mode (word, byte, halfbyte)
    logic         dmem_sext;              //! sign extension on non-word dmem data
    logic         en_dmem_wr;             //! data memory write enable
    logic         en_dmem_rd;             //! data memory read enable
    logic         swap_bytes;             //! swap byte order
  } pipe_mem_t;

  //! forwarding decisions: carried from EX down to WB (debug/log only)
  typedef struct packed {
    logic         rs1_ex;                 //! rs1 forwarded from EX
    logic         rs1_ma;                 //! rs1 forwarded from MA
    logic         rs1_wb;                 //! rs1 forwarded from WB
    logic         rs2_ex;                 //! rs2 forwarded from EX
    logic         rs2_ma;                 //! rs2 forwarded from MA
    logic         rs2_wb;                 //! rs2 forwarded from WB
  } pipe_bp_t;

  //! datapath values produced by RF/EX: carried from EX down to MA
  typedef struct packed {
    logic [31: 0] rd1_data;               //! regfile read data for port 1
    logic [31: 0] rd2_data;               //! regfile read data for port 2
    logic [31: 0] alu_out;                //! ALU output
    logic [31: 0] adder_out;              //! adder output
    logic [31: 0] muldiv_out;             //! MUL/DIV result
    logic [31: 0] csr_rd_data;            //! read data from csr
    logic [31: 0] copro_out0;             //! coprocessor 0 result
    logic [31: 0] copro_out1;             //! coprocessor 1 result
    logic [31: 0] copro_out2;             //! coprocessor 2 result
    logic         exec_done;              //! execute stage done
    logic         branch_taken;           //! a branch is taken
    sel_pc_e      sel_pc;                 //! program counter select
  } pipe_dat_t;

  /*****************************************************
                      Stage bundles
  *****************************************************/

  //! payload of the ID and RF stage barriers
  typedef struct packed {
    pipe_com_t    com;
    pipe_rs_t     rs;
    pipe_wbc_t    wbc;
    pipe_dec_t    dec;
    pipe_mem_t    mem;
    sel_br_e      sel_br;                 //! program counter mux selector
    logic         cond_branch;            //! conditionnal branch (ID only)
    logic         jump_reg;               //! jump register (ID only)
  } pipe_id_t;

  //! payload of the EX stage barrier
  typedef struct packed {
    pipe_com_t    com;
    pipe_rs_t     rs;
    pipe_wbc_t    wbc;
    pipe_mem_t    mem;
    pipe_dat_t    dat;
    pipe_bp_t     bp;
    sel_br_e      sel_br;                 //! program counter mux selector
  } pipe_ex_t;

  //! payload of the MA stage barrier (dmem control is consumed by then)
  typedef struct packed {
    pipe_com_t    com;
    pipe_rs_t     rs;
    pipe_wbc_t    wbc;
    pipe_dat_t    dat;
    pipe_bp_t     bp;
    sel_br_e      sel_br;                 //! program counter mux selector
  } pipe_ma_t;

  //! payload of the WB stage barrier
  //! WB does not simply forward the MA payload: it also carries the register
  //! file write it is about to perform, so the fields it needs are listed
  //! explicitly rather than reusing `pipe_dat_t`.
  typedef struct packed {
    pipe_com_t    com;
    pipe_rs_t     rs;
    pipe_bp_t     bp;
    logic [31: 0] imm_raw;                //! immediate without branch bypass (debug)
    logic [31: 0] rd1_data_raw;           //! rs1 data without branch bypass (debug)
    logic [31: 0] rd1_data;               //! regfile read data for port 1
    logic [31: 0] rd2_data;               //! regfile read data for port 2
    logic [31: 0] alu_out;                //! ALU output (used by jalr)
    logic         branch_taken;           //! a branch is taken
    sel_wb_e      sel_wb;                 //! write back source selector
    logic [ 4: 0] rf_wr_addr;             //! regfile write address
    logic [31: 0] rf_wr_data;             //! regfile write data
    logic         rf_wr_en;               //! regfile write enable
  } pipe_wb_t;

  /*****************************************************
                     Bubble constants
  *****************************************************/
  //
  // A pipeline bubble. Enum fields must be named explicitly: an implicit
  // integer-to-enum conversion through `default: '0` is rejected by strict
  // elaborators (Verilator, Design Compiler, Genus). Plain `logic` fields are
  // covered by `default: '0`, so adding one needs no change here.

  localparam pipe_com_t PIPE_COM_NOP = '{
    instr_name:   isa_nop,
    instr_decomp: pck_isa_i::NOP,
    default: '0
  };

  localparam pipe_rs_t  PIPE_RS_NOP  = '{ default: '0 };

  localparam pipe_wbc_t PIPE_WBC_NOP = '{ sel_wb: wb_none, default: '0 };

  localparam pipe_dec_t PIPE_DEC_NOP = '{
    sel_alu_op:   alu_nop,
    sel_alu_opa:  opa_none,
    sel_alu_opb:  opb_none,
    sel_md_op:    muldiv_nop,
    sel_csr_wr:   csr_wr_none,
    sel_csr_op:   csr_op_none,
    default: '0
  };

  localparam pipe_mem_t PIPE_MEM_NOP = '{ sel_dmem_be: be_none, default: '0 };

  localparam pipe_bp_t  PIPE_BP_NOP  = '{ default: '0 };

  //! `sel_pc` is the one field whose reset value (`pc_plus_4`) differs from its
  //! squash value (`pc_none`), hence the two flavours below.
  localparam pipe_dat_t PIPE_DAT_NOP = '{ sel_pc: pc_none,   default: '0 };
  localparam pipe_dat_t PIPE_DAT_RST = '{ sel_pc: pc_plus_4, default: '0 };

  localparam pipe_id_t  PIPE_ID_NOP  = '{
    com: PIPE_COM_NOP, rs: PIPE_RS_NOP, wbc: PIPE_WBC_NOP,
    dec: PIPE_DEC_NOP, mem: PIPE_MEM_NOP,
    sel_br: br_none, cond_branch: 1'b0, jump_reg: 1'b0
  };

  localparam pipe_ex_t  PIPE_EX_NOP  = '{
    com: PIPE_COM_NOP, rs: PIPE_RS_NOP, wbc: PIPE_WBC_NOP,
    mem: PIPE_MEM_NOP, dat: PIPE_DAT_NOP, bp: PIPE_BP_NOP,
    sel_br: br_none
  };
  localparam pipe_ex_t  PIPE_EX_RST  = '{
    com: PIPE_COM_NOP, rs: PIPE_RS_NOP, wbc: PIPE_WBC_NOP,
    mem: PIPE_MEM_NOP, dat: PIPE_DAT_RST, bp: PIPE_BP_NOP,
    sel_br: br_none
  };

  localparam pipe_ma_t  PIPE_MA_NOP  = '{
    com: PIPE_COM_NOP, rs: PIPE_RS_NOP, wbc: PIPE_WBC_NOP,
    dat: PIPE_DAT_NOP, bp: PIPE_BP_NOP,
    sel_br: br_none
  };
  localparam pipe_ma_t  PIPE_MA_RST  = '{
    com: PIPE_COM_NOP, rs: PIPE_RS_NOP, wbc: PIPE_WBC_NOP,
    dat: PIPE_DAT_RST, bp: PIPE_BP_NOP,
    sel_br: br_none
  };

  localparam pipe_wb_t  PIPE_WB_NOP  = '{
    com: PIPE_COM_NOP, rs: PIPE_RS_NOP, bp: PIPE_BP_NOP,
    imm_raw: 32'd0, rd1_data_raw: 32'd0,
    rd1_data: 32'd0, rd2_data: 32'd0, alu_out: 32'd0,
    branch_taken: 1'b0, sel_wb: wb_none,
    rf_wr_addr: 5'd0, rf_wr_data: 32'd0, rf_wr_en: 1'b0
  };
  localparam pipe_wb_t  PIPE_WB_RST  = '{
    com: PIPE_COM_NOP, rs: PIPE_RS_NOP, bp: PIPE_BP_NOP,
    imm_raw: 32'd0, rd1_data_raw: 32'd0,
    rd1_data: 32'd0, rd2_data: 32'd0, alu_out: 32'd0,
    branch_taken: 1'b0, sel_wb: wb_none,
    rf_wr_addr: 5'd0, rf_wr_data: 32'd0, rf_wr_en: 1'b0
  };

endpackage

`endif // __PCK_PIPE__
