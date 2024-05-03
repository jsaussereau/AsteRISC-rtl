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

//! CPU decompressor

`ifndef __CPU_DECOMPRESSOR__
`define __CPU_DECOMPRESSOR__

`ifdef VIVADO
 `include "packages/pck_isa.sv"
 `include "packages/pck_isa_i.sv"
 `include "packages/pck_isa_c.sv"
`else
 `include "core/packages/pck_isa.sv"
 `include "core/packages/pck_isa_i.sv"
 `include "core/packages/pck_isa_c.sv"
`endif

module cpu_decompressor
  import pck_isa::*;
  import pck_isa_i::*;
  import pck_isa_c::*;
#(
  parameter p_ext_rvc      = 1            //! use RV32C extension
)(
  input  wire          i_clk,             //! global clock
  input  wire          i_rst,             //! global reset
  input  wire          i_en_decomp,       //! enable decompressor
  input  wire          i_update_comp,     //! 
  input  wire          i_bypass_decomp,   //!
  input  wire          i_branch_taken,    //! branch taken
  input  wire          i_half_pc_addr,    //! high when the pc is not a multiple of 4
  input  wire          i_half_npc_addr,   //! high when the npc is not a multiple of 4
  input  isa_instr_t   i_instr,           //! instruction from 'fetch' stage
  output isa_instr_t   o_instr,           //! decompressed instruction
  output logic         o_offset_pc,       //! force fetching next pc
  output logic         o_refetch,         //! force fetching next pc
  output logic         o_freeze_pc,       //! disable next pc determination
  output logic         o_compressed       //! instruction is compressed
);

typedef enum logic [2:0] {
  st_full_joint, // joint 32-bit instruction
  st_full_split, // split 32-bit instruction
  st_half_lower, // lower 16-bit instruction 
  st_half_upper, // upper 16-bit instruction
  st_fetch_next, // fetch next instruction
  st_half_upper_old
} decomp_state_e;

generate 
  if (!p_ext_rvc) begin
    always_comb begin
      o_instr      = i_instr;
      o_offset_pc  = 1'b0;
      o_refetch    = 1'b0;
      o_compressed = 1'b0;
      o_freeze_pc  = 1'b0;
    end
  end else begin
    isa_instr_c_e instr_name_c;

    isa_instr_t   instr_old;      //! old instruction
    isa_pcompr_t  instr_i;        //! input with 2 compressed instructions
    isa_instr_c_t instr_c;        //! compressed instruction
    isa_instr_t   instr_d;        //! decompressed instruction
    logic [11: 1] j_offset;
    logic [ 8: 1] b_offset;

    logic         lower_comp;     //! high when the lower instruction is compressed
    logic         upper_comp;     //! high when the upper instruction is compressed
    logic         lower_comp_old; //! high when the old lower instruction is compressed
    logic         upper_comp_old; //! high when the old upper instruction is compressed

    logic         branch_taken;
    logic         half_pc_addr;
    logic         half_npc_addr;

    //! save last instruction
    always_ff @(posedge i_clk) begin: save_last_instr
      if (i_rst) begin
        instr_old <= 32'b0;
      end else begin
        if (i_update_comp) begin
          instr_old <= i_instr;
        end
      end
    end

    decomp_state_e last_state; //! current fsm state
    decomp_state_e curr_state; //! next fsm state

    //! update the current fsm state every clock cycle
    always_ff @(posedge i_clk) begin: update_curr_state
      if (i_rst) begin
        last_state <= st_full_joint;
      end else begin
        if (i_update_comp) begin
          last_state <= curr_state;
        end
      end
    end

    always_ff @(posedge i_clk) begin
      if (i_rst) begin
        branch_taken  <= 1'b0;
        half_pc_addr  <= 1'b0;
        half_npc_addr <= 1'b0;
      end else begin
        if (i_update_comp) begin
          branch_taken  <= i_branch_taken;
          half_pc_addr  <= i_half_pc_addr;
          half_npc_addr <= i_half_npc_addr;
        end
      end
    end

    //! decode next fsm state
    always_comb begin: next_state_decoder
      case (last_state)
        st_full_joint: begin
          if (branch_taken && half_pc_addr) begin
            curr_state = st_fetch_next;
          end else begin
            if (lower_comp) begin
              curr_state = st_half_lower;
            end else begin
              curr_state = st_full_joint;
            end
          end
        end
        st_half_lower: begin 
          if (branch_taken && !half_pc_addr) begin
            if (lower_comp) begin
              curr_state = st_half_lower;
            end else begin
              curr_state = st_full_joint;
            end
          end else begin
            if (upper_comp_old) begin
              curr_state = st_half_upper;
            end else begin
              curr_state = st_full_split;
            end
          end
        end
        st_half_upper, st_half_upper_old: begin
          if (branch_taken && half_pc_addr) begin
            if (upper_comp) begin
              curr_state = st_half_upper;
            end else begin
              curr_state = st_full_split;
            end
          end else begin
            if (lower_comp) begin
              curr_state = st_half_lower;
            end else begin
              curr_state = st_full_joint;
            end
          end
        end
        st_full_split: begin
          if (branch_taken && !half_npc_addr) begin
            if (lower_comp) begin
              curr_state = st_half_lower;
            end else begin
              curr_state = st_full_joint;
            end
          end else begin
            if (upper_comp_old) begin
              curr_state = st_half_upper;
            end else begin
              curr_state = st_full_split;
            end
          end
        end
        st_fetch_next: begin
          if (upper_comp_old) begin
            curr_state = st_half_upper_old;
          end else begin
            curr_state = st_full_split;
          end
        end
        default: curr_state = st_full_joint;
      endcase
    end

    // output signals
    always_comb begin: output_decoder
      if (i_bypass_decomp) begin
        instr_c.code = 16'b0;
        o_instr      = i_instr;
        o_compressed = 1'b0;
        o_refetch    = 1'b0;
      end else begin
        case (curr_state)
          st_full_joint: begin
            instr_c.code = 16'b0;
            o_instr      = i_instr;
            o_compressed = 1'b0;
            o_offset_pc  = 1'b0;
            o_refetch    = i_branch_taken & (i_half_pc_addr | half_npc_addr);
            o_freeze_pc  = i_branch_taken & (i_half_pc_addr | half_npc_addr);
          end
          st_half_lower: begin
            instr_c.code = i_instr.code[15: 0];
            o_instr      = instr_d;
            o_compressed = 1'b1;
            o_offset_pc  = !upper_comp && i_half_npc_addr;
            o_refetch    = 1'b0;
            o_freeze_pc  = 1'b0;
          end
          st_half_upper: begin
            instr_c.code = i_instr.code[31:16];
            o_instr      = instr_d;
            o_compressed = 1'b1;
            o_offset_pc  = 1'b0;
            o_refetch    = 1'b0;
            o_freeze_pc  = 1'b0;
          end
          st_full_split: begin
            instr_c.code = 16'b0;
            o_instr      = { i_instr[15: 0], instr_old[31:16] };
            o_compressed = 1'b0;
            o_offset_pc  = !upper_comp && i_half_npc_addr;
            o_refetch    = 1'b0;
            o_freeze_pc  = 1'b0;
          end
          st_fetch_next: begin
            instr_c.code = 16'b0;
            o_instr      = 32'b0;
            o_compressed = 1'b0;
            o_offset_pc  = 1'b1;
            o_refetch    = 1'b0;
            o_freeze_pc  = 1'b1;
          end
          st_half_upper_old: begin
            instr_c.code = instr_old[31:16];
            o_instr      = instr_d;
            o_compressed = 1'b1;
            o_offset_pc  = 1'b0;
            o_refetch    = 1'b0;
            o_freeze_pc  = 1'b0;
          end
          default: begin
            instr_c.code = 16'b0;
            o_instr      = 32'b0;
            o_compressed = 1'b0;
            o_offset_pc  = 1'b0;
            o_refetch    = 1'b0;
            o_freeze_pc  = 1'b0;
          end
        endcase
      end
    end

    // check if current input contains compressed instructions
    assign instr_i = i_instr.code;
    always_comb begin
      lower_comp = (instr_i.lower_opcode == pck_isa_c::OPCODE_NC) ? 1'b0 : 1'b1;
      upper_comp = (instr_i.upper_opcode == pck_isa_c::OPCODE_NC || !lower_comp) ? 1'b0 : 1'b1;
      upper_comp_old = (instr_old[17:16] == pck_isa_c::OPCODE_NC) ? 1'b0 : 1'b1;
      lower_comp_old = (instr_old[ 1: 0] == pck_isa_c::OPCODE_NC) ? 1'b0 : 1'b1;
    end

    // get offset for c.j and c.jal instructions
    assign j_offset = { 
      instr_c.cj.imm[12], instr_c.cj.imm[8], instr_c.cj.imm[10:9], instr_c.cj.imm[6],
      instr_c.cj.imm[7], instr_c.cj.imm[2], instr_c.cj.imm[11], instr_c.cj.imm[5:3] 
    };

    // get offset for c.beqz and c.bnez instructions
    assign b_offset = { 
      instr_c.code[12], instr_c.code[6:5], instr_c.code[2], instr_c.code[11:10], instr_c.code[4:3]
    };
    
    // decode current compressed instruction
    always_comb begin
      casez (instr_c.code)
        pck_isa_c::C_NOP      : begin
          instr_name_c        = isa_c_nop;
          instr_d.code        = pck_isa_i::NOP;
        end
        pck_isa_c::C_ZERO     : begin
          instr_name_c        = isa_c_zero;
          instr_d.code        = pck_isa_i::ZERO;
        end
        pck_isa_c::C_EBREAK   : begin
          instr_name_c        = isa_c_ebreak;
          instr_d.code        = pck_isa_i::EBREAK;
        end

        pck_isa_c::C_LWSP     : begin   // lw rd, offset[7:2](x2)
          instr_name_c        = isa_c_lwsp;
          instr_d.code        = pck_isa_i::LW;
          instr_d.i.imm       = { 4'b0, instr_c.ci.imm4_0[3:2], instr_c.ci.imm5, instr_c.ci.imm4_0[6:4], 2'b0 };
          instr_d.i.rs1       = 5'd2; // x2
          instr_d.i.rd        = instr_c.ci.rd_rs1;
        end 
        pck_isa_c::C_SWSP     : begin   // sw rs2, offset[7:2](x2)
          instr_name_c        = isa_c_swsp;
          instr_d.code        = pck_isa_i::SW;
          instr_d.s.imm11_5   = { 4'b0, instr_c.code[8:7], instr_c.code[12] };
          instr_d.s.imm4_0    = { instr_c.code[11:9], 2'b0 };
          instr_d.s.rs1       = 5'd2; // x2
          instr_d.s.rs2       = instr_c.css.rs2;
        end 

        pck_isa_c::C_LW       : begin   // lw rd', offset[6:2](rs1')
          instr_name_c        = isa_c_lw;
          instr_d.code        = pck_isa_i::LW;
          instr_d.i.imm       = { 5'b0, instr_c.cl.imm1_0[5], instr_c.cl.imm4_2, instr_c.cl.imm1_0[6], 2'b0 };
          instr_d.i.rs1       = uncompress_reg_addr(instr_c.cl.rs1_);
          instr_d.i.rd        = uncompress_reg_addr(instr_c.cl.rd_);
        end 
        pck_isa_c::C_SW       : begin   // sw rs2', offset[6:2](rs1')
          instr_name_c        = isa_c_sw;
          instr_d.code        = pck_isa_i::SW;
          instr_d.s.imm11_5   = { 4'b0, instr_c.cs.imm1_0[5], instr_c.cs.imm4_2[12:11] };
          instr_d.s.imm4_0    = { instr_c.cs.imm4_2[10], instr_c.cs.imm1_0[6], 2'b0 };
          instr_d.s.rs1       = uncompress_reg_addr(instr_c.cs.rs1_);
          instr_d.s.rs2       = uncompress_reg_addr(instr_c.cs.rs2_);
        end 

        pck_isa_c::C_J        : begin   // jal x0, offset[11:1]
          instr_name_c        = isa_c_j;
          instr_d.code        = pck_isa_i::JAL;
          instr_d.j.imm20     = j_offset[11];
          instr_d.j.imm19_12  = { 8{j_offset[11]} };
          instr_d.j.imm11     = j_offset[11];
          instr_d.j.imm10_1   = j_offset[10:1];
          instr_d.j.rd        = 5'd0; // x0
        end 
        pck_isa_c::C_JAL      : begin   // jal x1, offset[11:1]
          instr_name_c        = isa_c_jal;
          instr_d.code        = pck_isa_i::JAL;
          instr_d.j.imm20     = j_offset[11];
          instr_d.j.imm19_12  = { 8{j_offset[11]} };
          instr_d.j.imm11     = j_offset[11];
          instr_d.j.imm10_1   = j_offset[10:1];
          instr_d.j.rd        = 5'd1; // x1
        end
        pck_isa_c::C_JR       : begin   // jalr x0, 0(rs1)
          instr_name_c        = isa_c_jr;
          instr_d.code        = pck_isa_i::JALR;
          instr_d.i.imm       = 12'b0;
          instr_d.i.rs1       = instr_c.cr.rd_rs1;
          instr_d.i.rd        = 5'd0; // x0
        end
        pck_isa_c::C_JALR     : begin   // jalr x1, 0(rs1)
          instr_name_c        = isa_c_jalr;
          instr_d.code        = pck_isa_i::JALR;
          instr_d.i.imm       = 12'b0;
          instr_d.i.rs1       = instr_c.cr.rd_rs1;
          instr_d.i.rd        = 5'd1; // x1
        end 
        pck_isa_c::C_BEQZ     : begin   // beq rs1', x0, offset[8:1]
          instr_name_c        = isa_c_beqz;
          instr_d.code        = pck_isa_i::BEQ;
          instr_d.b.imm12     = 1'b0;
          instr_d.b.imm10_5   = { 1'b0, b_offset[8:4] };
          instr_d.b.rs2       = 5'd0; // x0
          instr_d.b.rs1       = uncompress_reg_addr(instr_c.cb.rd_rs1_);
          instr_d.b.imm4_1    = { b_offset[3:1], 1'b0 };
          instr_d.b.imm11     = 1'b0;
        end
        pck_isa_c::C_BNEZ     :  begin  // bne rs1', x0, offset[8:1]
          instr_name_c        = isa_c_bnez;
          instr_d.code        = pck_isa_i::BNE;
          instr_d.b.imm12     = 1'b0;
          instr_d.b.imm10_5   = { 1'b0, b_offset[8:4] };
          instr_d.b.rs2       = 5'd0; // x0
          instr_d.b.rs1       = uncompress_reg_addr(instr_c.cb.rd_rs1_);
          instr_d.b.imm4_1    = { b_offset[3:1], 1'b0 };
          instr_d.b.imm11     = 1'b0;
        end

        pck_isa_c::C_ADDI     : begin   // addi rd, rd, nzimm[5:0]
          instr_name_c        = isa_c_addi;
          instr_d.code        = pck_isa_i::ADDI;
          instr_d.i.imm       = { {6{instr_c.ci.imm5}}, instr_c.ci.imm5, instr_c.ci.imm4_0 };
          instr_d.i.rs1       = instr_c.ci.rd_rs1;
          instr_d.i.rd        = instr_c.ci.rd_rs1;
        end
        pck_isa_c::C_ADDI16SP : begin   // addi x2, x2, nzimm[9:4]
          instr_name_c        = isa_c_addi16sp;
          instr_d.code        = pck_isa_i::ADDI;
          instr_d.i.imm       = { {2{instr_c.code[12]}}, instr_c.code[12], instr_c.code[4:3], instr_c.code[5], instr_c.code[2], instr_c.code[6], 4'b0 };
          instr_d.i.rs1       = 5'd2; // x2;
          instr_d.i.rd        = 5'd2; // x2;
        end
        pck_isa_c::C_ADDI4SPN : begin   // addi rd ′, x2, nzuimm[9:2]
          instr_name_c        = isa_c_addi4spn;
          instr_d.code        = pck_isa_i::ADDI;
          instr_d.i.imm       = { 4'b0, instr_c.code[10:7], instr_c.code[12:11], instr_c.code[5], instr_c.code[6], 2'b0 };
          instr_d.i.rs1       = 5'd2; // x2;
          instr_d.i.rd        = uncompress_reg_addr(instr_c.ciw.rd_);
        end
        pck_isa_c::C_SLLI     : begin   // slli rd, rd, shamt[5:0]
          instr_name_c        = isa_c_slli;
          instr_d.code        = pck_isa_i::SLLI;
          instr_d.i.imm       = { 6'b0, instr_c.ciw.imm };
          instr_d.i.rs1       = instr_c.ciw.rd_;
          instr_d.i.rd        = instr_c.ciw.rd_;
        end
        pck_isa_c::C_SRLI     : begin   // srli rd',rd', shamt[5:0]
          instr_name_c        = isa_c_srli;
          instr_d.code        = pck_isa_i::SRLI;
          instr_d.i.imm       = { 6'b0, instr_c.cb2.imm5, instr_c.cb2.imm4_0 };
          instr_d.i.rs1       = uncompress_reg_addr(instr_c.cb2.rd_rs1_);
          instr_d.i.rd        = uncompress_reg_addr(instr_c.cb2.rd_rs1_);
        end
        pck_isa_c::C_SRAI     : begin   // srai rd', rd', shamt[5:0]
          instr_name_c        = isa_c_srai;
          instr_d.code        = pck_isa_i::SRAI;
          instr_d.i.imm       = { 6'b0, instr_c.cb2.imm5, instr_c.cb2.imm4_0 };
          instr_d.i.rs1       = uncompress_reg_addr(instr_c.cb2.rd_rs1_);
          instr_d.i.rd        = uncompress_reg_addr(instr_c.cb2.rd_rs1_);
        end
        pck_isa_c::C_ANDI     : begin   // andi rd', rd', imm[5:0]
          instr_name_c        = isa_c_andi;
          instr_d.code        = pck_isa_i::ANDI;
          instr_d.i.imm       = { 6'b0, instr_c.cb2.imm5, instr_c.cb2.imm4_0 };
          instr_d.i.rs1       = uncompress_reg_addr(instr_c.cb2.rd_rs1_);
          instr_d.i.rd        = uncompress_reg_addr(instr_c.cb2.rd_rs1_);
        end

        pck_isa_c::C_LI       : begin   // addi rd, x0, imm[5:0]
          instr_name_c        = isa_c_li;
          instr_d.code        = pck_isa_i::ADDI;
          instr_d.i.imm       = { 6'b0, instr_c.ci.imm5, instr_c.ci.imm4_0 };
          instr_d.i.rs1       = 5'd0; // x0
          instr_d.i.rd        = instr_c.ci.rd_rs1;
        end
        pck_isa_c::C_LUI      : begin   // lui rd, nzimm[17:12]
          instr_name_c        = isa_c_lui;
          instr_d.code        = pck_isa_i::LUI;
          instr_d.u.imm       = { {14{instr_c.ci.imm5}}, instr_c.ci.imm5, instr_c.ci.imm4_0 }; //sext
          instr_d.u.rd        = instr_c.ci.rd_rs1;
        end

        pck_isa_c::C_MV       : begin   // add rd, x0, rs2
          instr_name_c        = isa_c_mv;
          instr_d.code        = pck_isa_i::ADD;
          instr_d.r.rs2       = instr_c.cr.rs2;
          instr_d.r.rs1       = 5'd0; // x0;
          instr_d.r.rd        = instr_c.cr.rd_rs1;
        end
        pck_isa_c::C_ADD      : begin   // add rd, rd, rs2
          instr_name_c        = isa_c_add;
          instr_d.code        = pck_isa_i::ADD;
          instr_d.r.rs2       = instr_c.cr.rs2;
          instr_d.r.rs1       = instr_c.cr.rd_rs1;
          instr_d.r.rd        = instr_c.cr.rd_rs1;
        end
        pck_isa_c::C_AND      : begin   // and rd', rd', rs2' 
          instr_name_c        = isa_c_and;
          instr_d.code        = pck_isa_i::AND;
          instr_d.r.rs2       = uncompress_reg_addr(instr_c.ca.rs2_);
          instr_d.r.rs1       = uncompress_reg_addr(instr_c.ca.rd_rs1_);
          instr_d.r.rd        = uncompress_reg_addr(instr_c.ca.rd_rs1_);
        end
        pck_isa_c::C_OR       : begin   // or rd', rd', rs2'
          instr_name_c        = isa_c_or;
          instr_d.code        = pck_isa_i::OR;
          instr_d.r.rs2       = uncompress_reg_addr(instr_c.ca.rs2_);
          instr_d.r.rs1       = uncompress_reg_addr(instr_c.ca.rd_rs1_);
          instr_d.r.rd        = uncompress_reg_addr(instr_c.ca.rd_rs1_);
        end
        pck_isa_c::C_XOR      : begin   // xor rd', rd', rs2'
          instr_name_c        = isa_c_xor;
          instr_d.code        = pck_isa_i::XOR;
          instr_d.r.rs2       = uncompress_reg_addr(instr_c.ca.rs2_);
          instr_d.r.rs1       = uncompress_reg_addr(instr_c.ca.rd_rs1_);
          instr_d.r.rd        = uncompress_reg_addr(instr_c.ca.rd_rs1_);
        end
        pck_isa_c::C_SUB      : begin   // sub rd', rd', rs2'
          instr_name_c        = isa_c_sub;
          instr_d.code        = pck_isa_i::SUB;
          instr_d.r.rs2       = uncompress_reg_addr(instr_c.ca.rs2_);
          instr_d.r.rs1       = uncompress_reg_addr(instr_c.ca.rd_rs1_);
          instr_d.r.rd        = uncompress_reg_addr(instr_c.ca.rd_rs1_);
        end

        // currently unsupported instructions:

        // RV32FC only:
        pck_isa_c::C_FLWSP    : begin
          instr_name_c        = isa_c_flwsp;
          instr_d.code        = pck_isa_i::ZERO;
        end
        pck_isa_c::C_FSWSP    : begin
          instr_name_c        = isa_c_fswsp;
          instr_d.code        = pck_isa_i::ZERO;
        end
        pck_isa_c::C_FLW      : begin
          instr_name_c        = isa_c_flw;
          instr_d.code        = pck_isa_i::ZERO;
        end
        pck_isa_c::C_FSW      : begin
          instr_name_c        = isa_c_fsw;
          instr_d.code        = pck_isa_i::ZERO;
        end

        // RV32DC only:
        pck_isa_c::C_FLDSP    : begin
          instr_name_c        = isa_c_fldsp;
          instr_d.code        = pck_isa_i::ZERO;
        end
        pck_isa_c::C_FSDSP    : begin
          instr_name_c        = isa_c_fsdsp;
          instr_d.code        = pck_isa_i::ZERO;
        end
        pck_isa_c::C_FLD      : begin
          instr_name_c        = isa_c_fld;
          instr_d.code        = pck_isa_i::ZERO;
        end
        pck_isa_c::C_FSD      : begin
          instr_name_c        = isa_c_fsd;
          instr_d.code        = pck_isa_i::ZERO;
        end

        default               : begin
          instr_name_c        = isa_c_other;
          instr_d.code        = pck_isa_i::ZERO;
        end
      endcase
    end  
  end
endgenerate

endmodule

`endif // __CPU_DECOMPRESSOR__
