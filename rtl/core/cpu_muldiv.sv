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


`ifndef __CPU_MULDIV__
`define __CPU_MULDIV__

`ifdef VIVADO
  `include "../soc/soc_config.sv"
  `include "packages/pck_control.sv"
`else
  `include "soc/soc_config.sv"
  `include "core/packages/pck_control.sv"
`endif

module cpu_muldiv
  import pck_control::*;
#(
  parameter p_mul_fast     = 0,           //! fast mul
  parameter p_mul_1_cycle  = 0            //! one cycle mul
)(
  input  wire          i_clk,             //! global clock
  input  wire          i_rst,             //! global reset

  input  wire          i_en_exec,         //! enable execution

  input  wire          i_opa_signed,      //! MUL/DIV operand A is signed
  input  wire          i_opb_signed,      //! MUL/DIV operand B is signed
  input  sel_md_op_e   i_sel_md_op,       //! MUL/DIV operation selector

  input  wire  [31:0]  i_op_a,            //! operand A
  input  wire  [31:0]  i_op_b,            //! operand B
  input  wire  [31:0]  i_alu_out,         //! ALU result
  input  wire          i_alu_null_out,    //! ALU result is null
  input  wire  [33:0]  i_alu_adder_out,   //! full length adder output

  output logic [32:0]  o_alu_op_a_md,     //! ALU operand A from muldiv
  output logic [32:0]  o_alu_op_b_md,     //! ALU operand B from muldiv
  output logic         o_alu_use_md,     //! use muldiv opa and opb outputs as ALU input

  output logic [31:0]  o_out,             //! MUL/DIV result
  output logic         o_exec_done        //! MUL/DIV done (data valid)
);

  logic        mul_en;                    //! MUL enable
  logic        div_en;                    //! DIV enable
  logic [ 1:0] signed_mode;               //! operands signed/unsigned

  logic [33:0] imd_val_d_ex [2];          //! intermediate register for multicycle ops
  logic [33:0] imd_val_q_ex [2];          //! intermediate register for multicycle ops
  logic [ 1:0] imd_val_we_ex;

  always_comb begin
    case (i_sel_md_op)
      muldiv_mull, muldiv_mulh: begin
        o_alu_use_md = 1'b1;
        mul_en       = 1'b1;
        div_en       = 1'b0;
      end
      muldiv_div, muldiv_rem: begin
        o_alu_use_md = 1'b1;
        mul_en       = 1'b0;
        div_en       = 1'b1;
      end
      default: begin
        o_alu_use_md = 1'b0;
        mul_en       = 1'b0;
        div_en       = 1'b0;
      end
    endcase
  end

  assign signed_mode = { i_opa_signed, i_opb_signed };

  for (genvar i = 0 ; i < 2 ; i++) begin: gen_intermediate_val_reg
    always_ff @(posedge i_clk) begin: intermediate_val_reg
      if (i_rst) begin
        imd_val_q_ex[i] <= '0;
      end else if (imd_val_we_ex[i]) begin
        imd_val_q_ex[i] <= imd_val_d_ex[i];
      end
    end
  end

  generate 
    if (p_mul_fast) begin
      ibex_multdiv_fast #(
        .p_mul_1_cycle      ( p_mul_1_cycle      )
      ) muldiv (
        .clk_i              ( i_clk              ),
        .rst_ni             ( ~i_rst             ),
        .mult_en_i          ( mul_en & i_en_exec ),
        .div_en_i           ( div_en & i_en_exec ),
        .mult_sel_i         ( mul_en             ),
        .div_sel_i          ( div_en             ),
        .operator_i         ( i_sel_md_op        ),
        .signed_mode_i      ( signed_mode        ),
        .op_a_i             ( i_op_a             ),
        .op_b_i             ( i_op_b             ),
        .alu_adder_ext_i    ( i_alu_adder_out    ),
        .alu_adder_i        ( i_alu_out          ),
        .equal_to_zero_i    ( i_alu_null_out     ),
        .data_ind_timing_i  ( 1'b0               ),

        .alu_operand_a_o    ( o_alu_op_a_md      ),
        .alu_operand_b_o    ( o_alu_op_b_md      ),

        .imd_val_q_i        ( imd_val_q_ex       ),
        .imd_val_d_o        ( imd_val_d_ex       ),
        .imd_val_we_o       ( imd_val_we_ex      ),

        .multdiv_ready_id_i ( i_en_exec          ),
        .multdiv_result_o   ( o_out              ),

        .valid_o            ( o_exec_done        )
      );
    end else begin
      ibex_multdiv_slow muldiv (
        .clk_i              ( i_clk              ),
        .rst_ni             ( ~i_rst             ),
        .mult_en_i          ( mul_en & i_en_exec ),
        .div_en_i           ( div_en & i_en_exec ),
        .mult_sel_i         ( mul_en             ),
        .div_sel_i          ( div_en             ),
        .operator_i         ( i_sel_md_op        ),
        .signed_mode_i      ( signed_mode        ),
        .op_a_i             ( i_op_a             ),
        .op_b_i             ( i_op_b             ),
        .alu_adder_ext_i    ( i_alu_adder_out    ),
        .alu_adder_i        ( i_alu_out          ),
        .equal_to_zero_i    ( i_alu_null_out     ),
        .data_ind_timing_i  ( 1'b0               ),

        .alu_operand_a_o    ( o_alu_op_a_md      ),
        .alu_operand_b_o    ( o_alu_op_b_md      ),

        .imd_val_q_i        ( imd_val_q_ex       ),
        .imd_val_d_o        ( imd_val_d_ex       ),
        .imd_val_we_o       ( imd_val_we_ex      ),

        .multdiv_ready_id_i ( i_en_exec          ),
        .multdiv_result_o   ( o_out              ),

        .valid_o            ( o_exec_done        )
      );
    end
  endgenerate

endmodule

`endif // __CPU_MULDIV__
