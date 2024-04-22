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

`ifndef __CPU_CSR__
`define __CPU_CSR__

`ifdef VIVADO
  `include "packages/pck_control.sv"
  `include "packages/pck_isa_zicsr.sv"
`else
  `include "core/packages/pck_control.sv"
  `include "core/packages/pck_isa_zicsr.sv"
`endif

 //TODO: bufferize o_rd_data for swap operations

module cpu_csr
  import pck_control::*;
  import pck_isa_zicsr::*;
#(
  parameter p_ext_rvzicsr = 0
)(
  input  logic        i_clk,              //! global clock
  input  logic        i_rst,              //! global reset

  input  logic [63:0] i_mcycle,           //! cycle count
  input  logic [63:0] i_mtime,            //! real time clock
  input  logic [63:0] i_minstret,         //! instructions-retired

  input  wire  [31:0] i_imm,              //! immediate value
  input  sel_csr_wr_e i_sel_csr_wr,       //! csr write data selector
  input  sel_csr_op_e i_sel_csr_op,       //! csr write operation selector

  input  logic [11:0] i_addr,             //! csr address
  input  logic        i_wr_en,            //! csr write enable
  input  wire  [31:0] i_rf_rd1_data,      //! regfile memory read data on port 1
  output logic [31:0] o_rd_data           //! csr read data
);

if (p_ext_rvzicsr) begin
  
  logic [31:0] rd_data;
  logic [31:0] wr_data;
  logic [31:0] wr_mask;

  /* no write yet
  control_status_registers csr ();
  */

  logic [31:0] test_csr;

  //! mask select
  always_comb begin: csr_mask_sel
    case (i_sel_csr_wr)
      csr_wr_rf    : wr_mask = i_rf_rd1_data;
      csr_wr_imm   : wr_mask = i_imm;
      default      : wr_mask = 32'd0;
    endcase
  end

  //! operation select
  always_comb begin: csr_op_sel
    case (i_sel_csr_op)
      csr_op_none  : wr_data = rd_data;
      csr_op_swap  : wr_data = wr_mask;
      csr_op_set   : wr_data = rd_data | wr_mask;
      csr_op_clear : wr_data = rd_data & ~wr_mask;
      default      : wr_data = 32'd0;
    endcase
  end

  always_ff @(posedge i_clk) begin: registers_write
    if (i_rst) begin
      test_csr <= 32'b0;
    end else begin 
      if (i_wr_en) begin
        case (i_addr)
          pck_isa_zicsr::ISA_CSR_TEST     : test_csr <= wr_data;
        endcase
      end
    end
  end

  always_comb begin: registers_read
    if (i_rst) begin
      rd_data = 0;
    end else begin
      case (i_addr)
        // Machine Information Registers
        12'hf11 : rd_data = 32'hdeadbeef;
        pck_isa_zicsr::ISA_CSR_MARCHID    : rd_data = 32'd42;
        pck_isa_zicsr::ISA_CSR_MIMPID     : rd_data = 32'd42;
        pck_isa_zicsr::ISA_CSR_MHARTID    : rd_data = 32'd1;
      
        //  User Counter/Timers
        pck_isa_zicsr::ISA_CSR_CYCLEH     : rd_data = i_mcycle[63:32];
        pck_isa_zicsr::ISA_CSR_CYCLE      : rd_data = i_mcycle[31: 0];
        pck_isa_zicsr::ISA_CSR_TIMEH      : rd_data = i_mtime[63:32];
        pck_isa_zicsr::ISA_CSR_TIME       : rd_data = i_mtime[31: 0];
        pck_isa_zicsr::ISA_CSR_INSTRETH   : rd_data = i_minstret[63:32];
        pck_isa_zicsr::ISA_CSR_INSTRET    : rd_data = i_minstret[31: 0];

        // Machine Counter/Timers
        pck_isa_zicsr::ISA_CSR_MCYCLEH    : rd_data = i_mcycle[63:32];
        pck_isa_zicsr::ISA_CSR_MCYCLE     : rd_data = i_mcycle[31: 0];
        pck_isa_zicsr::ISA_CSR_MINSTRETH  : rd_data = i_minstret[63:32];
        pck_isa_zicsr::ISA_CSR_MINSTRET   : rd_data = i_minstret[31: 0];

        pck_isa_zicsr::ISA_CSR_TEST       : rd_data = test_csr;

        default                           : rd_data = 32'b0;
      endcase
    end
  end 
  assign o_rd_data = rd_data;

end else begin
  assign o_rd_data = 32'b0;
end

endmodule

`endif // __CPU_CSR__
