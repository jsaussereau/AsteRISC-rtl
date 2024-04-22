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

`ifndef SYNTHESIS

`ifndef __TB_WRAPPER_MUSTANG_TOP__
`define __TB_WRAPPER_MUSTANG_TOP__

`timescale 1ns / 1ps

`ifdef VIVADO
  `include "../../core/cpu_core.sv"
`endif

//////////////////////////////////////////////////////////////
// COMMENT THE LINE BELLOW TO USE NON-PIPELINE ARCHITECTURE //
//////////////////////////////////////////////////////////////
//`define USE_PIPELINE
//////////////////////////////////////////////////////////////

`ifdef USE_PIPELINE
 `define cpu_ref uut.soc_top_level.pipe.cpu
 `define P_PIPELINE 1
`else
 `define cpu_ref uut.soc_top_level.nonpipe.cpu
 `define P_PIPELINE 0
`endif

module tb_wrapper_top #(
  // simulation:
  parameter p_runtime   = 500,

`ifdef TARGET_FPGA_XC7K325T
  parameter half_period = 2.5, // 5 ns period
`else
  parameter half_period = 5, // 10 ns period
`endif

  parameter p_instruction_log = 0,
  parameter p_bypass_log      = 0,
  parameter p_software_print  = 1,

  // uut:
  parameter p_reset_vector    = 32'hf0000000,
  parameter p_num_gpios       = 24,

  //imem/dmem init files
  //parameter p_imem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/mega_example/bin/mega_example_imem.hex",
  //parameter p_dmem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/mega_example/bin/mega_example_dmem.hex",
  //parameter p_imem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/mega_example/bin_comp/mega_example_imem.hex",
  //parameter p_dmem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/mega_example/bin_comp/mega_example_dmem.hex",
  //parameter p_imem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/mul_example/bin/mul_example_imem.hex",
  //parameter p_dmem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/mul_example/bin/mul_example_dmem.hex",
  //parameter p_imem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/uart_example/bin/uart_example_imem.hex",
  //parameter p_dmem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/uart_example/bin/uart_example_dmem.hex",
  //parameter p_imem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/dhrystone_benchmark/bin/dhrystone_benchmark_imem.hex",
  //parameter p_dmem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/dhrystone_benchmark/bin/dhrystone_benchmark_dmem.hex",
  //parameter p_imem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/pointeur/bin/pointeur_imem.hex",
  //parameter p_dmem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/pointeur/bin/pointeur_dmem.hex",
  //parameter p_imem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/crc_example/bin/crc_example_imem.hex",
  //parameter p_dmem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/crc_example/bin/crc_example_dmem.hex",
  //parameter p_imem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/swap_example/bin/swap_example_imem.hex",
  //parameter p_dmem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/swap_example/bin/swap_example_dmem.hex",
  parameter p_imem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/coremark/bin/coremark_benchmark_imem.hex",
  parameter p_dmem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/coremark/bin/coremark_benchmark_dmem.hex",
  //parameter p_imem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/tests/bin/tests_imem.hex",
  //parameter p_dmem_init       = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/tests/bin/tests_dmem.hex",
  
  //chip ids
  parameter p_manufacturer_id = 12'h456,
  parameter p_product_id      = 8'h01,

  //imem/dmem detpth (power of two)
  parameter p_imem_depth_pw2  = 14,
  parameter p_dmem_depth_pw2  = 13,

  //RISC-V extensions
  parameter p_ext_rve         = 0,            //! use RV32E extension (reduces the integer register count to 
  parameter p_ext_rvc         = 0,            //! use RV32C extension (compressed instructions)
  parameter p_ext_rvm         = 0,            //! use RV32M extension (multiplication and division)
  parameter p_ext_rvzicsr     = 1,            //! use RV32Zicsr extension (control and status registers)
  parameter p_ext_custom      = 1,            //! use RV32Zicsr extension (control and status registers)

  parameter p_counters        = 1,            //! use counters (mcycle, minstret, mtime)

  parameter p_mul_fast        = 0,            //! fast mul
  parameter p_mul_1_cycle     = 0,            //! one cycle mul

  parameter p_pipeline        = `P_PIPELINE,

  // pipeline settings:   
  parameter p_stage_IF        = 1,
  parameter p_stage_IC        = 0,
  parameter p_stage_ID        = 1,
  parameter p_stage_RF        = 0,
  parameter p_stage_EX        = 1,
  parameter p_stage_MA        = 1,
  parameter p_stage_WB        = 1,

  // non pipeline settings:   
  parameter p_prefetch_buf    = 0,           //! use a prefetch buffer
  parameter p_decode_buf      = 1,           //! add buffers to decode stage outputs
  parameter p_rf_sp           = 1,           //! register file is a single port ram
  parameter p_rf_read_buf     = 1,           //! register file has synchronous read
  parameter p_mem_buf         = 0,           //! add buffers to mem stage inputs
  parameter p_wb_buf          = 1,           //! add buffers to write back stage inputs
  parameter p_branch_buf      = 0,           //! add buffers to alu comp outputs (+1 cycle for conditionnal branches)

  //security
  parameter p_wait_for_ack    = 0
);

  localparam period = 2*half_period;

  logic clk = 0;
  always #half_period clk = ~clk;

  logic rst = 0;

  wire  [p_num_gpios-1:0] io_gpio;
  logic [p_num_gpios-1:0] gpio;

  logic en_uart = 0;

  initial begin

    $display($time, " <<< Simulation starts... >>>");

    // apply reset
    rst             <= 1'b1;
    //TODO: link reset and pll_locked ? por reset cycles > pll lock time ? Asynchronous reset ?
    #(50*period)
    rst             <= 1'b0;

    gpio[15]        <= 1'b0;
    #(1500*period)
    gpio[15]        <= 1'b1;
    #(1500*period)
    en_uart         <= 1'b1;

    //#(p_runtime*period)
    //$display($time, " <<< Simulation ends... >>>");
    //$stop;

  end

  initial begin
    integer i;
    for (i = 0 ; i < 2**p_imem_depth_pw2 ; i++) begin
      uut.soc_top_level.imem.imem.mem[i] = 32'h00000003;
    end
    $readmemh(p_imem_init, uut.soc_top_level.imem.imem.mem);
  end

  initial begin
    integer i;
    for (i = 0 ; i < 2**p_dmem_depth_pw2 ; i++) begin
      uut.soc_top_level.dmem.dmem.mem[i] = 32'h00000000;
    end
    $readmemh(p_dmem_init, uut.soc_top_level.dmem.dmem.mem);
  end

  assign io_gpio[15] = gpio[15];
  assign io_gpio[18] = gpio[18];

  soc_wrapper_top #(
    .p_reset_vector    ( p_reset_vector    ),
    .p_imem_init       ( p_imem_init       ),
    .p_dmem_init       ( p_dmem_init       ),
    .p_num_gpios       ( p_num_gpios       ),
    .p_manufacturer_id ( p_manufacturer_id ),
    .p_product_id      ( p_product_id      ),
    .p_imem_depth_pw2  ( p_imem_depth_pw2  ),
    .p_dmem_depth_pw2  ( p_dmem_depth_pw2  ),
    .p_ext_rvc         ( p_ext_rvc         ),
    .p_ext_rve         ( p_ext_rve         ),
    .p_ext_rvm         ( p_ext_rvm         ),
    .p_ext_rvzicsr     ( p_ext_rvzicsr     ),
    .p_ext_custom      ( p_ext_custom      ),
    .p_counters        ( p_counters        ),
    .p_mul_fast        ( p_mul_fast        ),
    .p_mul_1_cycle     ( p_mul_1_cycle     ),
    .p_pipeline        ( p_pipeline        ),
    .p_stage_IF        ( p_stage_IF        ),
    .p_stage_IC        ( p_stage_IC        ),
    .p_stage_ID        ( p_stage_ID        ),
    .p_stage_RF        ( p_stage_RF        ),
    .p_stage_EX        ( p_stage_EX        ),
    .p_stage_MA        ( p_stage_MA        ),
    .p_stage_WB        ( p_stage_WB        ),
    .p_prefetch_buf    ( p_prefetch_buf    ),
    .p_decode_buf      ( p_decode_buf      ),
    .p_rf_sp           ( p_rf_sp           ),
    .p_rf_read_buf     ( p_rf_read_buf     ),
    .p_branch_buf      ( p_branch_buf      ),
    .p_mem_buf         ( p_mem_buf         ),
    .p_wb_buf          ( p_wb_buf          ),
    .p_wait_for_ack    ( p_wait_for_ack    )
  ) uut (            
    .i_xtal_p          ( clk               ),
    .i_xtal_n          ( ~clk              ),
    .i_xclk            (                   ),
    .i_xrst            ( rst               ),
    .i_sck             ( 1'b0              ),
    .i_sdi             ( 1'b0              ),
    .o_sdo             (                   ),
    .i_csb             ( 1'b0              ),
    .o_flash_clk       (                   ),
    .o_flash_csb       (                   ),
    .io_flash0         (                   ),
    .io_flash1         (                   ),
    .io_flash2         (                   ),
    .io_flash3         (                   ),
    .io_gpio           ( io_gpio           )
  );

  perif_uart_tx uart_tx ( 
    .i_clk             ( clk               ), 
    .i_rst             ( rst               ), 
    .i_en              ( en_uart           ),
    .i_wr_en           ( en_uart           ),
    .i_baudrate        ( 16'd347           ), // 115200 baud @ 40 MHz (40*10^6 / 115200 = 347)
    //.i_baudrate     ( 16'd10                        ),
    .i_data_tx         ( 8'd77             ), // 'M'
    .o_tx_full         (                   ),
    .o_uart_tx         ( gpio[18]          )
  );

  // print characters written in debug module (first byte)
  if (p_software_print) begin
    always @(posedge clk) begin
      if (uut.soc_top_level.dbus_wr_en) begin
        if (uut.soc_top_level.dbus_addr == 32'h0x0A000000) begin
          $write("%c", uut.soc_top_level.dbus_wr_data);
        end
      end
    end
  end
    
  // stop simulation if a ebreak is received
  always @(posedge clk) begin
    if (`cpu_ref.debug_instr_code == 32'h00100073) begin
      $display("ebreak");
      $stop;
    end
  end
  
  logic crc_calc;
  logic crc_tab;

  always @(*) begin
    crc_calc = 0;
    case (`cpu_ref.debug_pc)
      32'hF00000A4, 32'hF00000A8, 32'hF00000AC, 32'hF00000B0, 32'hF00000B4,
      32'hF00000B8, 32'hF00000BC, 32'hF00000C0, 32'hF00000C4, 32'hF00000C8,
      32'hF00000CC, 32'hF00000D0, 32'hF00000D4, 32'hF00000D8, 32'hF00000DC,
      32'hF00000E0, 32'hF00000E4:
        crc_calc = 1;
      default:
        crc_calc = 0;
    endcase
    
    crc_tab = 0;
    case (`cpu_ref.debug_pc)
      32'hF00000E8, 32'hF00000EC, 32'hF00000F0, 32'hF00000F4, 32'hF00000F8,
      32'hF00000FC, 32'hF0000100, 32'hF0000104, 32'hF0000108, 32'hF000010C:
        crc_tab = 1;
      default:
        crc_tab = 0;
    endcase
  end
  
  function automatic string strtail(string str);
    strtail = str.substr(4, str.len()-1);
  endfunction

 /* function automatic write_instr(string name, logic[31:0] wb_addr, rs1_addr, rs2_addr, rs1_val, rs2_val, imm);
    $write("%8s r%0d, r%0d r%0d: ", strtail(name), wb_addr, rs1_addr, rs2_addr);
  endfunction*/

  if (p_instruction_log) begin
    logic [63:0] last_instret = 2;
    logic debug_print_flag;
    // log each instruction
    //always @(`cpu_ref.debug_instret) begin
    always @(posedge clk) begin
      debug_print_flag <= 1'b0;
      if (`cpu_ref.debug_instret != 0 && `cpu_ref.debug_instret != last_instret) begin
        debug_print_flag <= 1'b1;
        last_instret <= `cpu_ref.debug_instret;
        $write("%0d) pc 0x%08x:\n ", `cpu_ref.debug_instret, `cpu_ref.debug_pc);
        $write(" instr: 0x%08x > %s\n", `cpu_ref.debug_instr_code, strtail(`cpu_ref.debug_instr_name.name()));
        if (`cpu_ref.debug_br_taken) begin
          $write("  -> branch taken\n");
        end
        $write("  imm       = 0x%08x\n", `cpu_ref.debug_imm);
        $write("  rs1 (x%02d) = 0x%08x", `cpu_ref.debug_rs1_addr, `cpu_ref.debug_rs1_data);
        if (p_bypass_log && `cpu_ref.debug_bp_rs1_ex) begin
          $write(" (bp ex)\n");
        end else if (p_bypass_log && `cpu_ref.debug_bp_rs1_ma) begin
          if (`cpu_ref.debug_stall_rs1) begin
            $write(" (stall + ");
          end else begin
            $write(" (");
          end
          $write("bp ma)\n");
        end else if (p_bypass_log && `cpu_ref.debug_bp_rs1_wb) begin
          $write(" (bp wb)\n");
        end else begin
          $write("\n");
        end
        if (`cpu_ref.debug_rs2_used) begin
          $write("  rs2 (x%02d) = 0x%08x", `cpu_ref.debug_rs2_addr, `cpu_ref.debug_rs2_data);
          if (p_bypass_log && `cpu_ref.debug_bp_rs2_ex) begin
            $write(" (bp ex)\n");
          end else if (p_bypass_log && `cpu_ref.debug_bp_rs2_ma) begin
            if (`cpu_ref.debug_stall_rs2) begin
              $write(" (stall + ");
            end else begin
              $write(" (");
            end
            $write("bp ma)\n");
          end else if (p_bypass_log && `cpu_ref.debug_bp_rs2_wb) begin
            $write(" (bp wb)\n");
          end else begin
            $write("\n");
          end
        end
        if (`cpu_ref.debug_wb_en) begin
          $write("  rd  (x%02d) = 0x%08x\n", `cpu_ref.debug_wb_addr, `cpu_ref.debug_wb_data);  
        end else begin
          //$write("  <<<rd  (x%02d) = 0x%08x\n", `cpu_ref.debug_wb_addr, `cpu_ref.debug_wb_data);  
          //$write("\n");
        end
      end
    end
  end

endmodule

`endif /* __TB_WRAPPER_MUSTANG_TOP__ */

`endif
