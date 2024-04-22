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

//! Testbench for CPU minimal SoC

`ifndef __TB_SOC_FULL__
`define __TB_SOC_FULL__


module tb_soc_full #(
  // simulation:
  parameter p_runtime = 500,
  parameter half_period = 5, // 10 ns period

  // uut:
  parameter p_reset_vector = 32'hf0000000,
  parameter p_imem_init    = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/mega_example/bin/mega_example_imem.hex",
  parameter p_dmem_init    = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/mega_example/bin/mega_example_dmem.hex",
  //parameter p_imem_init    = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/cpu_example/bin/cpu_example_imem.hex",
  //parameter p_dmem_init    = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/cpu_example/bin/cpu_example_dmem.hex",
  //parameter p_imem_init    = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/tests/bin/tests_imem.hex",
  //parameter p_dmem_init    = "/home/jsaussereau/Documents/bdxrcg/asterisc-rv32i-2c/firmware/tests/bin/tests_dmem.hex",
  parameter p_num_gpios    = 24
)();

  localparam period = 2*half_period;

  logic clk = 0;
  always #half_period clk = ~clk;

  logic rst = 0;

  
  logic         sleep;
  logic [63: 0] mtime;

  initial begin

    $display($time, " <<< Simulation starts... >>>");

    // apply reset
    rst             = 1'b1;
    #(2*period)
    rst             = 1'b0;

    //#(p_runtime*period)
    //$display($time, " <<< Simulation ends... >>>");
    //$stop;

  end

  soc_full #(
    .p_reset_vector ( p_reset_vector ),
    .p_imem_init    ( p_imem_init    ),
    .p_dmem_init    ( p_dmem_init    )
  ) soc (
    // global
    .i_clk          ( clk            ), //! global clock: triggers with a rising edge
    .i_rst          ( rst            ), //! global reset: **active high**, synchronous
    .i_sleep        ( sleep          ), //! active high sleep control
    .i_mtime        ( mtime          ), //! mtime system time 
    .i_gpio_in      (                ),
    .o_gpio_out     (                ),
    .o_gpio_out_en  (                ),
    .o_gpio_pullup  (                ),
    .o_gpio_pulldown(                )
  );

endmodule

`endif // __TB_SOC_FULL__
