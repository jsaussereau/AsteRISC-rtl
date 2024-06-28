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

`ifndef __SOC_TOP_WRAPPER__
`define __SOC_TOP_WRAPPER__

`ifdef VIVADO
  `include "soc_config.sv"
`else
  `include "soc/soc_config.sv"
`endif
 
module soc_wrapper_top #(
  /* verilator public_on*/

  //general settings
  parameter p_reset_vector    = 32'hf0000000,
  parameter p_num_gpios       = 24,

  //chip ids
  parameter p_manufacturer_id = 12'h456,
  parameter p_product_id      = 8'h01,

  //imem/dmem detpth (power of two)
  parameter p_imem_depth_pw2  = 14,           //! depth of the instruction memory in power of two (number of 32-bit words)
  parameter p_dmem_depth_pw2  = 13,           //! depth of the data memory in power of two (number of 32-bit words)

  //imem/dmem init files (simulations)
  parameter string p_imem_init = "/home/jsaussereau/Documents/bdxrcg/AsteRISC/AsteRISC-firmware/hex/dhrystone_benchmark_imem.hex",
  parameter string p_dmem_init = "/home/jsaussereau/Documents/bdxrcg/AsteRISC/AsteRISC-firmware/hex/dhrystone_benchmark_dmem.hex",

  //RISC-V extensions
  parameter p_ext_rve         = 0,           //! use RV32E extension (reduces the integer register count to 16)
  parameter p_ext_rvc         = 0,           //! use RV32C extension (compressed instructions)
  parameter p_ext_rvm         = 0,           //! use RV32M extension (multiplication and division)
  parameter p_ext_rvzicsr     = 1,           //! use RV32Zicsr extension (control and status registers)
  parameter p_ext_custom      = 0,           //! use custom extension
  
  parameter p_counters        = 1,           //! use counters (mcycle, minstret, mtime)

  parameter p_mul_fast        = 0,           //! fast mul
  parameter p_mul_1_cycle     = 0,           //! one cycle mul

  parameter p_pipeline        = 0,           //! implement a pipepined architecure

  // pipeline settings:   
  parameter p_stage_IF        = 1,           //! use a register barrier after IF stage
  parameter p_stage_IC        = 0,           //! use a register barrier after IC stage
  parameter p_stage_ID        = 1,           //! use a register barrier after ID stage
  parameter p_stage_RF        = 0,           //! use a register barrier after RF stage
  parameter p_stage_EX        = 1,           //! use a register barrier after EX stage
  parameter p_stage_MA        = 1,           //! use a register barrier after MA stage
  parameter p_stage_WB        = 1,           //! use a register barrier after WB stage

  // non pipeline settings:   
  parameter p_prefetch_buf    = 0,           //! use a prefetch buffer
  parameter p_decode_buf      = 1,           //! add buffers to decode stage outputs
  parameter p_rf_sp           = 0,           //! register file is a single port ram
  parameter p_rf_read_buf     = 0,           //! register file has synchronous read
  parameter p_mem_buf         = 0,           //! add buffers to mem stage inputs
  parameter p_wb_buf          = 1,           //! add buffers to write back stage inputs
  parameter p_branch_buf      = 0,           //! add buffers to alu comp outputs (+1 cycle for conditionnal branches)

  //implementation customization (not functionnal yet...)
  parameter p_imem_sram       = 1,            //! implement instruction memory in sram?
  parameter p_dmem_sram       = 1,            //! implement data memory in sram?
  parameter p_rf_sram         = 0,            //! implement regfile in sram? warning: 'p_rf_read_buf' must be '1'

  //security
  parameter p_wait_for_ack    = 0             //! wait for data bus acknowledgement. warning: gets stuck if addressing a non responding memory + reduces fax frequency when activated
  
  /* verilator public_off*/
)(
  // Global
  input  wire                   i_xtal_p,     //! Pin 13: XTAL positive
  input  wire                   i_xtal_n,     //! Pin 14: XTAL negative
  input  wire                   i_xclk,       //! Pin 28: External Clock
  input  wire                   i_xrst,       //! Pin 29: External Reset

  // Standalone SPI
  input  wire                   i_sck,        //! Pin 18: SPI Clock
  input  wire                   i_csb,        //! Pin 17: SPI Chip Select
  input  wire                   i_sdi,        //! Pin 16: SPI Data In
  output wire                   o_sdo,        //! Pin 15: SPI Data Out

  // QSPI flash
  output wire                   o_flash_clk,  //! Pin 21: QSPI Clock
  output wire                   o_flash_csb,  //! Pin 22: QSPI Chip Select
  inout  wire                   io_flash0,    //! Pin 23: QSPI Bidirectionnal Data IO 0
  inout  wire                   io_flash1,    //! Pin 24: QSPI Bidirectionnal Data IO 1
  inout  wire                   io_flash2,    //! Pin 25: QSPI Bidirectionnal Data IO 2
  inout  wire                   io_flash3,    //! Pin 26: QSPI Bidirectionnal Data IO 3

  // GPIOs
  inout  wire [p_num_gpios-1:0] io_gpio       //! General Purpose IOs
);


// Clock
wire                    w_i_xclk;
wire                    w_i_pll_clk;
wire                    w_i_pll_locked;
wire                    w_o_clk;
wire                    w_o_rst;
wire                    w_o_xtal_en;
wire                    w_o_pll_vco_en;
wire                    w_o_pll_cp_en;
wire  [ 3: 0]           w_o_pll_trim;
wire                    w_i_xtal_p;
wire                    w_i_xtal_n;

// Reset
wire                    w_i_xrst;
wire                    w_i_por;

// Power 
wire                    w_o_regulator_en;

// SPI
wire                    w_i_sck;
wire                    w_i_sdi;
wire                    w_o_sdo;
wire                    w_o_sdo_en;
wire                    w_i_csb;

// QSPI FLASH   
wire                    w_o_flash_clk;
wire                    w_o_flash_clk_out_en;
wire                    w_o_flash_csb;
wire                    w_o_flash_csb_out_en; 
wire                    w_i_flash_io0_din;
wire                    w_i_flash_io1_din;
wire                    w_i_flash_io2_din;
wire                    w_i_flash_io3_din;
wire                    w_o_flash_io0_dout;
wire                    w_o_flash_io1_dout;
wire                    w_o_flash_io2_dout;
wire                    w_o_flash_io3_dout;
wire                    w_o_flash_io0_out_en;
wire                    w_o_flash_io1_out_en;
wire                    w_o_flash_io2_out_en;
wire                    w_o_flash_io3_out_en;

// GPIOs
wire  [p_num_gpios-1:0] w_o_gpio_out;
wire  [p_num_gpios-1:0] w_i_gpio_in;
wire  [p_num_gpios-1:0] w_o_gpio_pullup;
wire  [p_num_gpios-1:0] w_o_gpio_pulldown;
wire  [p_num_gpios-1:0] w_o_gpio_out_en;


/******************
       Power
******************/

wire  vdd_io;
wire  vdd_co;
wire  vss;  

wire  netTie1;
wire  netTie0;

`KEEP_HIERARCHY
wrap_nettie nettie (
  .vdd_co             ( vdd_co               ),
  .vdd_io             ( vdd_io               ),
  .vss                ( vss                  ),
  .netTie0            ( netTie0              ),
  .netTie1            ( netTie1              )
);


/******************
     SoC main
******************/

`KEEP_HIERARCHY
//soc_full #(
soc_min #(
  .p_reset_vector     ( p_reset_vector       ),
  .p_imem_init        ( p_imem_init          ),
  .p_dmem_init        ( p_dmem_init          ),
  .p_num_gpios        ( p_num_gpios          ),
  .p_manufacturer_id  ( p_manufacturer_id    ),
  .p_product_id       ( p_product_id         ),
  .p_imem_depth_pw2   ( p_imem_depth_pw2     ),
  .p_dmem_depth_pw2   ( p_dmem_depth_pw2     ),
  .p_ext_rvc          ( p_ext_rvc            ),
  .p_ext_rve          ( p_ext_rve            ),
  .p_ext_rvm          ( p_ext_rvm            ),
  .p_ext_rvzicsr      ( p_ext_rvzicsr        ),
  .p_ext_custom       ( p_ext_custom         ),
  .p_counters         ( p_counters           ),
  .p_mul_fast         ( p_mul_fast           ),
  .p_mul_1_cycle      ( p_mul_1_cycle        ),
  .p_pipeline         ( p_pipeline           ),
  .p_stage_IF         ( p_stage_IF           ),
  .p_stage_IC         ( p_stage_IC           ),
  .p_stage_ID         ( p_stage_ID           ),
  .p_stage_RF         ( p_stage_RF           ),
  .p_stage_EX         ( p_stage_EX           ),
  .p_stage_MA         ( p_stage_MA           ),
  .p_stage_WB         ( p_stage_WB           ),
  .p_prefetch_buf     ( p_prefetch_buf       ),
  .p_decode_buf       ( p_decode_buf         ),
  .p_rf_sp            ( p_rf_sp              ),
  .p_rf_read_buf      ( p_rf_read_buf        ),
  .p_branch_buf       ( p_branch_buf         ),
  .p_mem_buf          ( p_mem_buf            ),
  .p_wb_buf           ( p_wb_buf             ),
  .p_wait_for_ack     ( p_wait_for_ack       )
) soc_top_level ( 
  .i_xclk             ( w_i_xclk             ),
  .i_pll_clk          ( w_i_pll_clk          ),
  .i_pll_locked       ( w_i_pll_locked       ),
  .o_pll_vco_en       ( w_o_pll_vco_en       ),
  .o_pll_cp_en        ( w_o_pll_cp_en        ),
  .o_pll_trim         ( w_o_pll_trim         ),
  .o_xtal_en          ( w_o_xtal_en          ),
  .o_clk              ( w_o_clk              ),
  .o_rst              ( w_o_rst              ),
  .o_regulator_en     ( w_o_regulator_en     ),
  .i_xrst             ( w_i_xrst             ),
  .i_por              ( w_i_por              ),

  .i_sck              ( w_i_sck              ),
  .i_sdi              ( w_i_sdi              ),
  .o_sdo              ( w_o_sdo              ),
  .o_sdo_en           ( w_o_sdo_en           ),
  .i_csb              ( w_i_csb              ),
  .o_flash_clk        ( w_o_flash_clk        ),
  .o_flash_clk_out_en ( w_o_flash_clk_out_en ),
  .o_flash_csb        ( w_o_flash_csb        ),
  .o_flash_csb_out_en ( w_o_flash_csb_out_en ),
  .i_flash_io0_din    ( w_i_flash_io0_din    ),
  .i_flash_io1_din    ( w_i_flash_io1_din    ),
  .i_flash_io2_din    ( w_i_flash_io2_din    ),
  .i_flash_io3_din    ( w_i_flash_io3_din    ),
  .o_flash_io0_dout   ( w_o_flash_io0_dout   ),
  .o_flash_io1_dout   ( w_o_flash_io1_dout   ),
  .o_flash_io2_dout   ( w_o_flash_io2_dout   ),
  .o_flash_io3_dout   ( w_o_flash_io3_dout   ),
  .o_flash_io0_out_en ( w_o_flash_io0_out_en ),
  .o_flash_io1_out_en ( w_o_flash_io1_out_en ),
  .o_flash_io2_out_en ( w_o_flash_io2_out_en ),
  .o_flash_io3_out_en ( w_o_flash_io3_out_en ),
  .o_gpio_out         ( w_o_gpio_out         ),
  .i_gpio_in          ( w_i_gpio_in          ),
  .o_gpio_pullup      ( w_o_gpio_pullup      ),
  .o_gpio_pulldown    ( w_o_gpio_pulldown    ),
  .o_gpio_out_en      ( w_o_gpio_out_en      )
);


/******************
    Pad settings
******************/

wire  pad_input;
wire  pad_output;
wire  pad_pullup_on;
wire  pad_pullup_off;
wire  pad_pulldown_on;
wire  pad_pulldown_off;

`KEEP_HIERARCHY
soc_pad_settings pad_settings (
  .netTie0         ( netTie0             ),
  .netTie1         ( netTie1             ),
  .pad_input       ( pad_input           ),
  .pad_output      ( pad_output          ),
  .pad_pullup_on   ( pad_pullup_on       ),
  .pad_pullup_off  ( pad_pullup_off      ),
  .pad_pulldown_on ( pad_pulldown_on     ),
  .pad_pulldown_off( pad_pulldown_off    )
);


/******************
       Clock
******************/

`KEEP_HIERARCHY
wrap_pll pll (
  .i_rst          ( w_i_por              ), //TODO: use global reset ?
  .i_xtal_p       ( w_i_xtal_p           ),
  .i_xtal_n       ( w_i_xtal_n           ),
  .o_clk          ( w_i_pll_clk          ),
  .o_locked       ( w_i_pll_locked       )
);

`KEEP_HIERARCHY
wrap_clock_pad P_i_xclk (
  .io_pad         ( i_xclk               ),
  .o_pad_in       ( w_i_xclk             ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);

`KEEP_HIERARCHY
wrap_input_pad P_i_xtal_p (
  .io_pad         ( i_xtal_p             ),
  .o_pad_in       ( w_i_xtal_p           ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);

`KEEP_HIERARCHY
wrap_input_pad P_i_xtal_n (
  .io_pad         ( i_xtal_n             ),
  .o_pad_in       ( w_i_xtal_n           ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);




/******************
        SPI
******************/

`KEEP_HIERARCHY
wrap_input_pad P_i_sck (
  .io_pad         ( i_sck                ),
  .o_pad_in       ( w_i_sck              ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);

`KEEP_HIERARCHY
wrap_input_pad P_i_sdi (
  .io_pad         ( i_sdi                ),
  .o_pad_in       ( w_i_sdi              ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);

`KEEP_HIERARCHY
wrap_output_pad P_o_sdo (
  .io_pad         ( o_sdo                ),
  .i_pad_out      ( w_o_sdo              ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);

`KEEP_HIERARCHY
wrap_input_pad P_i_csb (
  .io_pad         ( i_csb                ),
  .o_pad_in       ( w_i_csb              ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);


/******************
       FLASH
******************/

`KEEP_HIERARCHY
wrap_output_pad P_o_flash_csb (
  .io_pad         ( o_flash_csb          ),
  .i_pad_out      ( w_o_flash_csb        ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);

`KEEP_HIERARCHY
wrap_output_pad P_o_flash_clk (
  .io_pad         ( o_flash_clk          ),
  .i_pad_out      ( w_o_flash_clk        ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);

`KEEP_HIERARCHY
wrap_io_pad P_io_flash0(
  .io_pad         ( io_flash0            ),
  .o_pad_in       ( w_i_flash_io0_din    ),
  .i_pad_out      ( w_o_flash_io0_dout   ),
  .i_pad_out_en   ( w_o_flash_io0_out_en ),
  .i_pad_pullup   ( pad_pullup_off       ),
  .i_pad_pulldown ( pad_pulldown_off     ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);

`KEEP_HIERARCHY
wrap_io_pad P_io_flash1(
  .io_pad         ( io_flash1            ),
  .o_pad_in       ( w_i_flash_io1_din    ),
  .i_pad_out      ( w_o_flash_io1_dout   ),
  .i_pad_out_en   ( w_o_flash_io1_out_en ),
  .i_pad_pullup   ( pad_pullup_off       ),
  .i_pad_pulldown ( pad_pulldown_off     ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);

`KEEP_HIERARCHY
wrap_io_pad P_io_flash2(
  .io_pad         ( io_flash2            ),
  .o_pad_in       ( w_i_flash_io2_din    ),
  .i_pad_out      ( w_o_flash_io2_dout   ),
  .i_pad_out_en   ( w_o_flash_io2_out_en ),
  .i_pad_pullup   ( pad_pullup_off       ),
  .i_pad_pulldown ( pad_pulldown_off     ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);

`KEEP_HIERARCHY
wrap_io_pad P_io_flash3(
  .io_pad         ( io_flash3            ),
  .o_pad_in       ( w_i_flash_io3_din    ),
  .i_pad_out      ( w_o_flash_io3_dout   ),
  .i_pad_out_en   ( w_o_flash_io3_out_en ),
  .i_pad_pullup   ( pad_pullup_off       ),
  .i_pad_pulldown ( pad_pulldown_off     ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);


/******************
      Reset
******************/

`KEEP_HIERARCHY
wrap_input_pad P_i_xrst (
  .io_pad         ( i_xrst               ),
  .o_pad_in       ( w_i_xrst             ),
  .netTie0        ( netTie0              ),
  .netTie1        ( netTie1              ),
  .vdd_io         ( vdd_io               ),
  .vdd_co         ( vdd_co               ),
  .vss            ( vss                  )
);

`KEEP_HIERARCHY
wrap_por por(
 .i_clk           ( w_o_clk             ),
 .o_rst           ( w_i_por             )
);

/******************
       GPIOs
******************/

genvar i;
generate
  for (i = 0 ; i < p_num_gpios ; i = i+1) begin : P_io_GPIO
    `KEEP_HIERARCHY
    wrap_io_pad gen (
      .io_pad         ( io_gpio          [i] ),
      .o_pad_in       ( w_i_gpio_in      [i] ),
      .i_pad_out      ( w_o_gpio_out     [i] ),
      .i_pad_out_en   ( w_o_gpio_out_en  [i] ),
      .i_pad_pullup   ( w_o_gpio_pullup  [i] ),
      .i_pad_pulldown ( w_o_gpio_pulldown[i] ),
      .netTie0        ( netTie0              ),
      .netTie1        ( netTie1              ),
      .vdd_io         ( vdd_io               ),
      .vdd_co         ( vdd_co               ),
      .vss            ( vss                  )
    );
  end 
endgenerate

endmodule


`endif // __SOC_FULL__
