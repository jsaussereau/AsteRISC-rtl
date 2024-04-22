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

//! CPU full SoC

`ifndef __SOC_FULL__
`define __SOC_FULL__

`ifdef VIVADO
  `include "soc_config.sv"
  `include "packages/pck_soc_control.sv"
  `include "packages/pck_memory_map.sv"
  `include "../core/packages/pck_mem_bus.sv"
`else
  `include "soc/soc_config.sv"
  `include "soc/packages/pck_soc_control.sv"
  `include "soc/packages/pck_memory_map.sv"
  `include "core/packages/pck_mem_bus.sv"
`endif
 
module soc_full 
  import pck_memory_map::*;
  import pck_soc_control::*;
#(
  parameter p_reset_vector    = 32'hf0000000,
  parameter p_imem_init       = "firmware/imem.hex",
  parameter p_dmem_init       = "firmware/dmem.hex",
  parameter p_num_gpios       = 24,
  parameter p_manufacturer_id = 12'h456,
  parameter p_product_id      = 8'h01,

  parameter p_imem_depth_pw2  = 14,
  parameter p_dmem_depth_pw2  = 13,

  parameter p_ext_rvc         = 0,
  parameter p_ext_rve         = 0,
  parameter p_ext_rvm         = 0,
  parameter p_ext_rvzicsr     = 0,
  parameter p_counters        = 0,
  parameter p_mul_fast        = 0,
  parameter p_mul_1_cycle     = 0,
  parameter p_prefetch_buf    = 0,
  parameter p_decode_buf      = 0,
  parameter p_rf_sp           = 0,
  parameter p_rf_read_buf     = 0,
  parameter p_branch_buf      = 0,
  parameter p_mem_buf         = 0,
  parameter p_wb_buf          = 0,
  parameter p_wait_for_ack    = 0
)(
  // Clock
  input  wire                    i_xclk,              //! external clock
  input  wire                    i_pll_clk,           //! PLL clock
  input  wire                    i_pll_locked,        //! high when pll is locked
  output logic                   o_pll_vco_en,        //! 
  output logic                   o_pll_cp_en,         //! 
  output logic [ 3: 0]           o_pll_trim,          //! 
  output logic                   o_xtal_en,           //! XTAL enable
  output logic                   o_clk,               //! 

  // Reset
  input  wire                    i_xrst,              //! external reset
  output logic                   o_rst,               //! 
  input  wire                    i_por,               //! POR (Power On Reset)
  
  // Power
  output logic                   o_regulator_en,      //! power regulator enable

  // Memory buses
  //mem_bus.cpu_side               dbus,                //! data bus
  //mem_bus.cpu_side               ibus,                //! instruction bus

  // Standalone SPI
  input  wire                    i_sck,               //! SPI clock
  input  wire                    i_sdi,               //! SPI data in
  output logic                   o_sdo,               //! SPI data out
  output logic                   o_sdo_en,            //! SPI data output enable
  input  wire                    i_csb,               //! SPI select

  // SPI flash memory
  output logic                   o_flash_clk,
  output logic                   o_flash_clk_out_en,
  output logic                   o_flash_csb,  
  output logic                   o_flash_csb_out_en,  
  input  wire                    i_flash_io0_din,
  input  wire                    i_flash_io1_din,
  input  wire                    i_flash_io2_din,
  input  wire                    i_flash_io3_din,
  output logic                   o_flash_io0_dout,
  output logic                   o_flash_io1_dout,
  output logic                   o_flash_io2_dout,
  output logic                   o_flash_io3_dout,
  output logic                   o_flash_io0_out_en,
  output logic                   o_flash_io1_out_en,
  output logic                   o_flash_io2_out_en,
  output logic                   o_flash_io3_out_en,

  // GPIO  
  input  wire  [p_num_gpios-1:0] i_gpio_in,         //! gpio read 
  output logic [p_num_gpios-1:0] o_gpio_out,        //! gpio write
  output logic [p_num_gpios-1:0] o_gpio_out_en,     //! gpio output enable
  output logic [p_num_gpios-1:0] o_gpio_pullup,     //! gpio pullup enable
  output logic [p_num_gpios-1:0] o_gpio_pulldown    //! gpio pulldown enable
);

  // data bus
  //mem_bus       dbus();
  logic [31:0] dbus_addr;       //! data bus address
  logic [ 3:0] dbus_be;         //! data bus write byte enable
  logic        dbus_wr_en;      //! data bus write enable
  logic [31:0] dbus_wr_data;    //! data bus write data
  logic        dbus_rd_en;      //! data bus read enable
  logic [31:0] dbus_rd_data;    //! data bus read data
  logic        dbus_busy;       //! data bus busy
  logic        dbus_ack;        //! data bus transfer acknowledge

  // instruction bus
  //mem_bus       ibus();
  logic [31:0] ibus_addr;       //! instruction bus address
  logic [ 3:0] ibus_be;         //! instruction bus write byte enable
  logic        ibus_wr_en;      //! instruction bus write enable
  logic [31:0] ibus_wr_data;    //! instruction bus write data
  logic        ibus_rd_en;      //! instruction bus read enable
  logic [31:0] ibus_rd_data;    //! instruction bus read data
  logic        ibus_busy;       //! instruction bus busy
  logic        ibus_ack;        //! instruction bus transfer acknowledge

  // coprocessor
  logic [31: 0] alu_op_a;  
  logic [31: 0] alu_op_b;  
  logic [31: 0] copro_out0;
  logic [31: 0] copro_out1;
  logic [31: 0] copro_out2;
  logic [ 2: 0] copro_valid;

  // bridge
  typedef enum integer {
    DMEM_ID,
    //DMEM2_ID,
    //QSPI_FLASH_ID,
    IOF_SEL_ID,
    GPIO_ID,
    PWM0_ID,
    UART0_ID,
    //I2C_MASTER0_ID,
    //TIMER0_ID,
    DEBUG_ID, 
    NUM_PERIPH,
    NO_PERIPH     
  } bridge_id_e;

  logic [31:0] br_base [NUM_PERIPH-1:0];
  assign br_base[DMEM_ID       ] = DMEM_ADDR;
  //assign br_base[DMEM2_ID      ] = DMEM2_ADDR;
  //assign br_base[QSPI_FLASH_ID ] = SPI_FLASH_ADDR;
  assign br_base[IOF_SEL_ID    ] = IOF_ADDR;
  assign br_base[GPIO_ID       ] = GPIO_ADDR;
  assign br_base[PWM0_ID       ] = PWM0_ADDR;
  assign br_base[UART0_ID      ] = UART0_ADDR;
  //assign br_base[I2C_MASTER0_ID] = I2C_MASTER0_ADDR;
  //assign br_base[TIMER0_ID     ] = TIMER0_ADDR;
  assign br_base[DEBUG_ID      ] = DEBUG_ADDR;

  logic [31:0] br_mask [NUM_PERIPH-1:0];
  assign br_mask[DMEM_ID       ] = DMEM_ADDR_MASK;
  //assign br_mask[DMEM2_ID      ] = DMEM2_ADDR_MASK;
  //assign br_mask[QSPI_FLASH_ID ] = SPI_FLASH_ADDR_MASK;
  assign br_mask[IOF_SEL_ID    ] = IOF_ADDR_MASK;
  assign br_mask[GPIO_ID       ] = GPIO_ADDR_MASK;
  assign br_mask[PWM0_ID       ] = PWM0_ADDR_MASK;
  assign br_mask[UART0_ID      ] = UART0_ADDR_MASK;
  //assign br_mask[I2C_MASTER0_ID] = I2C_MASTER0_ADDR_MASK;
  //assign br_mask[TIMER0_ID     ] = TIMER0_ADDR_MASK;
  assign br_mask[DEBUG_ID      ] = DEBUG_ADDR_MASK;

  logic [31:0] br_addr     [NUM_PERIPH-1:0];
  logic [ 3:0] br_be       [NUM_PERIPH-1:0];
  logic        br_wr_en    [NUM_PERIPH-1:0];
  logic [31:0] br_wr_data  [NUM_PERIPH-1:0];
  logic        br_rd_en    [NUM_PERIPH-1:0];
  logic [31:0] br_rd_data  [NUM_PERIPH-1:0];
  logic        br_busy     [NUM_PERIPH-1:0];
  logic        br_ack      [NUM_PERIPH-1:0];

  // gpio
  logic [p_num_gpios-1:0] gpio_in;
  logic [p_num_gpios-1:0] gpio_out;
  logic [p_num_gpios-1:0] gpio_out_en;
  logic [p_num_gpios-1:0] gpio_pullup;
  logic [p_num_gpios-1:0] gpio_pulldown;
  
  // pwm
  logic [ 3:0] pwm0_out;
  logic [ 3:0] pwm0_irq;
    
  // uart
  logic uart0_rx;
  logic uart0_tx;

  // CSR
  logic [63:0] mcycle;
  logic [63:0] mtime;

  /******************
          CLK
  ******************/

  logic        clk;
  sel_clk_e    clk_sel;
/*
  always_comb begin
    case (clk_sel)
      clk_pll      : clk = i_pll_clk;
      clk_external : clk = i_xclk;
    endcase
  end
*/
  assign clk = i_pll_clk;
  assign o_clk = clk;


  /******************
          RST
  ******************/

  logic        rst;
  logic        spi_rst;
  sel_rst_e    rst_sel;
  logic        sleep;

  //TODO: 
  // standalone reset = i_xrst | i_por
  // global reset = i_xrst | i_por | spi_rst

  //assign pll_rst = (clk_sel == clk_external && !i_pll_locked) ? 1'b1 : 1'b0;
/*  
  always_comb begin
    case (rst_sel)
      rst_por      : rst = i_por;
      rst_external : rst = i_xrst;
      rst_spi      : rst = spi_rst;
      rst_all      : rst = i_xrst | i_por;
    endcase
  end
*/
  assign rst = i_xrst | i_por;
  assign o_rst = rst;

  assign sleep = ~i_pll_locked;

  /******************
        CPU CORE
  ******************/

  `KEEP_HIERARCHY
  cpu_core #(
    .p_reset_vector ( p_reset_vector             ),
    .p_ext_rvc      ( p_ext_rvc                  ),
    .p_ext_rve      ( p_ext_rve                  ),
    .p_ext_rvm      ( p_ext_rvm                  ),
    .p_ext_rvzicsr  ( p_ext_rvzicsr              ),
    .p_counters     ( p_counters                 ),
    .p_mul_fast     ( p_mul_fast                 ),
    .p_mul_1_cycle  ( p_mul_1_cycle              ),
    .p_prefetch_buf ( p_prefetch_buf             ),
    .p_decode_buf   ( p_decode_buf               ),
    .p_rf_sp        ( p_rf_sp                    ),
    .p_rf_read_buf  ( p_rf_read_buf              ),
    .p_branch_buf   ( p_branch_buf               ),
    .p_mem_buf      ( p_mem_buf                  ),
    .p_wb_buf       ( p_wb_buf                   ),
    .p_wait_for_ack ( p_wait_for_ack             )
  ) cpu (
    .i_clk          ( clk                        ),
    .i_rst          ( rst                        ),
    .i_sleep        ( sleep                      ),
    .i_mcycle       ( mcycle                     ),
    .i_mtime        ( mtime                      ),
    //.ibus           ( ibus                       ),
    .ibus_addr      ( ibus_addr                  ),
    .ibus_be        ( ibus_be                    ),
    .ibus_wr_en     ( ibus_wr_en                 ),
    .ibus_wr_data   ( ibus_wr_data               ),
    .ibus_rd_en     ( ibus_rd_en                 ),
    .ibus_rd_data   ( ibus_rd_data               ),
    .ibus_busy      ( ibus_busy                  ),
    .ibus_ack       ( ibus_ack                   ),
    //.dbus           ( dbus                       ),
    .dbus_addr      ( dbus_addr                  ),
    .dbus_be        ( dbus_be                    ),
    .dbus_wr_en     ( dbus_wr_en                 ),
    .dbus_wr_data   ( dbus_wr_data               ),
    .dbus_rd_en     ( dbus_rd_en                 ),
    .dbus_rd_data   ( dbus_rd_data               ),
    .dbus_busy      ( dbus_busy                  ),
    .dbus_ack       ( dbus_ack                   ),
    .o_alu_op_a     ( alu_op_a                   ),
    .o_alu_op_b     ( alu_op_b                   ),
    .i_copro_out0   ( copro_out0                 ),
    .i_copro_out1   ( copro_out1                 ),
    .i_copro_out2   ( copro_out2                 ),
    .i_copro_valid  ( copro_valid                )
  );


  /******************
   Instruction Memory
  ******************/

  `KEEP_HIERARCHY
  wrap_ram_imem #(
    .p_addr_base    ( IMEM_ADDR                  ),
    .p_addr_mask    ( IMEM_ADDR_MASK             ),
    .p_init_file    ( p_imem_init                ),
    .p_init_val     ( 32'h00000003               ),
    .p_depth_pw2    ( p_imem_depth_pw2           )
  ) imem (
    .i_clk          ( clk                        ),
    .i_rst          ( rst                        ),
    .i_addr         ( ibus_addr[31:2]            ),
    .i_be           ( ibus_be                    ),
    .i_wr_en        ( ibus_wr_en                 ),
    .i_wr_data      ( ibus_wr_data               ),
    .i_rd_en        ( ibus_rd_en                 ),
    .o_rd_data      ( ibus_rd_data               ),
    .o_busy         ( ibus_busy                  ),
    .o_ack          ( ibus_ack                   ) 
  );


  /******************
      Data Memory
  ******************/

  `KEEP_HIERARCHY
  wrap_ram_dmem #(
    .p_addr_base    ( DMEM_ADDR                  ),
    .p_addr_mask    ( DMEM_ADDR_MASK             ),
    .p_init_file    ( p_dmem_init                ),
    .p_init_val     ( 32'h00000000               ),
    .p_depth_pw2    ( p_dmem_depth_pw2           )
  ) dmem (
    .i_clk          ( clk                        ),
    .i_rst          ( rst                        ),
    .i_addr         ( br_addr    [DMEM_ID][31:2] ),
    .i_be           ( br_be      [DMEM_ID]       ),
    .i_wr_en        ( br_wr_en   [DMEM_ID]       ),
    .i_wr_data      ( br_wr_data [DMEM_ID]       ),
    .i_rd_en        ( br_rd_en   [DMEM_ID]       ),
    .o_rd_data      ( br_rd_data [DMEM_ID]       ),
    .o_busy         ( br_busy    [DMEM_ID]       ),
    .o_ack          ( br_ack     [DMEM_ID]       ) 
  );


  /******************
    Data bus bridge
  ******************/

  `KEEP_HIERARCHY
  soc_bridge #(
    .p_num_masters  ( NUM_PERIPH                 )
  ) bridge (
    .i_clk          ( clk                        ),
    //.dbus           ( dbus                       ),
    .dbus_addr      ( dbus_addr                  ),
    .dbus_be        ( dbus_be                    ),
    .dbus_wr_en     ( dbus_wr_en                 ),
    .dbus_wr_data   ( dbus_wr_data               ),
    .dbus_rd_en     ( dbus_rd_en                 ),
    .dbus_rd_data   ( dbus_rd_data               ),
    .dbus_busy      ( dbus_busy                  ),
    .dbus_ack       ( dbus_ack                   ),
    .i_base         ( br_base                    ),
    .i_mask         ( br_mask                    ),
    .o_addr         ( br_addr                    ),
    .o_be           ( br_be                      ),
    .o_wr_en        ( br_wr_en                   ),
    .o_wr_data      ( br_wr_data                 ),
    .o_rd_en        ( br_rd_en                   ),
    .i_rd_data      ( br_rd_data                 ),
    .i_busy         ( br_busy                    ),
    .i_ack          ( br_ack                     ) 
  );


  /******************
         GPIO
  ******************/

  `KEEP_HIERARCHY
  perif_gpio #(
    .p_num_gpios    ( p_num_gpios                   )
  ) gpio (
    .i_clk          ( clk                           ),
    .i_rst          ( rst                           ),
    .i_addr         ( br_addr    [GPIO_ID][9:2]     ),
    .i_be           ( br_be      [GPIO_ID]          ),
    .i_wr_en        ( br_wr_en   [GPIO_ID]          ),
    .i_wr_data      ( br_wr_data [GPIO_ID]          ),
    .i_rd_en        ( br_rd_en   [GPIO_ID]          ),
    .o_rd_data      ( br_rd_data [GPIO_ID]          ),
    .o_busy         ( br_busy    [GPIO_ID]          ),
    .o_ack          ( br_ack     [GPIO_ID]          ),

    .i_gpio_in      ( gpio_in                       ),
    .o_gpio_out     ( gpio_out                      ),
    .o_gpio_out_en  ( gpio_out_en                   ),
    .o_gpio_pullup  ( gpio_pullup                   ),
    .o_gpio_pulldown( gpio_pulldown                 )
  );


  /******************
          PWM
  ******************/

  `KEEP_HIERARCHY
  perif_pwm #(
    .p_cmp_width    ( 16                            )
  ) pwm0 (
    .i_clk          ( clk                           ),
    .i_rst          ( rst                           ),
    .i_addr         ( br_addr    [PWM0_ID][9:2]     ),
    .i_be           ( br_be      [PWM0_ID]          ),
    .i_wr_en        ( br_wr_en   [PWM0_ID]          ),
    .i_wr_data      ( br_wr_data [PWM0_ID]          ),
    .i_rd_en        ( br_rd_en   [PWM0_ID]          ),
    .o_rd_data      ( br_rd_data [PWM0_ID]          ),
    .o_busy         ( br_busy    [PWM0_ID]          ),
    .o_ack          ( br_ack     [PWM0_ID]          ),

    .o_pwm_irq      ( pwm0_irq                      ),
    .o_pwm_out      ( pwm0_out                      )
  ); 


  /******************
          UART
  ******************/

  `KEEP_HIERARCHY
  perif_uart uart0(
    .i_clk          ( clk                        ),
    .i_rst          ( rst                        ),
    .i_addr         ( br_addr    [UART0_ID][9:2] ),
    .i_be           ( br_be      [UART0_ID]      ),
    .i_wr_en        ( br_wr_en   [UART0_ID]      ),
    .i_wr_data      ( br_wr_data [UART0_ID]      ),
    .i_rd_en        ( br_rd_en   [UART0_ID]      ),
    .o_rd_data      ( br_rd_data [UART0_ID]      ),
    .o_busy         ( br_busy    [UART0_ID]      ),
    .o_ack          ( br_ack     [UART0_ID]      ),
    .i_uart_rx      ( uart0_rx                   ),
    .o_uart_tx      ( uart0_tx                   ) 
  );

  
  /******************
         DEBUG
  ******************/
  
  `KEEP_HIERARCHY
  /*perif_debug debug (
    .i_clk          ( clk                        ),
    .i_rst          ( rst                        ),
    .i_addr         ( br_addr    [DEBUG_ID][9:2] ),
    .i_be           ( br_be      [DEBUG_ID]      ),
    .i_wr_en        ( br_wr_en   [DEBUG_ID]      ),
    .i_wr_data      ( br_wr_data [DEBUG_ID]      ),
    .i_rd_en        ( br_rd_en   [DEBUG_ID]      ),
    .o_rd_data      ( br_rd_data [DEBUG_ID]      ),
    .o_busy         ( br_busy    [DEBUG_ID]      ),
    .o_ack          ( br_ack     [DEBUG_ID]      ) 
  );*/
  soc_sp_ram #(
    .p_addr_base    ( DEBUG_ADDR                  ),
    .p_addr_mask    ( DEBUG_ADDR_MASK             ),
    .p_init_mem     ( 0                           ),
    .p_depth_pw2    ( 3                           )
  ) debug (
    .i_clk          ( clk                         ),
    .i_rst          ( rst                         ),
    .i_addr         ( br_addr    [DEBUG_ID][31:2] ),
    .i_be           ( br_be      [DEBUG_ID]       ),
    .i_wr_en        ( br_wr_en   [DEBUG_ID]       ),
    .i_wr_data      ( br_wr_data [DEBUG_ID]       ),
    .i_rd_en        ( br_rd_en   [DEBUG_ID]       ),
    .o_rd_data      ( br_rd_data [DEBUG_ID]       ),
    .o_busy         ( br_busy    [DEBUG_ID]       ),
    .o_ack          ( br_ack     [DEBUG_ID]       ) 
  );


  /******************
       FPGA core
  ******************/

  logic [15:0] fpga_io_out;
  logic [15:0] fpga_io_in;
  logic [15:0] fpga_io_out_en;
  logic        fpga_config_rx;

  localparam p_efpga = 0;

  generate
    if (p_efpga) begin
      eFPGA_top efpga_core(
        .i_clk        ( clk                        ),
        .i_op_a       ( alu_op_a                   ),
        .i_op_b       ( alu_op_b                   ),
        .o_efpga_out0 ( copro_out0                 ),
        .o_efpga_out1 ( copro_out1                 ),
        .o_efpga_out2 ( copro_out2                 ),

        // External USER ports 
        .o_io_in      ( fpga_io_out                ),
        .o_io_out_en  ( fpga_io_out_en             ),
        .i_io_out     ( fpga_io_in                 ),
      
        // CPU configuration ports
        .i_wr_strobe  ( 1'b0                       ),
        .i_wr_data    ( 32'b0                      ),
      
        // UART configuration ports
        .i_config_rx  ( fpga_config_rx             ),
        .o_com_active (                            ),
        .o_rx_led     (                            )
      );      
    end else begin
      //assign fpga_io_out    = '{32'b0};
      //assign fpga_io_out_en = '{32'b0};
      assign copro_out0     = 32'bz;
      assign copro_out1     = 32'bz;
      assign copro_out2     = 32'bz;
    end
  endgenerate


  /******************
      IO Function
  ******************/

  `KEEP_HIERARCHY
  perif_io_function_sel #(
    .p_num_gpios           ( p_num_gpios                      )
  ) iof_sel (
    .i_clk                 ( clk                              ),
    .i_rst                 ( rst                              ),
    .i_addr                ( br_addr    [IOF_SEL_ID][3:2]     ),
    .i_be                  ( br_be      [IOF_SEL_ID]          ),
    .i_wr_en               ( br_wr_en   [IOF_SEL_ID]          ),
    .i_wr_data             ( br_wr_data [IOF_SEL_ID]          ),
    .i_rd_en               ( br_rd_en   [IOF_SEL_ID]          ),
    .o_rd_data             ( br_rd_data [IOF_SEL_ID]          ),
    .o_busy                ( br_busy    [IOF_SEL_ID]          ),
    .o_ack                 ( br_ack     [IOF_SEL_ID]          ),

    .i_gpio_out            ( gpio_out                         ),
    .o_gpio_in             ( gpio_in                          ),
    .i_gpio_pullup         ( gpio_pullup                      ),
    .i_gpio_pulldown       ( gpio_pulldown                    ),
    .i_gpio_out_en         ( gpio_out_en                      ),

    .i_pwm0_out            ( pwm0_out                         ),
    .o_i2c_master0_scl_in  (                                  ),
    .i_i2c_master0_scl_out ( 1'b0                             ),
    .i_i2c_master0_scl_oen ( 1'b0                             ),
    .o_i2c_master0_sda_in  (                                  ),
    .i_i2c_master0_sda_out ( 1'b0                             ),
    .i_i2c_master0_sda_oen ( 1'b0                             ),
    .i_fpga_io_out         ( fpga_io_out    [7:0]             ),
    .o_fpga_io_in          ( fpga_io_in     [7:0]             ),
    .i_fpga_io_out_en      ( fpga_io_out_en [7:0]             ),
    .o_fpga_config_rx      ( fpga_config_rx                   ),
    .o_uart0_rx            ( uart0_rx                         ),
    .i_uart0_tx            ( uart0_tx                         ),

    .i_gpio_in             ( i_gpio_in                        ),
    .o_gpio_out            ( o_gpio_out                       ),
    .o_gpio_out_en         ( o_gpio_out_en                    ),
    .o_gpio_pullup         ( o_gpio_pullup                    ),
    .o_gpio_pulldown       ( o_gpio_pulldown                  )
  );


  /******************
      SPI slave
  ******************/

  `KEEP_HIERARCHY
  perif_standalone_spi #(
    .p_manufacturer_id    ( p_manufacturer_id  ),
    .p_product_id         ( p_product_id       )
  ) spi_slave (
    .i_rst                ( rst                ),
    .i_sck                ( i_sck              ),
    .i_csb                ( i_csb              ),
    .i_sdi                ( i_sdi              ),
    .o_sdo                ( o_sdo              ),
    .o_sdo_en             ( o_sdo_en           ),
    .o_regulator_en       (  ),
    .o_xtal_en            (  ),
    .o_pll_vco_en         (  ),
    .o_pll_cp_en          (  ),
    .o_pll_trim           (  ),
    .o_clk_sel            ( clk_sel            ),
    .o_rst_sel            ( rst_sel            ),
    .o_spi_rst            ( spi_rst            ),
    .o_pass_through_rst   (  ),
    .o_pass_through_sck   (  ),
    .o_pass_through_csb   (  ),
    .o_pass_through_sdi   (  ),
    .i_pass_through_sdo   ( 1'b0 )
  );


  /******************
     Cycle Counter
  ******************/
  soc_cycle_counter cycle_counter (
    .i_clk                ( clk                ),
    .i_rst                ( rst                ),
    .i_sleep              ( sleep              ),
    .o_mcycle             ( mcycle             )
  );

endmodule

`endif // __SOC_FULL__
