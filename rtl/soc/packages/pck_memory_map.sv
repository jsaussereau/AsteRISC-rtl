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

`ifndef __PCK_MEMORY_MAP__
`define __PCK_MEMORY_MAP__

package pck_memory_map;

  parameter READ_MASK                   = 4'b0000;
  parameter WRITE_MASK_BYTE0            = 4'b0001;
  parameter WRITE_MASK_BYTE1            = 4'b0010;
  parameter WRITE_MASK_BYTE2            = 4'b0100;
  parameter WRITE_MASK_BYTE3            = 4'b1000;
  parameter WRITE_MASK_ALL              = 4'b1111;
 
 
  /******************  
       Addresses  
  ******************/  

 // quad spi flash memory 
  parameter SPI_FLASH_ADDR              = 32'h00000000;
  parameter SPI_FLASH_ADDR_MASK         = 32'hfe000000;
  parameter SPI_FLASH_CONTROL_OFFSET    = 32'h01ffffff;
 
  // io function 
  parameter IOF_ADDR                    = 32'h03000000;
  parameter IOF_ADDR_MASK               = 32'hfffff000;
  parameter IOF_SEL_OFFSET              = 2'h0;
   
  // gpio 
  parameter GPIO_ADDR                   = 32'h04000000;
  parameter GPIO_ADDR_MASK              = 32'hfffff000;
  parameter GPIO_WRITE_VALUE_OFFSET     = 8'h00;
  parameter GPIO_READ_VALUE_OFFSET      = 8'h04;
  parameter GPIO_OUTPUT_EN_OFFSET       = 8'h08;
  parameter GPIO_PULLUP_EN_OFFSET       = 8'h0C;
  parameter GPIO_PULLDOWN_EN_OFFSET     = 8'h10;
 
  // pwm 
  parameter PWM0_ADDR                   = 32'h05000000;
  parameter PWM0_ADDR_MASK              = 32'hfffff000;
  parameter PWM_CONFIG_OFFSET           = 8'h00;
  parameter PWM_COUNTER_OFFSET          = 8'h08;
  parameter PWM_SCALED_COUNTER_OFFSET   = 8'h10;
  parameter PWM_COMPARE0_OFFSET         = 8'h20;
  parameter PWM_COMPARE1_OFFSET         = 8'h24;
  parameter PWM_COMPARE2_OFFSET         = 8'h28;
  parameter PWM_COMPARE3_OFFSET         = 8'h2C;
 
  // uart 
  parameter UART0_ADDR                  = 32'h09000000;
  parameter UART0_ADDR_MASK             = 32'hfffff000;
  parameter UART_DATA_TX_OFFSET         = 8'h00;
  parameter UART_DATA_RX_OFFSET         = 8'h04;
  parameter UART_CONFIG_OFFSET          = 8'h08;
  parameter UART_STATUS_OFFSET          = 8'h0C;
 
  // i²c 
  parameter I2C_MASTER0_ADDR            = 32'h06000000;
  parameter I2C_MASTER0_ADDR_MASK       = 32'hfffff000;
  parameter I2C_DATA_TX_OFFSET          = 8'h00;
  parameter I2C_DATA_RX_OFFSET          = 8'h04;
  parameter I2C_CONFIG_OFFSET           = 8'h08;
  parameter I2C_COMMAND_OFFSET          = 8'h12;
  parameter I2C_STATUS_OFFSET           = 8'h16;

  // spi slave
  parameter SPI_SLAVE0_ADDR             = 32'h0b000000;
  parameter SPI_SLAVE0_ADDR_MASK        = 32'hfffff000;
  parameter SPI_CONFIG_OFFSET           = 8'h00;
  parameter SPI_CMP_PATTERN_OFFSET      = 8'h04;
  parameter SPI_CMP_PATTERN_MSK_OFFSET  = 8'h08;
  parameter SPI_START_PATTERN_OFFSET    = 8'h0C;
  parameter SPI_START_PATTERN_MSK_OFFSET= 8'h10;
  parameter SPI_STATUS_TX_OFFSET        = 8'h14;
  parameter SPI_STATUS_RX_OFFSET        = 8'h18;
  parameter SPI_DATA_TX_OFFSET          = 8'h1c;
  parameter SPI_DATA_TX_x4_OFFSET       = 8'h20;
  parameter SPI_DATA_RX_OFFSET          = 8'h24;
  parameter SPI_DATA_RX_x4_OFFSET       = 8'h28;
 
  // spi master 
  parameter SPI_MASTER0_ADDR            = 32'h07000000;
  parameter SPI_MASTER0_ADDR_MASK       = 32'hfffff000;
  parameter SPI_MASTER_CONFIG_OFFSET    = 8'h00;

  // timer
  parameter TIMER0_ADDR                 = 32'h08000000;
  parameter TIMER0_ADDR_MASK            = 32'hfffff000;
  parameter TIMER_CONFIG_OFFSET         = 8'h00;
  parameter TIMER_COUNTER_MSB_OFFSET    = 8'h08;
  parameter TIMER_COUNTER_LSB_OFFSET    = 8'h10;
  parameter TIMER_SCALED_COUNTER_OFFSET = 8'h12;
  parameter TIMER_COMPARE_OFFSET        = 8'h20;

  // debug
  parameter DEBUG_ADDR                  = 32'h0A000000;
  parameter DEBUG_ADDR_MASK             = 32'hfffff000;
  parameter DEBUG_REG0_OFFSET           = 8'h00;
  parameter DEBUG_REG1_OFFSET           = 8'h04;
  parameter DEBUG_REG2_OFFSET           = 8'h08;
  parameter DEBUG_REG3_OFFSET           = 8'h0c;
  parameter DEBUG_REG4_OFFSET           = 8'h10;
  parameter DEBUG_REG5_OFFSET           = 8'h14;
  parameter DEBUG_REG6_OFFSET           = 8'h18;
  parameter DEBUG_REG7_OFFSET           = 8'h1c;
  parameter DEBUG_REG8_OFFSET           = 8'h20;

  // reg to pin
  parameter REG_TO_PIN_ADDR             = 32'h0A001000;
  parameter REG_TO_PIN_ADDR_MASK        = 32'hfffff000;
  parameter OUTPUT_REG0_OFFSET          = 8'h00;

  // memory
  parameter DMEM_ADDR                   = 32'h10000000;
  parameter DMEM_ADDR_MASK              = 32'hff000000;

  parameter DMEM2_ADDR                  = 32'h20000000;
  parameter DMEM2_ADDR_MASK             = 32'hfffff000;

  parameter IMEM_ADDR                   = 32'hf0000000;
  parameter IMEM_ADDR_MASK              = 32'hf0000000;

  /******************
      Bridge IDs
  ******************/
/*
  typedef union {
    bridge_id_e name;
    integer value;
  } periph_name_t;
*/
  /////////////////////////////////////////////////////
  //       /!\ not accessible through CPU /!\        //
  /////////////////////////////////////////////////////

  // spi slave : 
  parameter SPI_SLAVE1_ADDR             = 32'hffffffff;
  parameter SPI_SLAVE1_ADDR_MASK        = 32'h00000fff;
  parameter SPI_SLAVE_CONGIG_OFFSET     = 8'h00;
  parameter SPI_SLAVE_CONGIG_OFFSET0    = 8'h00;
  parameter SPI_SLAVE_CONGIG_OFFSET1    = 8'h01;
  parameter SPI_SLAVE_CONGIG_OFFSET2    = 8'h02;
  parameter SPI_SLAVE_CONGIG_OFFSET3    = 8'h03;
  parameter SPI_SLAVE_SOC_CTRL_OFFSET   = 8'h04;
  parameter SPI_SLAVE_SOC_CTRL_OFFSET0  = 8'h04;
  parameter SPI_SLAVE_SOC_CTRL_OFFSET1  = 8'h05;
  parameter SPI_SLAVE_SOC_CTRL_OFFSET2  = 8'h06;
  parameter SPI_SLAVE_SOC_CTRL_OFFSET3  = 8'h07;

  /////////////////////////////////////////////////////


endpackage

`endif // __PCK_MEMORY_MAP__
