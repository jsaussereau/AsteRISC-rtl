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
 
`ifndef __PCK_REGISTERS__
`define __PCK_REGISTERS__


typedef enum logic[22:21] {
  single   = 2'b00,
  dual     = 2'b01,
  quad     = 2'b10,
  quad_ddr = 2'b11
} access_mode_e;
 
// qpsi flash master
interface qspi_flash_registers;

  typedef enum logic[22:21] {
    single   = 2'b00,
    dual     = 2'b01,
    quad     = 2'b10,
    quad_ddr = 2'b11
  } access_mode_e;

  typedef struct packed {
    logic         enable;
    logic [30:27] reserved2;
    access_mode_e access_mode;
    logic         continuous_read;
    logic [23:20] dummy;
    logic [19:16] reserved1;
    logic [15:12] bb_flash_oe;
    logic [11:10] reserved0;
    logic         bb_flash_csb;
    logic         bb_flash_clk;
    logic [ 7: 4] bb_flash_out;
    logic [ 3: 0] bb_flash_in;
  } qspi_flash_control_t;

  qspi_flash_control_t qspi_flash_control;

endinterface


// io function
interface iof_registers#(
  parameter p_num_gpios = 24
);

  typedef struct packed {
    logic [31 : p_num_gpios] reserved;
    logic [p_num_gpios-1: 0] sel;
  } iof_config_t;

  iof_config_t iof_config;

endinterface


// gpio
interface gpio_registers#(
  parameter p_num_gpios = 24
);

  typedef struct packed {
    logic [31 : p_num_gpios] reserved;
    logic [p_num_gpios-1: 0] value;
  } gpio_t;

  gpio_t gpio_write_value;
  gpio_t gpio_read_value; //read only
  gpio_t gpio_output_en;
  gpio_t gpio_pullup_en;
  gpio_t gpio_pulldown_en;

endinterface


// pwm
interface pwm_registers #(
  parameter p_cmp_width = 16 // /!\ 16 bits max
);

  localparam PWM_MAX_SCALE = 2**4 - 1; //pwm_config.scale -> 4 bits wide

  typedef struct packed {
    logic         compare3_ip;            //PWM3 interrupt_pending (read-only)
    logic         compare2_ip;            //PWM2 interrupt_pending (read-only)
    logic         compare1_ip;            //PWM1 interrupt_pending (read-only)
    logic         compare0_ip;            //PWM0 interrupt_pending (read-only)
    logic         compare3_gang;          //gang PWM3 and PWM0
    logic         compare2_gang;          //gang PWM2 and PWM3
    logic         compare1_gang;          //gang PWM1 and PWM2
    logic         compare0_gang;          //gang PWM0 and PWM1
    logic         compare3_complement;    //complemenent PWM3 (output at '1' before match then '0')
    logic         compare2_complement;    //complemenent PWM2 (output at '1' before match then '0')
    logic         compare1_complement;    //complemenent PWM1 (output at '1' before match then '0')
    logic         compare0_complement;    //complemenent PWM0 (output at '1' before match then '0')
    logic         compare3_center;        //compare center align    /!\ CURRENTLY UNSUPPORTED /!\
    logic         compare2_center;        //compare center align    /!\ CURRENTLY UNSUPPORTED /!\
    logic         compare1_center;        //compare center align    /!\ CURRENTLY UNSUPPORTED /!\
    logic         compare0_center;        //compare center align    /!\ CURRENTLY UNSUPPORTED /!\
    logic [15:14] reserved2; 
    logic         en_oneshot;             //run one cycle
    logic         en_always;              //run continuously
    logic         reserved1;        
    logic         deglitch;               //capture output of comparator to avoid gliches when changing compare values
    logic         zero_cmp;               //counter resets to zero after match on compare0
    logic         sticky;                 //disallow clearing compareX_ip bits
    logic [ 7: 4] reserved0;
    logic [ 3: 0] scale;                  //counter scale
  } pwm_config_t;

  typedef struct packed {
    logic [31 : p_cmp_width+PWM_MAX_SCALE] reserved;
    logic [p_cmp_width+PWM_MAX_SCALE-1: 0] value;
  } pwm_counter_t;

  typedef struct packed {
    logic [31 : p_cmp_width] reserved;
    logic [p_cmp_width-1: 0] value;
  } pwm_scounter_t;

  pwm_config_t    pwm_config;
  pwm_counter_t   pwm_counter;
  pwm_scounter_t  pwm_scaled_counter;
  pwm_scounter_t  pwm_compare0;
  pwm_scounter_t  pwm_compare1;
  pwm_scounter_t  pwm_compare2;
  pwm_scounter_t  pwm_compare3;
  
endinterface


// i²c
interface i2c_registers;

  typedef struct packed {
    logic [31:24] reserved1;
    logic         system_enable;
    logic         interrupt_enable;
    logic [21:16] reserved0;
    logic [15: 0] prescale;
  } i2c_config_t;

  typedef struct packed {
    logic [31: 8] reserved1;
    logic         start;
    logic         stop;
    logic         read;
    logic         write;
    logic         acknowledge;
    logic [ 2: 1] reserved0;
    logic         interrupt_acknowledge;
  } i2c_command_t;

  typedef struct packed {
    logic [31: 8] reserved1;
    logic         receive_acknowledge;
    logic         i2c_busy;
    logic         arbitrary_lost;
    logic [ 4: 2] reserved0;
    logic         transfer_in_progress;
    logic         interrupt_pending;
  } i2c_status_t;

  typedef struct packed {
    logic [31: 8] reserved;
    logic [ 7: 0] value;
  } i2c_data_t;

  i2c_config_t  i2c_config;
  i2c_command_t i2c_command;
  i2c_status_t  i2c_status;
  i2c_data_t    i2c_data_tx;
  i2c_data_t    i2c_data_rx;

endinterface 

// SPI slave 

interface spi_slave_registers;
  typedef struct packed {
    //logic         system_enable;
    //logic         interrupt_enable;
    logic [31:21] reserved1;
    logic [ 6: 0] send_countdown;
    logic [ 3: 0] compare_offset;
    logic         send_on_compare;
    logic         write_start_pattern; // in fifo
    logic [ 3: 0] msg_size_init_value;
    logic [ 3: 0] msg_size_byte;
    logic         use_msg_size;
    logic         no_current_msg;
    logic         wait_for_msg_start;
  } spi_config_t;

  typedef struct packed {
    logic [31: 0] value;
  } spi_cmp_pattern_t;

  typedef struct packed {
    logic [31: 0] value;
  } spi_cmp_pattern_msk_t;

  typedef struct packed {
    logic [31: 0] value;
  } spi_start_pattern_t;

  typedef struct packed {
    logic [31: 0] value;
  } spi_start_pattern_msk_t;

  typedef struct packed {
    logic [ 6: 0] reserved3;
    logic         fifo_empty_x4;
    logic [ 6: 0] reserved2;
    logic         fifo_empty;
    logic [ 6: 0] reserved1;
    logic         fifo_full_x4;
    logic [ 6: 0] reserved0;
    logic         fifo_full;
  } spi_status_t;

  typedef struct packed {
    logic [31: 8] reserved;
    logic [ 7: 0] value;
  } spi_data_t;

  spi_config_t            spi_config;
  spi_cmp_pattern_t       spi_cmp_pattern;
  spi_cmp_pattern_msk_t   spi_cmp_pattern_msk;
  spi_start_pattern_t     spi_start_pattern;
  spi_start_pattern_msk_t spi_start_pattern_msk;
  spi_status_t            spi_status_tx;
  spi_status_t            spi_status_rx;
  spi_data_t              spi_data_tx;
  logic [31: 0]           spi_data_tx_x4;
  //spi_data_t              spi_data_rx;

endinterface 

// timer
interface timer_registers #(
  parameter p_cmp_width = 32, // /!\ 17 bits min, 32 bits max
  parameter p_postscale = 4   // /1 -> /(2**p_postscale)  /!\ 8 max
);

  localparam TIMER_MAX_SCALE = 2**p_postscale - 1;

  typedef struct packed {
    logic [31:26]            reserved1;
    logic                    overflow_ip;      //counter overflow interrupt pending (read-only)
    logic                    compare_ip;       //compare match interrupt pending (read-only)
    logic                    overflow_if;      //counter overflow interrupt flag
    logic                    compare_if;       //compare match interrupt flag
    logic                    overflow_ie;      //interrupt enable
    logic                    compare_ie;       //interrupt enable
    logic                    sticky;           //disallow clearing compare_ip bits
    logic                    clock_source;     //clock source 0 or 1
    logic                    en_always;        //run continuously
    logic                    en_oneshot;       //run one cycle
    logic                    count_up;         //count up (1) or down (0)
    logic                    zero_cmp;         //counter resets to zero after match
    logic [15: 8]            prescale;         //counter prescale
    logic [ 7 : p_postscale] reserved0;
    logic [p_postscale-1: 0] postscale;        //counter postscale
  } timer_config_t;

  typedef struct packed {
    logic [64 : p_cmp_width+TIMER_MAX_SCALE] reserved;
    logic [p_cmp_width+TIMER_MAX_SCALE-1: 0] value;
  } timer_counter_t;

  typedef struct packed {
    logic [31 : p_cmp_width] reserved;
    logic [p_cmp_width-1: 0] value;
  } timer_scounter_t;

  timer_config_t   timer_config;
  timer_counter_t  timer_counter;
  timer_scounter_t timer_scaled_counter;
  timer_scounter_t timer_compare;
  
endinterface


// uart
interface uart_registers;

  typedef struct packed {
    logic [31: 8] reserved;
    logic [ 7: 0] value;
  } uart_data_t;

  typedef struct packed {
    logic [31:20] reserved;
    //logic [21:20] nb_data_bits; // number of data bits -1 
    logic         nb_stop_bits; // number of stop bits -1 
    logic         parity;       // enable parity
    logic         rx_en;        // receive enable
    logic         tx_en;        // transmit enable
    logic [15: 0] baudrate;     // = ( input_freq / target_baudrate ) -1
  } uart_config_t;

  typedef struct packed {
    logic [31: 2] reserved;
    logic         rx_empty;
    logic         tx_full;
  } uart_status_t;

  uart_data_t   uart_data_tx;
  uart_data_t   uart_data_rx;
  uart_config_t uart_config;
  uart_status_t uart_status;
  
endinterface


// debug
interface debug_registers;

  typedef struct packed {
    logic [31:24] byte3;
    logic [23:16] byte2;
    logic [15: 8] byte1;
    logic [ 7: 0] byte0;
  } debug_reg32_t;

  debug_reg32_t reg0;
  debug_reg32_t reg1;
  debug_reg32_t reg2;
  debug_reg32_t reg3;
  debug_reg32_t reg4;
  debug_reg32_t reg5;  
  debug_reg32_t reg6;  
  debug_reg32_t reg7; 

endinterface

`endif // __PCK_REGISTERS__
