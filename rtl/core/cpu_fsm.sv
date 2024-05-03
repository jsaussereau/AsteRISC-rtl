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

`ifndef __CPU_FSM__
`define __CPU_FSM__

`ifdef VIVADO
 `include "packages/pck_control.sv"
`else
 `include "core/packages/pck_control.sv"
`endif

//TODO: (p_decode_buf && p_mem_buf) => remove second memory cycle -> do it in next decode state 
//TODO: HANDLE BUSY INSTRUCTION MEMORY:

module cpu_fsm 
  import pck_control::*;
#(
  parameter p_rf_sp        = 0,       //! register file is a single port ram
  parameter p_rf_read_buf  = 0,       //! register file has synchronous read
  parameter p_prefetch_buf = 0,       //! use a prefetch buffer
  parameter p_decode_buf   = 0,       //! add buffers to decode stage outputs
  parameter p_mem_buf      = 0,       //! add buffers to mem stage inputs
  parameter p_branch_buf   = 0,       //! add buffers to alu comp outputs (+1 cycle for conditionnal branches)
  parameter p_wait_for_ack = 0,       //! wait for data bus acknowledgement
  parameter p_wb_buf       = 0        //! add buffers to write back stage inputs
)(
  input  wire          i_clk,         //! global clock
  input  wire          i_rst,         //! global reset
  input  wire          i_sleep,       //! active high sleep control

  input  wire          i_refetch,     //! fetch again
  input  wire          i_exec_done,   //! execute stage done
  input  sel_br_e      i_sel_br,      //! branch
  input  wire          i_dmem_wr,     //! data memory write
  input  wire          i_dmem_rd,     //! data memory read
  input  wire          i_wb,          //! write back
  input  wire          i_cond_branch, //! conditionnal branch
  input  wire          i_jump_reg,    //! jump register
  input  wire          i_bad_predict, //! bad branch prediction

  input  wire          i_rf_rd2_used, //! register file read port 2 is used
  input  wire          i_rf_busy,     //! regfile busy

  input  wire          i_dbus_busy,   //! data bus busy
  input  wire          i_dbus_ack,    //! data bus acknowledge

  input  wire          i_ibus_busy,   //! instruction bus busy
  input  wire          i_ibus_ack,    //! instruction bus acknowledge
  
  output logic         o_en_fetch,    //! fetch next instruction
  output logic         o_update_pc,   //! update program counter
  output logic         o_en_decomp,   //! enable decompressor
  output logic         o_update_comp, //! update decompressor fsm
  output logic         o_en_decode,   //! enable decode
  output logic         o_en_rf_rd1,   //! read first register (used only if p_rf_sp = 1)
  output logic         o_en_rf_rd2,   //! read second register (used only if p_rf_sp = 1)
  output logic         o_en_exec,     //! enable execution
  output logic         o_en_dmem_wr,  //! data memory write enable
  output logic         o_en_dmem_rd,  //! data memory read enable
  output logic         o_en_wb,       //! write back enable
  output logic         o_wb_state     //! currently in write back state
);

  typedef enum logic [3:0] {
    st_init,
    st_refetch,
    st_refetch2,
    st_decode,
    st_read_rf,
    st_read_rf_sp,
    st_execute,
    st_execute_buf,
    st_atom_memory,
    st_memory,
    st_memory_prebuf,
    st_write_back,
    st_write_back_buf
  } fsm_state_e;

  fsm_state_e last_state;             //! last fsm state
  fsm_state_e curr_state;             //! current fsm state
  fsm_state_e next_state;             //! next fsm state

  //localparam fsm_state_e decode_next_state  = p_rf_read_buf ? st_read_rf    : st_execute;
  //localparam fsm_state_e init_next_state    = p_decode_buf  ? st_decode     : p_rf_read_buf ? st_read_rf : st_execute;
  //localparam fsm_state_e rf_read_next_state = p_rf_sp       ? st_read_rf_sp : st_execute;

  logic init_update_pc;
  logic exec_en_fetch;
  logic exec_update_pc;
  logic mem_en_fetch;
  logic mem_update_pc;
  logic mem_en_wb;

  logic state_change;

  logic [ 7: 0] state_counter;

  //! determine mealy machine outputs
  always_comb begin
    init_update_pc = !i_rf_busy && state_change; 
    exec_en_fetch  = !i_dmem_rd;                              // if load from memory: do not fetch yet (in exec stage)
    exec_update_pc = !i_dmem_rd && state_change;
    mem_en_fetch   = i_dmem_rd && !i_dbus_busy && (!p_wait_for_ack || i_dbus_ack); // if load from memory: now we can fetch (in mem stage)
    mem_update_pc  = i_dmem_rd && !i_dbus_busy && (!p_wait_for_ack || i_dbus_ack) && state_change;
    mem_en_wb      = i_wb && !i_dmem_rd; // do not write back in mem stage if load from memory (write back at wb stage)
  end

  
  //! update the current state every clock cycle
  always_ff @(posedge i_clk) begin: update_curr_state
    if (i_rst) begin
      curr_state <= st_init;
      last_state <= st_init;
    end else begin
      if (!i_sleep) begin
        curr_state <= next_state;
        last_state <= curr_state;
      end
    end
  end

  //! decode next state
  always_comb begin: next_state_decoder
    state_change = 1'b1; // by default, we consider we are changing state

    case (curr_state)
      st_init: begin
        if (!i_ibus_busy) begin // wait for instruction memory to be ready
          next_state = p_decode_buf  ? st_decode     : p_rf_read_buf ? st_read_rf : st_execute;
        end else begin
          next_state = st_init;
          state_change = 1'b0;
        end
      end
      st_refetch: begin
        if (!i_ibus_busy) begin // wait for instruction memory to be ready
          next_state = st_refetch2;
        end else begin
          next_state = st_init;
          state_change = 1'b0;
        end
      end
      st_refetch2: begin
        next_state = p_decode_buf  ? st_decode     : p_rf_read_buf ? st_read_rf : st_execute;
      end
      st_decode: begin
        next_state = p_rf_read_buf ? st_read_rf : st_execute;
      end
      st_read_rf: begin
        next_state = (p_rf_sp && i_rf_rd2_used) ? st_read_rf_sp : st_execute;
      end
      st_read_rf_sp: begin
        next_state = st_execute;
      end
      st_execute: begin
        if (i_exec_done) begin
          if (i_dmem_wr || i_dmem_rd) begin
            next_state = p_mem_buf ? st_memory_prebuf : st_memory;
          end else if (p_branch_buf && (i_cond_branch || i_jump_reg)) begin
            next_state = st_execute_buf;
          end else begin
            next_state = st_write_back;
          end
        end else begin
          next_state = st_execute;
          state_change = 1'b0;
        end
      end
      st_execute_buf: begin
        next_state = st_write_back;
      end
      st_memory_prebuf: begin
        next_state = st_memory;
      end
      st_memory: begin
        if (!i_dbus_busy && (!p_wait_for_ack || i_dbus_ack)) begin  // wait for data memory to be ready
          if (i_dmem_rd) begin
            next_state = st_write_back;
          end else begin
            if (!i_ibus_busy) begin // wait for instruction memory to be ready
              next_state = i_refetch ? st_refetch : p_decode_buf  ? st_decode     : p_rf_read_buf ? st_read_rf : st_execute;
            end else begin
              next_state = st_memory;
              state_change = 1'b0;
            end
          end
        end else begin
          next_state = st_memory;
          state_change = 1'b0;
        end
      end
      st_write_back: begin
        if (!i_ibus_busy && !(p_prefetch_buf && i_bad_predict)) begin // wait for instruction memory to be ready
          if (p_wb_buf && !p_decode_buf) begin
            next_state = st_write_back_buf;
          end else begin
            next_state = i_refetch ? st_refetch : p_decode_buf  ? st_decode     : p_rf_read_buf ? st_read_rf : st_execute; 
          end
        end else begin
          next_state = st_write_back;
          state_change = 1'b0;
        end
      end
      st_write_back_buf: begin
        next_state = i_refetch ? st_refetch : p_decode_buf  ? st_decode     : p_rf_read_buf ? st_read_rf : st_execute; 
      end
      default: begin
        next_state = st_init; 
      end
    endcase
  end

  //! decode output signals
  always_comb begin: output_signals
    case (curr_state)
      st_init: begin
        o_en_fetch    = 1'b1;          // fetch the first instruction
        o_update_pc   = init_update_pc;// fetch the first instruction
        o_en_decomp   = 1'b0;
        o_update_comp = 1'b0;
        o_en_decode   = 1'b1;          // decode the first instruction
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0;
        o_en_dmem_rd  = 1'b0;
        o_en_wb       = 1'b0;
        o_wb_state    = 1'b0;
      end
      st_refetch: begin
        o_en_fetch    = 1'b1;          // fetch the first instruction
        o_update_pc   = 1'b0;
        o_en_decomp   = state_change;
        o_update_comp = state_change;
        o_en_decode   = 1'b0;
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0;
        o_en_dmem_rd  = 1'b0;
        o_en_wb       = 1'b0;
        o_wb_state    = 1'b0;
      end
      st_refetch2: begin
        o_en_fetch    = 1'b0;          // fetch the first instruction
        o_update_pc   = 1'b0;
        o_en_decomp   = 1'b1;
        o_update_comp = 1'b0;
        o_en_decode   = 1'b0;
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0;
        o_en_dmem_rd  = 1'b0;
        o_en_wb       = 1'b0;
        o_wb_state    = 1'b0;
      end
      st_decode: begin
        o_en_fetch    = 1'b0;
        o_update_pc   = 1'b0;
        o_en_decomp   = 1'b1;
        o_update_comp = 1'b0;
        o_en_decode   = 1'b1;          // decode the current instruction
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0;
        o_en_dmem_rd  = 1'b0;
        o_en_wb       = 1'b0;
        o_wb_state    = 1'b0;
      end
      st_read_rf: begin
        o_en_fetch    = 1'b0;
        o_update_pc   = 1'b0;
        o_en_decomp   = 1'b0;
        o_update_comp = 1'b0;
        o_en_decode   = 1'b1;          // decode the current instruction
        o_en_rf_rd1   = 1'b1;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0;
        o_en_dmem_rd  = 1'b0;
        o_en_wb       = 1'b0;
        o_wb_state    = 1'b0;
      end
      st_read_rf_sp: begin
        o_en_fetch    = 1'b0;
        o_update_pc   = 1'b0;
        o_en_decomp   = 1'b0;
        o_update_comp = 1'b0;
        o_en_decode   = 1'b0;
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b1;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0;
        o_en_dmem_rd  = 1'b0;
        o_en_wb       = 1'b0;
        o_wb_state    = 1'b0;
      end
      st_execute: begin
        o_en_fetch    = exec_en_fetch && !(p_branch_buf && (i_cond_branch || i_jump_reg)) && state_change;
        o_update_pc   = exec_en_fetch && !(p_branch_buf && (i_cond_branch || i_jump_reg)) && state_change;
        o_en_decomp   = p_decode_buf ? 1'b0 : 1'b1;
        o_update_comp = 1'b0;
        o_en_decode   = 1'b1;
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b1;
        o_en_dmem_wr  = i_dmem_wr;     // write as soon as posible
        o_en_dmem_rd  = 1'b0;
        o_en_wb       = 1'b0;
        o_wb_state    = 1'b0;
      end
      st_execute_buf: begin
        o_en_fetch    = exec_en_fetch;
        o_update_pc   = exec_en_fetch;
        o_en_decomp   = 1'b0;
        o_update_comp = 1'b0;
        o_en_decode   = 1'b1;
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0; 
        o_en_dmem_rd  = 1'b0;
        o_en_wb       = 1'b0;
        o_wb_state    = 1'b0;
      end
      st_memory_prebuf: begin
        o_en_fetch    = 1'b0;
        o_update_pc   = 1'b0;
        o_en_decomp   = 1'b0;
        o_update_comp = 1'b0;
        o_en_decode   = 1'b0;
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0;
        o_en_dmem_rd  = i_dmem_rd;     // wait for the value
        o_en_wb       = 1'b0;
        o_wb_state    = 1'b0;
      end
      st_memory: begin
        o_en_fetch    = mem_en_fetch;
        o_update_pc   = mem_en_fetch && state_change;
        o_en_decomp   = 1'b0;
        o_update_comp = !i_dmem_rd;
        o_en_decode   = 1'b0;
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0;
        o_en_dmem_rd  = i_dmem_rd;     // wait for the value
        o_en_wb       = mem_en_wb;
        o_wb_state    = 1'b0;
      end
      st_write_back: begin
        o_en_fetch    = 1'b0;//i_refetch && !(p_wb_buf && !p_decode_buf) && state_change;
        o_update_pc   = 1'b0;
        o_en_decomp   = 1'b0;
        o_update_comp = !(p_wb_buf && !p_decode_buf) && state_change;
        o_en_decode   = 1'b0;
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0;
        o_en_dmem_rd  = i_dmem_rd;     // keeping read enable high 
                                       // to make sure the bridge outputs the value
        o_en_wb       = i_wb;          // write back
        o_wb_state    = 1'b1;
      end
      st_write_back_buf: begin
        o_en_fetch    = 1'b0;//i_refetch;
        o_update_pc   = 1'b0;
        o_en_decomp   = 1'b0;
        o_update_comp = 1'b1;
        o_en_decode   = 1'b0;          // decode the current instruction
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0;
        o_en_dmem_rd  = 1'b0;
        o_en_wb       = 1'b0;
        o_wb_state    = 1'b0;
      end
      default: begin
        o_en_fetch    = 1'b0;
        o_update_pc   = 1'b0;
        o_en_decomp   = 1'b0;
        o_update_comp = 1'b0;
        o_en_decode   = 1'b0;
        o_en_rf_rd1   = 1'b0;
        o_en_rf_rd2   = 1'b0;
        o_en_exec     = 1'b0;
        o_en_dmem_wr  = 1'b0;
        o_en_dmem_rd  = 1'b0;
        o_en_wb       = 1'b0;
        o_wb_state    = 1'b0;
      end
    endcase
  end

  // count cycles in same state
  always_ff @(posedge i_clk) begin: cycles_in_same_state
    if (i_rst) begin
      state_counter <= 8'd0;
    end else begin
      if (curr_state != last_state) begin
        state_counter <= 8'd0;
      end else begin
        state_counter <= state_counter + 1;
      end
    end
  end

endmodule

`endif // __CPU_FSM__
