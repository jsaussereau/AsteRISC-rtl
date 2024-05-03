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

`ifndef __SOC_CYCLE_COUNTER__
`define __SOC_CYCLE_COUNTER__

module soc_cycle_counter (
  input  logic        i_clk,              //! global clock
  input  logic        i_rst,              //! global reset
  input  logic        i_sleep,            //! active high sleep control

  output logic [63:0] o_mcycle            //! cycle count
);

  logic [63:0] mcycle;

  always_ff @(posedge i_clk) begin: registers_write
    if (i_rst) begin
      mcycle <= 32'b0;
    end else begin
      if (!i_sleep) begin
        mcycle <= mcycle + 1;
      end
    end
  end

  assign o_mcycle = mcycle;

endmodule

`endif // __SOC_CYCLE_COUNTER__
