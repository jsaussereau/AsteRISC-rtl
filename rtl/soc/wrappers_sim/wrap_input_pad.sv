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

`ifdef VIVADO
  `include "../soc_config.sv"
`else
  `include "soc/soc_config.sv"
`endif

`ifdef TARGET_SIM

`ifndef __INPUT_PAD__
`define __INPUT_PAD__

module wrap_input_pad (
  output o_pad_in,

  input  io_pad,

  input  netTie0,         // LEAVE UNCONNECTED FOR SIMULATIONS
  input  netTie1,         // LEAVE UNCONNECTED FOR SIMULATIONS  

  input  vdd_io,          // LEAVE UNCONNECTED FOR SIMULATIONS
  input  vdd_co,          // LEAVE UNCONNECTED FOR SIMULATIONS
  input  vss              // LEAVE UNCONNECTED FOR SIMULATIONS
);

  assign o_pad_in = io_pad;

endmodule

`endif // __WRAP_INPUT_PAD__

`endif // TARGET_SIM
