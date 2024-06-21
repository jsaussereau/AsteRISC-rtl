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

`ifndef __WRAP_POR__
`define __WRAP_POR__

module wrap_por (
  input  wire         i_clk,
  output logic        o_rst
);

 assign o_rst = 1'b0;

endmodule

`endif // __WRAP_POR__

`endif // TARGET_SIM
