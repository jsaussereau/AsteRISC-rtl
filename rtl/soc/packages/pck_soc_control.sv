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

`ifndef __PCK_SOC_CONTROL__
`define __PCK_SOC_CONTROL__

package pck_soc_control;

  typedef enum logic [0:0] {
    clk_pll      = 1'b0,
    clk_external = 1'b1
  } sel_clk_e;

  typedef enum logic [1:0] {
    rst_por      = 2'b00,
    rst_external = 2'b01,
    rst_spi      = 2'b10,
    rst_all      = 2'b11
  } sel_rst_e;

endpackage

`endif // __PCK_SOC_CONTROL__
