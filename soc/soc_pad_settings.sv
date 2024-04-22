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

`ifndef __SOC_PAD_SETTINGS__
`define __SOC_PAD_SETTINGS__

module soc_pad_settings (
  input  wire netTie0,
  input  wire netTie1,
  output wire pad_input,
  output wire pad_output,
  output wire pad_pullup_on,
  output wire pad_pullup_off,
  output wire pad_pulldown_on,
  output wire pad_pulldown_off
);

  assign pad_input        = netTie0;
  assign pad_output       = netTie1;
  assign pad_pullup_off   = netTie0;
  assign pad_pullup_on    = netTie1;
  assign pad_pulldown_off = netTie0;
  assign pad_pulldown_on  = netTie1;

endmodule

`endif // __SOC_PAD_SETTINGS__
