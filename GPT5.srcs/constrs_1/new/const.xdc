## EE2026 - GPT-5 OLED bring-up constraints (Basys 3)
## Only the pins used by the design are constrained.

## Clock signal (100 MHz)
set_property -dict { PACKAGE_PIN W5   IOSTANDARD LVCMOS33 } [get_ports CLOCK]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports CLOCK]

## Switch SW4 -> background colour select
set_property -dict { PACKAGE_PIN W15  IOSTANDARD LVCMOS33 } [get_ports sw4]

## Button BTNU -> reset
set_property -dict { PACKAGE_PIN T18  IOSTANDARD LVCMOS33 } [get_ports reset]

## Pmod OLED RGB on the JB header
set_property -dict { PACKAGE_PIN A14  IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_cs]     ;# JB[0] JB1
set_property -dict { PACKAGE_PIN A16  IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_sdin]   ;# JB[1] JB2
set_property -dict { PACKAGE_PIN B16  IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_sclk]   ;# JB[3] JB4
set_property -dict { PACKAGE_PIN A15  IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_d_cn]   ;# JB[4] JB7
set_property -dict { PACKAGE_PIN A17  IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_resn]   ;# JB[5] JB8
set_property -dict { PACKAGE_PIN C15  IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_vccen]  ;# JB[6] JB9
set_property -dict { PACKAGE_PIN C16  IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_pmoden] ;# JB[7] JB10

## Configuration options, can be used for all designs
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]

## SPI configuration mode options for QSPI boot, can be used for all designs
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
