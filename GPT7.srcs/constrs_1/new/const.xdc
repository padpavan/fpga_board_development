## 100 MHz CLOCK
set_property -dict { PACKAGE_PIN W5 IOSTANDARD LVCMOS33 } [get_ports CLOCK]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports CLOCK]

## BUTTONS
set_property -dict { PACKAGE_PIN T18 IOSTANDARD LVCMOS33 } [get_ports BTNU]
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33 } [get_ports BTND]
set_property -dict { PACKAGE_PIN U18   IOSTANDARD LVCMOS33 } [get_ports BTNC]

## SWITCHES
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports sw0]
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports sw1]

## OLED MODE SELECTION
set_property -dict { PACKAGE_PIN T1 IOSTANDARD LVCMOS33 } [get_ports sw14]
set_property -dict { PACKAGE_PIN R2 IOSTANDARD LVCMOS33 } [get_ports sw15]

## 7 SEGMENT DISPLAY
set_property -dict { PACKAGE_PIN W7 IOSTANDARD LVCMOS33 } [get_ports {seg[0]}]
set_property -dict { PACKAGE_PIN W6 IOSTANDARD LVCMOS33 } [get_ports {seg[1]}]
set_property -dict { PACKAGE_PIN U8 IOSTANDARD LVCMOS33 } [get_ports {seg[2]}]
set_property -dict { PACKAGE_PIN V8 IOSTANDARD LVCMOS33 } [get_ports {seg[3]}]
set_property -dict { PACKAGE_PIN U5 IOSTANDARD LVCMOS33 } [get_ports {seg[4]}]
set_property -dict { PACKAGE_PIN V5 IOSTANDARD LVCMOS33 } [get_ports {seg[5]}]
set_property -dict { PACKAGE_PIN U7 IOSTANDARD LVCMOS33 } [get_ports {seg[6]}]
set_property -dict { PACKAGE_PIN V7 IOSTANDARD LVCMOS33 } [get_ports {seg[7]}]

set_property -dict { PACKAGE_PIN U2 IOSTANDARD LVCMOS33 } [get_ports {an[0]}]
set_property -dict { PACKAGE_PIN U4 IOSTANDARD LVCMOS33 } [get_ports {an[1]}]
set_property -dict { PACKAGE_PIN V4 IOSTANDARD LVCMOS33 } [get_ports {an[2]}]
set_property -dict { PACKAGE_PIN W4 IOSTANDARD LVCMOS33 } [get_ports {an[3]}]

## PMOD OLED RGB - JB HEADER
set_property -dict { PACKAGE_PIN A14 IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_cs]
set_property -dict { PACKAGE_PIN A16 IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_sdin]
set_property -dict { PACKAGE_PIN B16 IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_sclk]
set_property -dict { PACKAGE_PIN A15 IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_d_cn]
set_property -dict { PACKAGE_PIN A17 IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_resn]
set_property -dict { PACKAGE_PIN C15 IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_vccen]
set_property -dict { PACKAGE_PIN C16 IOSTANDARD LVCMOS33 } [get_ports pmodoledrgb_pmoden]

## CONFIGURATION
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]