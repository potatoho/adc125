# Usage with Vitis IDE:
# In Vitis IDE create a Single Application Debug launch configuration,
# change the debug type to 'Attach to running target' and provide this 
# tcl script in 'Execute Script' option.
# Path of this script: C:\Users\annoa\FL9627\trigger_mode\vitis_control\test_system\_ide\scripts\systemdebugger_test_system_standalone.tcl
# 
# 
# Usage with xsct:
# To debug using xsct, launch xsct and run below command
# source C:\Users\annoa\FL9627\trigger_mode\vitis_control\test_system\_ide\scripts\systemdebugger_test_system_standalone.tcl
# 
connect -url tcp:127.0.0.1:3121
source C:/Xilinx/Vitis/2022.2/scripts/vitis/util/zynqmp_utils.tcl
targets -set -nocase -filter {name =~"APU*"}
rst -system
after 3000
targets -set -filter {jtag_cable_name =~ "Xilinx HW-Z1-ZCU104 FT4232H 61570A" && level==0 && jtag_device_ctx=="jsn-HW-Z1-ZCU104 FT4232H-61570A-14730093-0"}
fpga -file C:/Users/annoa/FL9627/trigger_mode/vitis_control/test/_ide/bitstream/top_footer2.bit
targets -set -nocase -filter {name =~"APU*"}
loadhw -hw C:/Users/annoa/FL9627/trigger_mode/vitis_control/top_footer2/export/top_footer2/hw/top_footer2.xsa -mem-ranges [list {0x80000000 0xbfffffff} {0x400000000 0x5ffffffff} {0x1000000000 0x7fffffffff}] -regs
configparams force-mem-access 1
targets -set -nocase -filter {name =~"APU*"}
set mode [expr [mrd -value 0xFF5E0200] & 0xf]
targets -set -nocase -filter {name =~ "*A53*#0"}
rst -processor
dow C:/Users/annoa/FL9627/trigger_mode/vitis_control/top_footer2/export/top_footer2/sw/top_footer2/boot/fsbl.elf
set bp_19_37_fsbl_bp [bpadd -addr &XFsbl_Exit]
con -block -timeout 60
bpremove $bp_19_37_fsbl_bp
targets -set -nocase -filter {name =~ "*A53*#0"}
rst -processor
dow C:/Users/annoa/FL9627/trigger_mode/vitis_control/test/Debug/test.elf
configparams force-mem-access 0
targets -set -nocase -filter {name =~ "*A53*#0"}
con
