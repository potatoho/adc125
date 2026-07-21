# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct C:\Users\annoa\FL9627\trigger_mode\vitis_control\top_footer2\platform.tcl
# 
# OR launch xsct and run below command.
# source C:\Users\annoa\FL9627\trigger_mode\vitis_control\top_footer2\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {top_footer2}\
-hw {C:\Users\annoa\FL9627\trigger_mode\top_footer2.xsa}\
-arch {64-bit} -fsbl-target {psu_cortexa53_0} -out {C:/Users/annoa/FL9627/trigger_mode/vitis_control}

platform write
domain create -name {standalone_psu_cortexa53_0} -display-name {standalone_psu_cortexa53_0} -os {standalone} -proc {psu_cortexa53_0} -runtime {cpp} -arch {64-bit} -support-app {lwip_echo_server}
platform generate -domains 
platform active {top_footer2}
domain active {zynqmp_fsbl}
domain active {zynqmp_pmufw}
domain active {standalone_psu_cortexa53_0}
platform generate -quick
platform generate
