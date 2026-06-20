# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct C:\Users\annoa\FL9627\project2_try1\vitis_2\top\platform.tcl
# 
# OR launch xsct and run below command.
# source C:\Users\annoa\FL9627\project2_try1\vitis_2\top\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {top}\
-hw {C:\Users\annoa\FL9627\project2_try1\top.xsa}\
-arch {64-bit} -fsbl-target {psu_cortexa53_0} -out {C:/Users/annoa/FL9627/project2_try1/vitis_2}

platform write
domain create -name {standalone_psu_cortexa53_0} -display-name {standalone_psu_cortexa53_0} -os {standalone} -proc {psu_cortexa53_0} -runtime {cpp} -arch {64-bit} -support-app {lwip_echo_server}
platform generate -domains 
platform active {top}
domain active {zynqmp_fsbl}
domain active {zynqmp_pmufw}
domain active {standalone_psu_cortexa53_0}
platform generate -quick
platform generate
