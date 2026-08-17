# package_gauss.tcl : sbm_alg1_gaussian IP打包脚本
set ip_name sbm_alg1_gaussian
create_project -in_memory -part xczu3eg-sfvc784-1-e
add_files {sbm_gauss_h.v sbm_gauss_v.v sbm_alg1_gaussian.v}
ipx::package_project -root_dir ./ip_repo -vendor muteng -library mvtm \
-taxonomy /UserIP -version 1.0 -force
set_property supported_families {zynquplus Production} \
[ipx::current_core]
set_property display_name $ip_name [ipx::current_core]
ipx::save_core [ipx::current_core]
