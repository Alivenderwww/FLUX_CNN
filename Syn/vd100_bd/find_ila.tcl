open_project C:/_Project/FLUX_CNN/Syn/vd100_bd/output/vd100_resnet11.xpr
puts "=== Available ILA / debug IPs for this part ==="
foreach ip [get_ipdefs -all -filter {NAME =~ *ila* || NAME =~ *probe*}] {
    puts "  $ip"
}
puts "=== Available debug bridge ==="
foreach ip [get_ipdefs -all -filter {NAME =~ *axis_ila* || NAME =~ *axis_vio* || NAME =~ *system_ila*}] {
    puts "  $ip"
}
exit
