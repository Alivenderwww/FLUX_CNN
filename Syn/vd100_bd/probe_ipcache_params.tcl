# probe IPCACHE-related vivado params
foreach p [list_param] {
    if {[string match -nocase "*cache*" $p] || [string match -nocase "*ip.thread*" $p] || [string match -nocase "*ipgen*" $p]} {
        puts "PARAM: $p = [get_param $p]"
    }
}
exit 0
