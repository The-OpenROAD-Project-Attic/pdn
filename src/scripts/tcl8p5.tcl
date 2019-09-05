proc lmap {args} {
    upvar height height
    set result {}
    foreach {*}[lrange $args 0 end-1] {
        lappend result [eval [lindex $args end]]
    }
    return $result
}
