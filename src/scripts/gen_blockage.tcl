namespace eval ::pdn {
    variable macros {}
    variable instances {}
    
    proc write_macrocell_list {file_name} {
        variable macros
        
        set ch [open $file_name "w"]
        foreach macro_name [dict keys $macros] {
            puts $ch $macro_name
        }
        close $ch
    }
    
    proc get_macro_boundaries {} {
        variable instances

        set boundaries {}
        foreach instance [dict keys $instances] {
            lappend boundaries [dict get $instances $instance macro_boundary]
        }
        
        return $boundaries
    }

    proc get_macro_halo_boundaries {} {
        variable instances

        set boundaries {}
        foreach instance [dict keys $instances] {
            lappend boundaries [dict get $instances $instance halo_boundary]
        }
        
        return $boundaries
    }
    
    proc read_macro_boundaries {def lef_files} {
        variable macros
        variable instances
        
        foreach file $lef_files {
            set ch [open $file]

            while {![eof $ch]} {
                set line [gets $ch]

                if {[regexp {^s*MACRO ([^\s]*)} $line - macro_name]} {
                    set reject 0
                    while {![eof $ch]} {
                        set line [gets $ch]
                        if {[regexp "END $macro_name" $line]} {
                            break
                        }
                        if {[regexp {^\s*CLASS CORE} $line]} {set reject 1}
                        if {[regexp {^\s*CLASS IO} $line]} {set reject 1}
                        if {[regexp {^\s*CLASS ENDCAP} $line]} {set reject 1}
                        if {$reject == 1} {break}

                        regexp {^\s*SIZE ([0-9\.]*) BY ([0-9\.]*)} $line - width height
                    }
                    if {$reject == 0} {
                        dict set macros $macro_name [list width $width height $height]
                    }
                }
            }
            close $ch
        }

        set ch [open $def]
        while {![eof $ch]} {
            set line [gets $ch]

            regexp {UNITS DISTANCE MICRONS ([0-9]*)} $line - dbu

            if {[regexp {^COMPONENTS [0-9]*} $line]} {
                set instances [read_def_components $ch [dict keys $macros]]
            }
        }
        close $ch

        foreach instance [dict keys $instances] {
            set macro_name [dict get $instances $instance macro]
            set width  [expr [dict get $macros $macro_name width] * $dbu]
            set height [expr [dict get $macros $macro_name height] *$dbu]

            set llx [dict get $instances $instance x]
            set lly [dict get $instances $instance y]

            set orient [dict get $instances $instance orient]    
            if {$orient == "N" || $orient == "FN" || $orient == "S" || $orient == "FS"} { 
                set urx [expr round($llx + $width)]
                set ury [expr round($lly + $height)]
            } elseif {$orient == "W" || $orient == "FW" || $orient == "E" || $orient == "FE"} { 
                set urx [expr round($llx + $height)]
                set ury [expr round($lly + $width)]
            }

            dict set instances $instance macro_boundary [list $llx $lly $urx $ury]

            set halo [dict get $instances $instance halo]
            set llx [expr round($llx - [lindex $halo 0])]
            set lly [expr round($lly - [lindex $halo 1])]
            set urx [expr round($urx + [lindex $halo 2])]
            set ury [expr round($ury + [lindex $halo 3])]

            dict set instances $instance halo_boundary [list $llx $lly $urx $ury]
        }
    }

    proc read_def_components {ch macros} {
        variable design_data
        set instances {}

        while {![eof $ch]} {
            set line [gets $ch]

            if {[regexp {END COMPONENTS} $line]} {
                return $instances
            }

            while {![regexp {;} $line]} {
                set line "$line [gets $ch]"
            }

            set macro_name [lindex $line 2]
            if {[lsearch $macros $macro_name] != -1} {
                set data {}
                if {[set idx [lsearch $line "FIXED"]] != -1} {
                    dict set data macro $macro_name
                    dict set data x [lindex $line [expr $idx + 2]]
                    dict set data y [lindex $line [expr $idx + 3]]
                    dict set data orient [lindex $line [expr $idx + 5]]
                }

                if {[set idx [lsearch $line "HALO"]] != -1} {
                    dict set data halo [lrange $line [expr $idx + 1] [expr $idx + 4]]
                } else {
                    dict set data halo [dict get $design_data config default_halo]
                }

                dict set instances [lindex $line 1] $data
            }
        }

        return $instances
    }

    namespace export read_macro_boundaries get_macro_boundaries get_macro_halo_boundaries write_macrocell_list
}
