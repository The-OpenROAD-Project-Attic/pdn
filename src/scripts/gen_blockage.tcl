namespace eval ::pdn {
    variable macros {}
    variable instances {}
    
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
    
    proc read_macro_boundaries {} {
        variable libs
        variable macros
        variable instances

        foreach lib $libs {
            foreach cell [$lib getMasters] {
                if {[$cell getType] == "CORE"} {break}
                if {[$cell getType] == "IO"} {break}
                if {[$cell getType] == "ENDCAP"} {break}
    
                set height [$cell getHeight]
                set width  [$cell getWidth]

                dict set macros [$cell getName] [list width $width height $height]
            }
        }

        set instances [read_def_components [dict keys $macros]]

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

    proc read_def_components {macros} {
        variable design_data
        variable block
        set instances {}

        foreach inst [$block getInsts] {
            set macro_name [[$inst getMaster] getName]
            if {[lsearch $macros $macro_name] != -1} {
                set data {}
                dict set data name [$inst getName]
                dict set data macro $macro_name
                dict set data x [lindex [$inst getOrigin] 0]
                dict set data y [lindex [$inst getOrigin] 1]
                dict set data orient [$inst getOrient]

                if {[$inst getHalo] != ""} {
                    dict set data halo [$inst getHalo]
                } else {
                    dict set data halo [dict get $design_data config default_halo]
                }

                dict set instances [$inst getName] $data
            }
        }

        return $instances
    }

    namespace export read_macro_boundaries get_macro_boundaries get_macro_halo_boundaries write_macrocell_list
}
