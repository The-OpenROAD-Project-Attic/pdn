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
                if {[$cell getType] == "CORE"} {continue}
                if {[$cell getType] == "IO"} {continue}
                if {[$cell getType] == "PAD"} {continue}
                if {[$cell getType] == "PAD_SPACER"} {continue}
                if {[$cell getType] == "SPACER"} {continue}
                if {[$cell getType] == "NONE"} {continue}
                if {[$cell getType] == "ENDCAP_PRE"} {continue}
                if {[$cell getType] == "ENDCAP_BOTTOMLEFT"} {continue}
                if {[$cell getType] == "ENDCAP_BOTTOMRIGHT"} {continue}
                if {[$cell getType] == "ENDCAP_TOPLEFT"} {continue}
                if {[$cell getType] == "ENDCAP_TOPRIGHT"} {continue}
                if {[$cell getType] == "ENDCAP"} {continue}
                if {[$cell getType] == "CORE_SPACER"} {continue}
                if {[$cell getType] == "CORE_TIEHIGH"} {continue}
                if {[$cell getType] == "CORE_TIELOW"} {continue}

                dict set macros [$cell getName] [list \
                    width  [$cell getWidth] \
                    height [$cell getHeight] \
                ]
            }
        }

        set instances [read_def_components [dict keys $macros]]

        foreach instance [dict keys $instances] {
            set macro_name [dict get $instances $instance macro]

            set llx [dict get $instances $instance xmin]
            set lly [dict get $instances $instance ymin]
            set urx [dict get $instances $instance xmax]
            set ury [dict get $instances $instance ymax]

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
                dict set data xmin [[$inst getBBox] xMin]
                dict set data ymin [[$inst getBBox] yMin]
                dict set data xmax [[$inst getBBox] xMax]
                dict set data ymax [[$inst getBBox] yMax]
                dict set data orient [$inst getOrient]

                if {[$inst getHalo] != "NULL"} {
                    dict set data halo [list \
                        [[$inst getHalo] xMin] \
                        [[$inst getHalo] yMin] \
                        [[$inst getHalo] xMax] \
                        [[$inst getHalo] yMax] \
                    ]
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
