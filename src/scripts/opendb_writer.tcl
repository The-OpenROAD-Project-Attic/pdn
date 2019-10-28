namespace eval ::pdn {
    proc write_opendb_vias {} {
        variable physical_viarules
        variable tech

        dict for {name rule} $physical_viarules {
            set via [dbVia_create $block $name]
            $via setViaGenerateRule [dict get $rule rule]
            set params [$via getViaParams]
            $params setBottomLayer [$tech findLayer [lindex [dict get $rule layers] 0]
            $params setCutLayer [$tech findLayer [lindex [dict get $rule layers] 2]
            $params setTopLayer [$tech findLayer [lindex [dict get $rule layers] 1]
            $params setXCutSize [lindex [dict get $rule cutsize] 0]
            $params setYCutSize [lindex [dict get $rule cutsize] 1]
            $params setXCutSpacing [lindex [dict get $rule cutspacing] 0]
            $params setYCutSpacing [lindex [dict get $rule cutspacing] 1]
            $params setXBottomEnlosure [lindex [dict get $rule enclosure] 0]
            $params setYBottomEnlosure [lindex [dict get $rule enclosure] 1]
            $params setXTopEnlosure [lindex [dict get $rule enclosure] 2]
            $params setXTopEnlosure [lindex [dict get $rule enclosure] 3]
            $params setNumCutRows [lindex [dict get $rule rowcol] 0]
            $params setNumCutCols [lindex [dict get $rule rowcol] 1]
        }
        
    }

    proc write_opendb_specialnet {net_name signal_type} {
        variable block
        variable instances
        
        set net [$block findNet $net_name]
        if {$net == "NULL"} {
            set net [dbNet_create $block $net_name]
        }
        $net setSpecial
        $net setSigTypePower

        foreach inst [$block getInsts] {
            foreach iterm [$inst getITerms] {
                if {[$iterm getSigType] == "POWER"} {
                    $iterm setConnected $net
                }
            }
        }

        dict for {inst_name instance} $instances {
            set inst [$block findInst $inst_name]
            foreach pin_name [get_macro_power_pins $inst_name] {
                set iterm [$inst findITerm $pin_name]
                $iterm setConnected $net
            }
        }

        foreach lay [get_metal_layers] {
            set dir [get_dir]
            if {$dir == "hor"} {
                foreach l_str $stripe_locs($lay,$tag) {
                    set l1 [lindex $l_str 0]
                    set l2 [lindex $l_str 1]
                    set l3 [lindex $l_str 2]
                    if {$l1 == $l3} {continue}
                    if {$lay == [get_rails_layer]} {
                        set swire [dbSWire_create]
                        dbWireGraph_createSegment
                        def_out "    $lay [expr round($widths($lay))] + SHAPE FOLLOWPIN ( $l1 $l2 ) ( $l3 * )"
                    } else {
                        def_out "    $lay [expr round($widths($lay))] + SHAPE STRIPE ( $l1 $l2 ) ( $l3 * )"
                    }

                }
            } elseif {$dir == "ver"} {
                foreach l_str $stripe_locs($lay,$tag) {
                    set l1 [lindex $l_str 0]
                    set l2 [lindex $l_str 1]
                    set l3 [lindex $l_str 2]
                    if {$l2 == $l3} {continue}
                    if {$lay == [get_rails_layer]} {
                        def_out "    $lay [expr round($widths($lay))] + SHAPE FOLLOWPIN ( $l1 $l2 ) ( * $l3 )"
                    } else {
                        def_out "    $lay [expr round($widths($lay))] + SHAPE STRIPE ( $l1 $l2 ) ( * $l3 )"
                    }
                }               
            }
        }
        write_opendb_vias $net_name
    }
        
    proc write_opendb_specialnets {} {
        variable block
        
        foreach net_name [dict get $design_data power_nets] {
            write_opendb_specialnet "VDD" "POWER"
        }

        foreach net_name [dict get $design_data ground_nets] {
            write_opendb_specialnet "VDD" "GROUND"
        }
        
    }
    
    proc write_rows {} {
        variable stripe_locs
        
        set stripes [concat $stripe_locs([get_rails_layer],POWER) $stripe_locs([get_rails_layer],GROUND)]
        set stripes [lmap x $stripes {expr {[lindex $x 0] == [lindex $x 2] ? [continue] : $x}}]
        set stripes [lsort -real -index 1 $stripes]
        set heights [lsort -unique -real [lmap x $stripes {lindex $x 1}]]

        init_orientation [lindex $heights 0]

	foreach height [lrange $heights 0 end-1] {
            set lower_rails [lsort -real -index 2 [lmap x $stripes {expr {[lindex $x 1] == $height ? $x : [continue] }}]]
            set lower_rails [lmap x $lower_rails {expr {[lindex $x 0] == [lindex $x 2] ? [continue] : $x}}]
            set upper_rails [lsort -real -index 2 [lmap x $stripes {expr {[lindex $x 1] == $height + $::row_height ? $x : [continue] }}]]
            set upper_rails [lmap x $upper_rails {expr {[lindex $x 0] == [lindex $x 2] ? [continue] : $x}}]
            set upper_extents [concat {*}[lmap x $upper_rails {list [lindex $x 0] [lindex $x 2]}]]

            foreach lrail $lower_rails {
                set idx 0
                set start [lindex $lrail 0]
                set end   [lindex $lrail 2]

                # Find index of first number that is greater than the start position of this rail
                while {$idx < [llength $upper_extents]} {
                    if {[lindex $upper_extents $idx] > $start} {
                        break
                    }
                    incr idx
                }

                if {[lindex $upper_extents $idx] <= $start} {
                    continue
                }

                if {$idx % 2 == 0} {
                    # If the index is even, then the start of the rail has no matching rail above it
                    set row_start [lindex $upper_extents $idx]
                    incr idx
                } else {
                    # If the index is odd, then the start of the rail has matchin rail above it
                    set row_start $start
                }

                if {$end <= [lindex $upper_extents $idx]} {
                    write_row $height $row_start $end
                    
                }

                while {$idx < [llength $upper_extents] && $end > [lindex $upper_extents [expr $idx + 1]]} {
                    write_row $height $row_start [lindex $upper_extents $idx]
                    set row_start [lindex $upper_extents [expr $idx + 1]]
                    set idx [expr $idx + 2]
                }
            }
        }
    }

    
}
