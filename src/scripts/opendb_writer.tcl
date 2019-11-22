namespace eval ::pdn {
    variable global_connections {
        VDD {
            {inst_name .* pin_name VDD}
            {inst_name .* pin_name VDDPE}
            {inst_name .* pin_name VDDCE}
        }
        VSS {
            {inst_name .* pin_name VSS}
            {inst_name .* pin_name VSSE}
        }
    }
    
    proc write_opendb_vias {} {
        variable physical_viarules
        variable block
        variable tech

        dict for {name rule} $physical_viarules {
            set via [dbVia_create $block $name]
            $via setViaGenerateRule [$tech findViaGenerateRule [dict get $rule rule]]
            set params [$via getViaParams]
            $params setBottomLayer [$tech findLayer [lindex [dict get $rule layers] 0]]
            $params setCutLayer [$tech findLayer [lindex [dict get $rule layers] 2]]
            $params setTopLayer [$tech findLayer [lindex [dict get $rule layers] 1]]
            $params setXCutSize [lindex [dict get $rule cutsize] 0]
            $params setYCutSize [lindex [dict get $rule cutsize] 1]
            $params setXCutSpacing [lindex [dict get $rule cutspacing] 0]
            $params setYCutSpacing [lindex [dict get $rule cutspacing] 1]
            $params setXBottomEnclosure [lindex [dict get $rule enclosure] 0]
            $params setYBottomEnclosure [lindex [dict get $rule enclosure] 1]
            $params setXTopEnclosure [lindex [dict get $rule enclosure] 2]
            $params setYTopEnclosure [lindex [dict get $rule enclosure] 3]
            $params setNumCutRows [lindex [dict get $rule rowcol] 0]
            $params setNumCutCols [lindex [dict get $rule rowcol] 1]

            $via setViaParams $params
        }
    }

    proc write_opendb_specialnet {net_name signal_type} {
        variable block
        variable instances
        variable metal_layers
        variable tech 
        variable stripe_locs
        variable widths
        variable global_connections
        
        set net [$block findNet $net_name]
        if {$net == "NULL"} {
            set net [dbNet_create $block $net_name]
        }
        $net setSpecial
        $net setSigType $signal_type

        foreach inst [$block getInsts] {
            set master [$inst getMaster]
            foreach mterm [$master getMTerms] {
                if {[$mterm getSigType] == $signal_type} {
                    foreach pattern [dict get $global_connections $net_name] {
                        if {[regexp [dict get $pattern inst_name] [$inst getName]] &&
                            [regexp [dict get $pattern pin_name] [$mterm getName]]} {
                            dbITerm_connect $inst $net $mterm
                        }
                    }
                }
            }
            foreach iterm [$inst getITerms] {
                if {[$iterm getNet] != "NULL" && [[$iterm getNet] getName] == $net_name} {
                    $iterm setSpecial
                }
            }
        }
        $net setWildConnected
        set swire [dbSWire_create $net "ROUTED"]

        foreach lay $metal_layers {
            set layer [$tech findLayer $lay]

            set dir [get_dir $lay]
            if {$dir == "hor"} {
                foreach l_str $stripe_locs($lay,$signal_type) {
                    set l1 [lindex $l_str 0]
                    set l2 [lindex $l_str 1]
                    set l3 [lindex $l_str 2]
                    if {$l1 == $l3} {continue}
                    if {$lay == [get_rails_layer]} {
                        dbSBox_create $swire $layer [expr round($l1)] [expr round($l2 - ($widths($lay)/2))] [expr round($l3)] [expr round($l2 + ($widths($lay)/2))] "FOLLOWPIN"
                    } else {
                        dbSBox_create $swire $layer [expr round($l1)] [expr round($l2 - ($widths($lay)/2))] [expr round($l3)] [expr round($l2 + ($widths($lay)/2))] "STRIPE"
                    }

                }
            } elseif {$dir == "ver"} {
                foreach l_str $stripe_locs($lay,$signal_type) {
                    set l1 [lindex $l_str 0]
                    set l2 [lindex $l_str 1]
                    set l3 [lindex $l_str 2]
                    if {$l2 == $l3} {continue}
                    if {$lay == [get_rails_layer]} {
                        dbSBox_create $swire $layer [expr round($l1 - ($widths($lay)/2))] [expr round($l2)] [expr round($l1 + ($widths($lay)/2))] [expr round($l3)] "FOLLOWPIN"
                    } else {
                        dbSBox_create $swire $layer [expr round($l1 - ($widths($lay)/2))] [expr round($l2)] [expr round($l1 + ($widths($lay)/2))] [expr round($l3)] "STRIPE"
                    }
                }               
            }
        }

        variable vias
        foreach via $vias {
            if {[dict get $via net_name] == $net_name} {
	        # For each layer between l1 and l2, add vias at the intersection
                foreach via_inst [dict get $via connections] {
                    set via_name [dict get $via_inst name]
                    set x        [dict get $via_inst x]
                    set y        [dict get $via_inst y]
                    set lay      [dict get $via_inst lower_layer]
                    regexp {(.*)_PIN} $lay - lay
                    set layer [$tech findLayer $lay]
                    dbSBox_create $swire [$block findVia $via_name] $x $y "STRIPE"
	        }
            }
        }
    }
        
    proc write_opendb_specialnets {} {
        variable block
        variable design_data
        
        foreach net_name [dict get $design_data power_nets] {
            write_opendb_specialnet "VDD" "POWER"
        }

        foreach net_name [dict get $design_data ground_nets] {
            write_opendb_specialnet "VSS" "GROUND"
        }
        
    }
    
    proc init_orientation {height} {
        variable lowest_rail
        variable orient_rows
        variable rails_start_with

        set lowest_rail $height
        if {$rails_start_with == "GROUND"} {
            set orient_rows {0 "R0" 1 "MX"}
        } else {
            set orient_rows {0 "MX" 1 "R0"}
        }
    }

    proc orientation {height} {
        variable lowest_rail
        variable orient_rows
        variable row_height
        
        set row_line [expr int(($height - $lowest_rail) / $row_height)]
        return [dict get $orient_rows [expr $row_line % 2]]
    }
        
    proc write_opendb_row {height start end} {
        variable row_index
        variable block
        variable site
        variable site_width
        variable def_units
        variable design_data
        
        set start  [expr int($start)]
        set height [expr int($height)]
        set end    [expr int($end)]

        if {$start == $end} {return}
        
        set llx [lindex [dict get $design_data config core_area] 0]
	if {[expr { int($start - $llx) % $site_width}] == 0} {
		set x $start
	} else {
		set offset [expr { int($start - $llx) % $site_width}]
		set x [expr {$start + $site_width - $offset}]
	}

	set num  [expr {($end - $x)/$site_width}]
        
        dbRow_create $block ROW_$row_index $site $x $height [orientation $height] "HORIZONTAL" $num $site_width
        incr row_index
    }

    proc write_opendb_rows {} {
        variable stripe_locs
        variable row_height
        variable row_index

        set row_index 1
        
        set stripes [concat $stripe_locs([get_rails_layer],POWER) $stripe_locs([get_rails_layer],GROUND)]
        set new_stripes {}
        foreach stripe $stripes {
            if {[lindex $stripe 0] != [lindex $stripe 2]} {
                lappend new_stripes $stripe
            }
        }
        set stripes $new_stripes
        set stripes [lsort -real -index 1 $stripes]
        set heights {}
        foreach stripe $stripes {
            lappend heights [lindex $stripe 1]
        }
        set heights [lsort -unique -real $heights]

        init_orientation [lindex $heights 0]

	foreach height [lrange $heights 0 end-1] {
            set rails {}
            foreach stripe $stripes {
                if {[lindex $stripe 1] == $height} {
                    lappend rails $stripe
                }
            }
            set lower_rails [lsort -real -index 2 $rails]
            set rails {}
            foreach stripe $stripes {
                if {[lindex $stripe 1] == ($height + $row_height)} {
                    lappend rails $stripe
                }
            }
            set upper_rails [lsort -real -index 2 $rails]
            set upper_extents {}
            foreach upper_rail $upper_rails {
                lappend upper_extents [lindex $upper_rail 0]
                lappend upper_extents [lindex $upper_rail 2]
            }

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
                    write_opendb_row $height $row_start $end
                    
                } else {
                    while {$idx < [llength $upper_extents] && $end > [lindex $upper_extents [expr $idx + 1]]} {
                        write_opendb_row $height $row_start [lindex $upper_extents $idx]
                        set row_start [lindex $upper_extents [expr $idx + 1]]
                        set idx [expr $idx + 2]
                    }
                }
            }
        }
    }

    namespace export write_opendb_vias write_opendb_specialnets write_opendb_rows    
}
