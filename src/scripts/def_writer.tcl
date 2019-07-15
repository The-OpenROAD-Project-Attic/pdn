namespace eval ::pdn {
    variable defOut
    
    proc open_def {file_name} {
        variable defOut
        set defOut [open $file_name w]
    }
    
    proc close_def {} {
        variable defOut
        close $defOut
    }
    
    proc def_out {args} {
        variable defOut
        
        if {[llength $args] == 2} {
            puts [lindex $args 0] $defOut [lindex $args 1]
        } else {
            puts $defOut [lindex $args 0]
        }
    }
    
    proc def_header {config} {
        set def_units [dict get $config def_units]
        
        def_out "###############################################################"
        def_out "# Created by apply_pdn script"
        def_out "# Created on: [clock format [clock seconds] -format {%A, the %d of %B, %Y}]"
        def_out "###############################################################"

        def_out "VERSION 5.8 ;"
        def_out "DIVIDERCHAR \"/\" ;"
        def_out "BUSBITCHARS \"\[\]\" ;"
        def_out "DESIGN [dict get $config design] ;"
        def_out "UNITS DISTANCE MICRONS $def_units ;"
        def_out ""

        set die_area_llx [expr [lindex [dict get $config die_area] 0] * $def_units]
        set die_area_lly [expr [lindex [dict get $config die_area] 1] * $def_units]
        set die_area_urx [expr [lindex [dict get $config die_area] 2] * $def_units]
        set die_area_ury [expr [lindex [dict get $config die_area] 3] * $def_units]

        def_out "PROPERTYDEFINITIONS 
            DESIGN FE_CORE_BOX_LL_X REAL [lindex [dict get $config core_area] 0] ;
            DESIGN FE_CORE_BOX_UR_X REAL [lindex [dict get $config core_area] 1] ;
            DESIGN FE_CORE_BOX_LL_Y REAL [lindex [dict get $config core_area] 2] ;
            DESIGN FE_CORE_BOX_UR_Y REAL [lindex [dict get $config core_area] 3] ;
        END PROPERTYDEFINITIONS"

        def_out ""
        def_out "DIEAREA ( [expr round($die_area_llx)] [expr round($die_area_lly)] ) ( [expr round($die_area_urx)] [expr round($die_area_ury)] ) ;"

        ##### Generating via rules
        generate_viarules
    }
    
    proc write_vias {net_name} {
        foreach via $::vias {
            if {[dict get $via net_name] == $net_name} {
                set connect [dict get $via connections]
                set i1 [dict get $connect layer1]
                set i2 [dict get $connect layer2]
                set viarules [dict get $connect rules]
                
                set intersections [dict get $connect intersections]

                set layer1 [dict get $connect layer1]
                regexp {(.*)_PIN} [dict get $connect layer1] - layer1 

                set layer2 [dict get $connect layer2]
                set i1 [lsearch $::met_layer_list $layer1]
                set i2 [lsearch $::met_layer_list $layer2]
                if {$i1 == -1} {puts "Layer1 [dict get $connect layer1], Layer2 $layer2"; exit -1}
                if {$i2 == -1} {puts "Layer1 [dict get $connect layer1], Layer2 $layer2"; exit -1}
	        # For each layer between l1 and l2, add vias at the intersection
	        for {set i $i1} {$i < $i2} {incr i} {
		    set lay [lindex $::met_layer_list $i]
		    set rule [lindex $viarules $i]
		    foreach loc $intersections {
		        def_out "    NEW $lay 0 + SHAPE STRIPE ( [expr round([lindex $loc 0])] [expr round([lindex $loc 1])] ) $rule"
		    }
	        }
            }
        }
    }

    ##proc to create fragments rows based on macros (blockages)

    proc init_orientation {height} {
        variable lowest_rail
        variable orient_rows
        
        set lowest_rail $height
        if {$::rails_start_with == "GROUND"} {
            set orient_rows {0 "N" 1 "FS"}
        } else {
            set orient_rows {0 "FS" 1 "N"}
        }
    }

    proc orientation {height} {
        variable lowest_rail
        variable orient_rows
        
        set row_line [expr int(($height - $lowest_rail) / $::row_height)]
        return [dict get $orient_rows [expr $row_line % 2]]
    }
        
    proc write_row {height start end} {
        set start  [expr int($start)]
        set height [expr int($height)]
        set end    [expr int($end)]

        if {$start == $end} {return}
	set site_width [expr {int(round($::site_width * $::def_units))}]
	if {[expr { int($start) % $site_width}] == 0} {
		set x $start

	} else {
		set offset [expr { int($start) % $site_width}]
		set x [expr {$start + $site_width - $offset}]

	}

	set num  [expr {($end - $x)/$site_width}]
        def_out "ROW ROW_$::row_index $::site_name $start $height [orientation $height] DO $num BY 1 STEP $site_width 0 ;"
        
        incr ::row_index
    }

    proc write_rows {} {
        set stripes [concat $::stripe_locs($::rails_mlayer,POWER) $::stripe_locs($::rails_mlayer,GROUND)]
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

    #proc to write special nets in the output def file, for each layer and each domains, 
    proc write_def {lay tag} {
        set dir [get_dir $lay]
        
	    if {$dir == "hor"} {
		    foreach l_str $::stripe_locs($lay,$tag) {
			    set l1 [expr round([lindex $l_str 0])]
			    set l2 [expr round([lindex $l_str 1])]
			    set l3 [expr round([lindex $l_str 2])]
                            if {$l1 == $l3} {continue}
			    if {$lay == $::rails_mlayer} {
				    if {$::seg_count == 1 } {
					    def_out "    $lay [expr round($::widths($lay))] + SHAPE FOLLOWPIN ( $l1 $l2 ) ( $l3 * )"
					    set ::seg_count 2
				    } else {
					    def_out "    NEW $lay [expr round($::widths($lay))] + SHAPE FOLLOWPIN ( $l1 $l2 ) ( $l3 * )"
					    set ::seg_count 2
				    }
			    } else {
				    if {$::seg_count == 1 } {
					    def_out "    $lay [expr round($::widths($lay))] + SHAPE STRIPE ( $l1 $l2 ) ( $l3 * )"
					    set ::seg_count 2
				    } else {
					    def_out "    NEW $lay [expr round($::widths($lay))] + SHAPE STRIPE ( $l1 $l2 ) ( $l3 * )"
					    set ::seg_count 2
				    }
			    }

		    }

	    } elseif {$dir == "ver"} {
		    foreach l_str $::stripe_locs($lay,$tag) {
			    set l1 [expr round([lindex $l_str 0])]
			    set l2 [expr round([lindex $l_str 1])]
			    set l3 [expr round([lindex $l_str 2])]
                            if {$l2 == $l3} {continue}
			    if {$lay == $::rails_mlayer} {
				    if {$::seg_count == 1 } {
					    def_out "    $lay [expr round($::widths($lay))] + SHAPE FOLLOWPIN ( $l1 $l2 ) ( * $l3 )"
					    set ::seg_count 2
				    } else {
					    def_out "    NEW $lay [expr round($::widths($lay))] + SHAPE FOLLOWPIN ( $l1 $l2 ) ( * $l3 )"	
					    set ::seg_count 2
				    }
			    } else {
				    if {$::seg_count == 1 } {
					    def_out "    $lay [expr round($::widths($lay))] + SHAPE STRIPE ( $l1 $l2 ) ( * $l3 )"
					    set ::seg_count 2
				    } else {
					    def_out "    NEW $lay [expr round($::widths($lay))] + SHAPE STRIPE ( $l1 $l2 ) ( * $l3 )"
					    set ::seg_count 2
				    }
			    }
		    }		

	    }


    }



    proc output_def {} {
        variable design_data
        
        #####
        # Start writing to the DEF file
        open_def [dict get $design_data config def_output]

        def_header [dict get $design_data config]

        ##### Generating the SPECIALNETS header

        set size_of_pg_nets [expr {[llength $::power_nets] + [llength $::ground_nets]}]
        def_out "\nSPECIALNETS $size_of_pg_nets ;"
        foreach net_name $::power_nets {
            set tag "POWER"
            def_out -nonewline "- $net_name  ( * $net_name )"
            foreach pin_name $::macro_power_pins {
                def_out -nonewline " ( * $pin_name )"
            }
            def_out ""
            
            def_out -nonewline "  + ROUTED " 

            set ::seg_count 1
            foreach lay $::met_layer_list {
                write_def $lay $tag	
            }
            write_vias $net_name
            def_out "  + USE $tag\n ;"
        }

        foreach net_name $::ground_nets {
            set tag "GROUND"
            def_out -nonewline "- $net_name  ( * $net_name )"
            foreach pin_name $::macro_ground_pins {
                def_out -nonewline " ( * $pin_name )"
            }
            def_out ""
            def_out -nonewline "  + ROUTED " 

            set ::seg_count 1
            foreach lay $::met_layer_list {
                write_def $lay $tag	
            }
            write_vias $net_name
            def_out "  + USE $tag\n ;"
        }
        def_out "\nEND SPECIALNETS"

        write_rows
        def_out "\nEND DESIGN"

        close_def

        puts "Total walltime = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds"
        puts [dict get $design_data config def_output]
        if {[catch {glob [dict get $design_data config def_output]}]} {
          puts "ERROR: Could not generate PDN DEF ..."
          puts "Exiting..."
          exit
        } else {
          puts "###PASSED PDN DEF creation ....."
        }
    }
    
    namespace export output_def def_out
}
