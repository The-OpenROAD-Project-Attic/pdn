namespace eval ::pdn {

    variable logical_viarules {}
    variable vias {}
    variable stripe_locs
    variable orig_stripe_locs
    variable layers {}
    
#This file contains procedures that are used for PDN generation

    proc get_dir {layer_name} {
        variable metal_layers 
        variable metal_layers_dir 
        if {[regexp {.*_PIN_(hor|ver)} $layer_name - dir]} {
            return $dir
        }
        
        set idx [lsearch [get_metal_layers] $layer_name]
        if {[lindex $metal_layer_dirs $idx] == "HORIZONTAL"} {
            return "hor"
        } else {
            return "ver"
        }
    }
    
    proc get_rails_layer {} {
        variable default_grid_data
        
        return [dict get $default_grid_data rails]
    }

    proc init_via_tech {} {
        variable tech
        variable def_via_tech
        
        set def_via_tech {}
        foreach via_rule [$tech getViaGenerateRules] {
            set lower [$via_rule getViaLayerRule 0]
            set upper [$via_rule getViaLayerRule 1]
            set cut   [$via_rule getViaLayerRule 2]

	    dict set def_via_tech [$via_rule getName] [list \
                lower [list layer [[$lower getLayer] getName] enclosure [$lower getEnclosure]] \
                upper [list layer [[$upper getLayer] getName] enclosure [$upper getEnclosure]] \
                cut   [list layer [[$cut getLayer] getName] spacing [$cut getSpacing] size [list [[$cut getRect] dx] [[$cut getRect] dy]]] \
            ]
        }
    }
    
    proc select_viainfo {lower} {
        variable def_via_tech

        set layer_name $lower
        regexp {(.*)_PIN} $lower - layer_name
        
        return [dict filter $def_via_tech script {rule_name rule} {expr {[dict get $rule lower layer] == $layer_name}}]
    }
    
    proc set_layer_info {layer_info} {
        variable layers
        
        set layers $layer_info
    }
    
    proc get_layer_info {} {
        variable layers 
        
        return $layers
    }
    
    # Layers that have a widthtable will only support some width values, the widthtable defines the 
    # set of widths that are allowed, or any width greater than or equal to the last value in the
    # table
    proc get_adjusted_width {layer width} {
        set layers [get_layer_info]
        
        if {[dict exists $layers $layer] && [dict exists $layers $layer widthtable]} {
            set widthtable [dict get $layers $layer widthtable]
            if {[lsearch $widthtable $width] > 0} {
                return $width
            } elseif {$width > [lindex $widthtable end]} {
                return $width
            } else {
                foreach value $widthtable {
                    if {$value > $width} {
                        return $value
                    }
                }
            }
        }
        
        return $width
    }
    
    # Given the via rule expressed in via_info, what is the via with the largest cut area that we can make
    proc get_via_option {lower_dir rule_name via_info x y width height} {
        set cut_width  [lindex [dict get $via_info cut size] 0]
        set cut_height [lindex [dict get $via_info cut size] 1]

        # Adjust the width and height values to the next largest allowed value if necessary
        set lower_width  [get_adjusted_width [dict get $via_info lower layer] $width]
        set lower_height [get_adjusted_width [dict get $via_info lower layer] $height]
        set upper_width  [get_adjusted_width [dict get $via_info upper layer] $width]
        set upper_height [get_adjusted_width [dict get $via_info upper layer] $height]
        
        set lower_enclosure [expr min([join [dict get $via_info lower enclosure] ","])]
        set upper_enclosure [expr min([join [dict get $via_info upper enclosure] ","])]
        set max_lower_enclosure [expr max([join [dict get $via_info lower enclosure] ","])]
        set max_upper_enclosure [expr max([join [dict get $via_info upper enclosure] ","])]

        # What are the maximum number of rows and columns that we can fit in this space?
        set i 0
        set via_width_lower 0
        set via_width_upper 0
        while {$via_width_lower < $lower_width && $via_width_upper < $upper_width} {
            incr i
            set xcut_pitch [lindex [dict get $via_info cut spacing] 0]
            set via_width_lower [expr $cut_width + $xcut_pitch * ($i - 1) + 2 * $lower_enclosure]
            set via_width_upper [expr $cut_width + $xcut_pitch * ($i - 1) + 2 * $upper_enclosure]
        }
        set xcut_spacing [expr $xcut_pitch - $cut_width]
        set columns [expr $i - 1]

        set i 0
        set via_height_lower 0
        set via_height_upper 0
        while {$via_height_lower < $lower_height && $via_height_upper < $upper_height} {
            incr i
            set ycut_pitch [lindex [dict get $via_info cut spacing] 1]
            set via_height_lower [expr $cut_height + $ycut_pitch * ($i - 1) + 2 * $lower_enclosure]
            set via_height_upper [expr $cut_height + $ycut_pitch * ($i - 1) + 2 * $upper_enclosure]
        }
        set ycut_spacing [expr $ycut_pitch - $cut_height]
        set rows [expr $i - 1]

	set lower_enc_width  [expr round(($lower_width  - ($cut_width   + $xcut_pitch * ($columns - 1))) / 2)]
	set lower_enc_height [expr round(($lower_height - ($cut_height  + $ycut_pitch * ($rows    - 1))) / 2)]
	set upper_enc_width  [expr round(($upper_width  - ($cut_width   + $xcut_pitch * ($columns - 1))) / 2)]
	set upper_enc_height [expr round(($upper_height - ($cut_height  + $ycut_pitch * ($rows    - 1))) / 2)]

        # Adjust calculated via width values to ensure that an allowed size is generated
        set lower_size_max_enclosure [get_adjusted_width [dict get $via_info lower layer] [expr round(($cut_width   + $xcut_pitch * ($columns - 1) + $max_lower_enclosure * 2))]]
        set upper_size_max_enclosure [get_adjusted_width [dict get $via_info upper layer] [expr round(($cut_width   + $xcut_pitch * ($columns - 1) + $max_upper_enclosure * 2))]]
        
        set max_lower_enclosure [expr round(($lower_size_max_enclosure  - ($cut_width   + $xcut_pitch * ($columns - 1))) / 2)]
        set max_upper_enclosure [expr round(($upper_size_max_enclosure  - ($cut_width   + $xcut_pitch * ($columns - 1))) / 2)]

        # Use the largest value of enclosure in the direction of the layer
        # Use the smallest value of enclosure perpendicular to direction of the layer
	if {$lower_dir == "hor"} {
            if {$lower_enc_height < $max_lower_enclosure} {
                set xBotEnc [expr max($max_lower_enclosure,$lower_enc_width)]
            } else {
                set xBotEnc $lower_enc_width
            }
            set yBotEnc $lower_enc_height
        } else {
            set xBotEnc $lower_enc_width
            if {$lower_enc_width < $max_lower_enclosure} {
                set yBotEnc [expr max($max_lower_enclosure,$lower_enc_height)]
            } else {
                set yBotEnc $lower_enc_height
            }
        }

        # Use the largest value of enclosure in the direction of the layer
        # Use the smallest value of enclosure perpendicular to direction of the layer
	if {[get_dir [dict get $via_info upper layer]] == "hor"} {
            if {$upper_enc_height < $max_upper_enclosure} {
                set xTopEnc [expr max($max_upper_enclosure,$upper_enc_width)]
            } else {
                set xTopEnc $upper_enc_width
            }
            set yTopEnc $upper_enc_height
        } else {
            set xTopEnc $upper_enc_width
            if {$upper_enc_width < $max_upper_enclosure} {
                set yTopEnc [expr max($max_upper_enclosure,$upper_enc_height)]
            } else {
                set yTopEnc $upper_enc_height
            }
        }
        
        set rule [list \
            rule $rule_name \
            cutsize [dict get $via_info cut size] \
            layers [list [dict get $via_info lower layer] [dict get $via_info cut layer] [dict get $via_info upper layer]] \
            cutspacing [lmap spacing [dict get $via_info cut spacing] size [dict get $via_info cut size] {expr $spacing - $size}] \
            rowcol [list $rows $columns] \
            enclosure [list $xBotEnc $yBotEnc $xTopEnc $yTopEnc] \
        ]
        
        return $rule
    }
    
    proc get_viarule_name {lower x y width height} {
        set rules [select_viainfo $lower]
        set first_key [lindex [dict keys $rules] 0]
        set cut_layer [dict get $rules $first_key cut layer]

        return ${cut_layer}_${width}x${height}
    }
    
    proc get_cut_area {rule} {
        return [expr [lindex [dict get $rule rowcol] 0] * [lindex [dict get $rule rowcol] 0] * [lindex [dict get $rule cutsize] 0] * [lindex [dict get $rule cutsize] 1]]
    }
    
    proc select_rule {rule1 rule2} {
        if {[get_cut_area $rule2] > [get_cut_area $rule1]} {
            return $rule2
        }
        return $rule1
    }
    
    proc get_via {lower x y width height} {
        # First cur will assume that all crossing points (x y) are on grid for both lower and upper layers
        # TODO: Refine the algorithm to cope with offgrid intersection points
        variable physical_viarules
        
        set rule_name [get_viarule_name $lower $x $y $width $height]

        if {![dict exists $physical_viarules $rule_name]} {
            set selected_rule {}

            dict for {name rule} [select_viainfo $lower] {
                set result [get_via_option [get_dir $lower] $name $rule $x $y $width $height]
                if {$selected_rule == {}} {
                    set selected_rule $result
                } else {
                    # Choose the best between selected rule and current result, the winner becomes the new selected rule
                    set selected_rule [select_rule $selected_rule $result]
                }
            }

            dict set physical_viarules $rule_name $selected_rule
        }        
        
        return $rule_name
    }
    
    proc generate_vias {layer1 layer2 intersections} {
        variable logical_viarules
        variable physical_viarules

        set vias {}
        set layer1_name $layer1
        set layer2_name $layer2
        regexp {(.*)_PIN_(hor|ver)} $layer1 - layer1_name layer1_direction
        
        set i1 [lsearch [get_metal_layers] $layer1_name]
        set i2 [lsearch [get_metal_layers] $layer2_name]
        if {$i1 == -1} {puts "Layer1 [dict get $connect layer1], Layer2 $layer2"; exit -1}
        if {$i2 == -1} {puts "Layer1 [dict get $connect layer1], Layer2 $layer2"; exit -1}

	# For each layer between l1 and l2, add vias at the intersection
        foreach intersection $intersections {
            if {![dict exists $logical_viarules [dict get $intersection rule]]} {
                puts "Missing key [dict get $intersection rule]"
                puts "Available keys [dict keys $logical_viarules]"
                exit -1
            }
            set logical_rule [dict get $logical_viarules [dict get $intersection rule]]

            set x [dict get $intersection x]
            set y [dict get $intersection y]
            set width  [dict get $logical_rule width]
            set height  [dict get $logical_rule height]
            
            set connection_layers [list $layer1 {*}[lrange [get_metal_layers] [expr $i1 + 1] [expr $i2 - 1]]]
	    foreach lay $connection_layers {
                set via_name [get_via $lay $x $y $width $height]

                lappend vias [list name $via_name lower_layer $lay x [expr round([dict get $intersection x])] y [expr round([dict get $intersection y])]]
	    }
	}
                
        return $vias
    }

## Proc to generate via locations, both for a normal via and stacked via
proc generate_via_stacks {l1 l2 tag grid_data} {
    variable logical_viarules
    variable default_grid_data
    variable orig_stripe_locs
    variable def_units
    
    set blockage [dict get $grid_data blockage]
    set area [dict get $grid_data area]
    
    #this variable contains locations of intersecting points of two orthogonal metal layers, between which via needs to be inserted
    #for every intersection. Here l1 and l2 are layer names, and i1 and i2 and their indices, tag represents domain (power or ground)	
    set intersections ""
    #check if layer pair is orthogonal, case 1
    set layer1 $l1
    if {[dict exists $grid_data layers $layer1]} {
        set layer1_direction [get_dir $layer1]
        set layer1_width [dict get $grid_data layers $layer1 width]
        set layer1_width [expr round($layer1_width * $def_units)]
    } elseif {[regexp {(.*)_PIN_(hor|ver)} $l1 - layer1 layer1_direction]} {
        #
    } else {
        puts "Invalid direction for layer $l1"
    }
    
    set layer2 $l2
    if {[dict exists $grid_data layers $layer2]} {
        set layer2_width [dict get $grid_data layers $layer2 width]
        set layer2_width [expr round($layer2_width * $def_units)]
    } elseif {[dict exists $default_grid_data layers $layer2]} {
        set layer2_width [dict get $default_grid_data layers $layer2 width]
        set layer2_width [expr round($layer2_width * $def_units)]
    } else {
        puts "No width information available for layer $layer2"
    }
    
    set ignore_count 0
    
    if {$layer1_direction == "hor" && [get_dir $l2] == "ver"} {

        #loop over each stripe of layer 1 and layer 2 
	foreach l1_str $orig_stripe_locs($l1,$tag) {
	    set a1  [expr {[lindex $l1_str 1]}]

	    foreach l2_str $orig_stripe_locs($l2,$tag) {
		set flag 1
		set a2	[expr {[lindex $l2_str 0]}]

                # Ignore if outside the area
                if {!($a2 >= [lindex $area 0] && $a2 <= [lindex $area 2] && $a1 >= [lindex $area 1] && $a1 <= [lindex $area 3])} {continue}
	        if {$a2 > [lindex $l1_str 2] || $a2 < [lindex $l1_str 0]} {continue}
	        if {$a1 > [lindex $l2_str 2] || $a1 < [lindex $l2_str 1]} {continue}

                if {[lindex $l2_str 1] == [lindex $area 3]} {continue}
                if {[lindex $l2_str 2] == [lindex $area 1]} {continue}

                #loop over each blockage geometry (macros are blockages)
		foreach blk1 $blockage {
		    set b1 [get_instance_llx $blk1]
		    set b2 [get_instance_lly $blk1]
		    set b3 [get_instance_urx $blk1]
		    set b4 [get_instance_ury $blk1]
		    ## Check if stripes are to be blocked on these blockages (blockages are specific to each layer). If yes, do not drop vias
		    if {  [lsearch [get_macro_blockage_layers $blk1] $l1] >= 0 || [lsearch [get_macro_blockage_layers $blk1] $l2] >= 0 } {
			if {($a2 > $b1 && $a2 < $b3 && $a1 > $b2 && $a1 < $b4 ) } {
			    set flag 0
                            break
			} 
			if {$a2 > $b1 && $a2 < $b3 && $a1 == $b2 && $a1 == [lindex $area 1]} {
			    set flag 0
                            break
			} 
			if {$a2 > $b1 && $a2 < $b3 && $a1 == $b4 && $a1 == [lindex $area 3]} {
			    set flag 0
                            break
			} 
		    }
		}

		if {$flag == 1} {
                    ## if no blockage restriction, append intersecting points to this "intersections"
                    if {[regexp {.*_PIN_(hor|ver)} $l1 - dir]} {
                        set layer1_width [lindex $l1_str 3] ; # Already in def units
                    }
                    set rule_name ${l1}${layer2}_${layer2_width}x${layer1_width}
                    if {![dict exists $logical_viarules $rule_name]} {
                        dict set logical_viarules $rule_name [list lower $l1 upper $layer2 width ${layer2_width} height ${layer1_width}]
                    }
		    lappend intersections "rule $rule_name x $a2 y $a1"
		}
	    }
        }

    } elseif {$layer1_direction == "ver" && [get_dir $l2] == "hor"} {
        ##Second case of orthogonal intersection, similar criteria as above, but just flip of coordinates to find intersections
	foreach l1_str $orig_stripe_locs($l1,$tag) {
	    set n1  [expr {[lindex $l1_str 0]}]
            
	    foreach l2_str $orig_stripe_locs($l2,$tag) {
		set flag 1
		set n2	[expr {[lindex $l2_str 1]}]
                
                # Ignore if outside the area
                if {!($n1 >= [lindex $area 0] && $n1 <= [lindex $area 2] && $n2 >= [lindex $area 1] && $n2 <= [lindex $area 3])} {continue}
	        if {$n2 > [lindex $l1_str 2] || $n2 < [lindex $l1_str 1]} {continue}
	        if {$n1 > [lindex $l2_str 2] || $n1 < [lindex $l2_str 0]} {continue}
			
		foreach blk1 $blockage {
			set b1 [get_instance_llx $blk1]
			set b2 [get_instance_lly $blk1]
			set b3 [get_instance_urx $blk1]
			set b4 [get_instance_ury $blk1]
			if {  [lsearch [get_macro_blockage_layers $blk1] $l1] >= 0 || [lsearch [get_macro_blockage_layers $blk1] $l2] >= 0 } {
				if {($n1 >= $b1 && $n1 <= $b3 && $n2 >= $b2 && $n2 <= $b4)} {
					set flag 0	
				}
			}
		}

		if {$flag == 1} {
                        ## if no blockage restriction, append intersecting points to this "intersections"
                        if {[regexp {.*_PIN_(hor|ver)} $l1 - dir]} {
                            set layer1_width [lindex $l1_str 3] ; # Already in def units
                        }
                        set rule_name ${l1}${layer2}_${layer1_width}x${layer2_width}
                        if {![dict exists $logical_viarules $rule_name]} {
                            dict set logical_viarules $rule_name [list lower $l1 upper $layer2 width ${layer1_width} height ${layer2_width}]
                        }
			lappend intersections "rule $rule_name x $n1 y $n2"
		}


	    }
        }
    } else { 
	#Check if stripes have orthogonal intersections. If not, exit
	puts "ERROR: Adding vias between same direction layers is not supported yet."
        puts "Layer: $l1, Direction: $layer1_direction"
        puts "Layer: $l2, Direction: [get_dir $l2]"
	exit
    }

    return [generate_vias $l1 $l2 $intersections]
}

# proc to generate follow pin layers or standard cell rails

proc generate_lower_metal_followpin_rails {tag area} {
    variable orig_stripe_locs
    variable stripe_locs

	#Assumes horizontal stripes
	set lay [get_rails_layer]

	if {$tag == $::rails_start_with} { ;#If starting from bottom with this net, 
		set lly [lindex $area 1]
	} else {
		set lly [expr {[lindex $area 1] + $::row_height}]
	}
	lappend stripe_locs($lay,$tag) "[lindex $area 0] $lly [lindex $area 2]"
	lappend orig_stripe_locs($lay,$tag) "[lindex $area 0] $lly [lindex $area 2]"


	#Rail every alternate rows - Assuming horizontal rows and full width rails
	for {set y [expr {$lly + (2 * $::row_height)}]} {$y <= [lindex $area 3]} {set y [expr {$y + (2 * $::row_height)}]} {
	    lappend stripe_locs($lay,$tag) "[lindex $area 0] $y [lindex $area 2]"
	    lappend orig_stripe_locs($lay,$tag) "[lindex $area 0] $y [lindex $area 2]"
	}
}


# proc for creating pdn mesh for upper metal layers
proc generate_upper_metal_mesh_stripes {tag layer area} {
    variable widths
    variable pitches
    variable loffset
    variable boffset
    variable orig_stripe_locs
    variable stripe_locs

	if {[get_dir $layer] == "hor"} {
		set offset [expr [lindex $area 1] + $boffset($layer)]
		if {$tag != $::stripes_start_with} { ;#If not starting from bottom with this net, 
			set offset [expr {$offset + ($pitches($layer) / 2)}]
		}
		for {set y $offset} {$y < [expr {[lindex $area 3] - $widths($layer)}]} {set y [expr {$pitches($layer) + $y}]} {
			lappend stripe_locs($layer,$tag) "[lindex $area 0] $y [lindex $area 2]"
			lappend orig_stripe_locs($layer,$tag) "[lindex $area 0] $y [lindex $area 2]"
		}
	} elseif {[get_dir $layer] == "ver"} {
		set offset [expr [lindex $area 0] + $loffset($layer)]

		if {$tag != $::stripes_start_with} { ;#If not starting from bottom with this net, 
			set offset [expr {$offset + ($pitches($layer) / 2)}]
		}
		for {set x $offset} {$x < [expr {[lindex $area 2] - $widths($layer)}]} {set x [expr {$pitches($layer) + $x}]} {
			lappend stripe_locs($layer,$tag) "$x [lindex $area 1] [lindex $area 3]"
			lappend orig_stripe_locs($layer,$tag) "$x [lindex $area 1] [lindex $area 3]"
		}
	} else {
		puts "ERROR: Invalid direction \"[get_dir $layer]\" for metal layer ${layer}. Should be either \"hor\" or \"ver\". EXITING....."
		exit
	}
}

# this proc chops down metal stripes wherever they are to be blocked
# inputs to this proc are layer name, domain (tag), and blockage bbox cooridnates

proc generate_metal_with_blockage {layer area tag b1 b2 b3 b4} {
    variable stripe_locs
	set temp_locs($layer,$tag) ""
	set temp_locs($layer,$tag) $stripe_locs($layer,$tag)
	set stripe_locs($layer,$tag) ""
	foreach l_str $temp_locs($layer,$tag) {
		set loc1 [lindex $l_str 0]
		set loc2 [lindex $l_str 1]
		set loc3 [lindex $l_str 2]
		location_stripe_blockage $loc1 $loc2 $loc3 $layer $area $tag $b1 $b2 $b3 $b4
	}
		
        set stripe_locs($layer,$tag) [lsort -unique $stripe_locs($layer,$tag)]
}

# sub proc called from previous proc
proc location_stripe_blockage {loc1 loc2 loc3 lay area tag b1 b2 b3 b4} {
    variable widths
    variable stripe_locs

        set area_llx [lindex $area 0]
        set area_lly [lindex $area 1]
        set area_urx [lindex $area 2]
        set area_ury [lindex $area 3]

	if {[get_dir $lay] == "hor"} {
		##Check if stripe is passing through blockage
		##puts "HORIZONTAL BLOCKAGE "
		set x1 $loc1
		set y1 [expr max($loc2 - $widths($lay)/2, [lindex $area 1])]
		set x2 $loc3
		set y2 [expr min($y1 +  $widths($lay),[lindex $area 3])]
                #puts "segment:  [format {%9.1f %9.1f} $loc1 $loc3]"              
                #puts "blockage: [format {%9.1f %9.1f} $b1 $b3]"
		if {  ($y1 >= $b2) && ($y2 <= $b4) && ( ($x1 <= $b3 && $x2 >= $b3) || ($x1 <= $b1 && $x2 >= $b1)  || ($x1 <= $b1 && $x2 >= $b3) || ($x1 <= $b3 && $x2 >= $b1) )  } {

			if {$x1 <= $b1 && $x2 >= $b3} {	
				#puts "  CASE3 of blockage in between left and right edge of core, cut the stripe into two segments"
                                #puts "    $x1 $loc2 $b1"
                                #puts "    $b3 $loc2 $x2"
				lappend stripe_locs($lay,$tag) "$x1 $loc2 $b1"
				lappend stripe_locs($lay,$tag) "$b3 $loc2 $x2"	
			} elseif {$x1 <= $b3 && $x2 >= $b3} {	
				#puts "  CASE3 of blockage in between left and right edge of core, but stripe extending out only in one side (right)"
                                #puts "    $b3 $loc2 $x2"
				lappend stripe_locs($lay,$tag) "$b3 $loc2 $x2"	
			} elseif {$x1 <= $b1 && $x2 >= $b1} {	
				#puts "  CASE3 of blockage in between left and right edge of core, but stripe extending out only in one side (left)"
                                #puts "    $x1 $loc2 $b1"
				lappend stripe_locs($lay,$tag) "$x1 $loc2 $b1"
			} else {
                            #puts "  CASE5 no match - eliminated segment"
                            #puts "    $loc1 $loc2 $loc3"
                        }
		} else {
			lappend stripe_locs($lay,$tag) "$x1 $loc2 $x2"
			#puts "stripe does not pass thru any layer blockage --- CASE 4 (do not change the stripe location)"
		}
	}

	if {[get_dir $lay] == "ver"} {
		##Check if veritcal stripe is passing through blockage, same strategy as above
		set x1 $loc1 ;# [expr max($loc1 -  $widths($lay)/2, [lindex $area 0])]
		set y1 $loc2
		set x2 $loc1 ;# [expr min($loc1 +  $widths($lay)/2, [lindex $area 2])]
		set y2 $loc3

		if {$x2 > $b1 && $x1 < $b3} {

			if {$y1 <= $b2 && $y2 >= $b4} {	
				##puts "CASE3 of blockage in between top and bottom edge of core, cut the stripe into two segments
				lappend stripe_locs($lay,$tag) "$loc1 $y1 $b2"
				lappend stripe_locs($lay,$tag) "$loc1 $b4 $y2"	
			} elseif {$y1 <= $b4 && $y2 >= $b4} {	
				##puts "CASE3 of blockage in between top and bottom edge of core, but stripe extending out only in one side (right)"
				lappend stripe_locs($lay,$tag) "$loc1 $b4 $y2"	
			} elseif {$y1 <= $b2 && $y2 >= $b2} {	
				##puts "CASE3 of blockage in between top and bottom edge of core, but stripe extending out only in one side (left)"
				lappend stripe_locs($lay,$tag) "$loc1 $y1 $b2"
			} elseif {$y1 <= $b4 && $y1 >= $b2 && $y2 >= $b2 && $y2 <= $b4} {	
                                ##completely enclosed - remove segment
			} else {
                            #puts "  CASE5 no match"
                            #puts "    $loc1 $loc2 $loc3"
			    lappend stripe_locs($lay,$tag) "$loc1 $y1 $y2"
                        }
		} else {
			lappend stripe_locs($lay,$tag) "$loc1 $y1 $y2"
		}
	}
}


## this is a top-level proc to generate PDN stripes and insert vias between these stripes
proc generate_stripes_vias {tag net_name grid_data} {
        variable vias
        
        set area [dict get $grid_data area]
        set blockage [dict get $grid_data blockage]

	##puts -nonewline "Adding stripes for $net_name ..."
	foreach lay [dict keys [dict get $grid_data layers]] {

	    if {$lay == [get_rails_layer]} {
	        #Std. cell rails
	        generate_lower_metal_followpin_rails $tag $area

	        foreach blk1 $blockage {
		        set b1 [get_instance_llx $blk1]
		        set b2 [get_instance_lly $blk1]
		        set b3 [get_instance_urx $blk1]
		        set b4 [get_instance_ury $blk1]
		        generate_metal_with_blockage [get_rails_layer] $area $tag $b1 $b2 $b3 $b4
	        }

            } else {
	        #Upper layer stripes
		generate_upper_metal_mesh_stripes $tag $lay $area

		foreach blk2 $blockage {
		        if {  [lsearch [get_macro_blockage_layers $blk2] $lay] >= 0 } {
				set c1 [get_instance_llx $blk2]
				set c2 [get_instance_lly $blk2]
				set c3 [get_instance_urx $blk2]
				set c4 [get_instance_ury $blk2]

				generate_metal_with_blockage $lay $area $tag $c1 $c2 $c3 $c4
			}
		}
	    }
	}

	#Via stacks
	##puts -nonewline "Adding vias for $net_name ..."
	foreach tuple [dict get $grid_data connect] {
		set l1 [lindex $tuple 0]
		set l2 [lindex $tuple 1]

                set connections [generate_via_stacks $l1 $l2 $tag $grid_data]
		lappend vias [list net_name $net_name connections $connections]
	}
}

    namespace export write_def write_vias
}
