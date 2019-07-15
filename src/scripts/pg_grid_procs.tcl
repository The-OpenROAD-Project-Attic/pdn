namespace eval ::pdn {

#This file contains procedures that are used for PDN generation

proc get_dir {layer_name} {
    set idx [lsearch $::met_layer_list $layer_name]
    return [lindex $::met_layer_dir $idx]
}

## Proc to generate via locations, both for a normal via and stacked via
proc generate_via_stacks {l1 l2 tag grid_data} {
    set blockage [dict get $grid_data blockage]
    set area [dict get $grid_data area]
    
    #this variable contains locations of intersecting points of two orthogonal metal layers, between which via needs to be inserted
    #for every intersection. Here l1 and l2 are layer names, and i1 and i2 and their indices, tag represents domain (power or ground)	
    set intersections ""
    #check if layer pair is orthogonal, case 1
    set layer1 $l1
    if {[lsearch $::met_layer_list $layer1] != -1} {
        set layer1_direction [get_dir $layer1]
    } elseif {[regexp {(.*)_PIN_(hor|ver)} $l1 - layer1 layer1_direction]} {
        #
    } else {
        puts "Invalid direction for layer $l1"
    }
    set ignore_count 0
    
    if {$layer1_direction == "hor" && [get_dir $l2] == "ver"} {

        #loop over each stripe of layer 1 and layer 2 
	foreach l1_str $::orig_stripe_locs($l1,$tag) {
	    set a1  [expr {[lindex $l1_str 1]}]

	    foreach l2_str $::orig_stripe_locs($l2,$tag) {
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
		    set b1 [expr {[lindex $blk1 0] * $::def_units }]
		    set b2 [expr {[lindex $blk1 1] * $::def_units }]
		    set b3 [expr {[lindex $blk1 2] * $::def_units }]
		    set b4 [expr {[lindex $blk1 3] * $::def_units }]
		    ## Check if stripes are to be blocked on these blockages (blockages are specific to each layer). If yes, do not drop vias
		    if {  [lsearch $::macro_blockage_layer_list $l1] >= 0 || [lsearch $::macro_blockage_layer_list $l2] >= 0 } {
			if {($a2 > $b1 && $a2 < $b3 && $a1 > $b2 && $a1 < $b4 ) } {
			    set flag 0
			} 
			if {$a2 > $b1 && $a2 < $b3 && $a1 == $b2 && $a1 == [lindex $area 1]} {
			    set flag 0
			} 
			if {$a2 > $b1 && $a2 < $b3 && $a1 == $b4 && $a1 == [lindex $area 3]} {
			    set flag 0
			} 
		    }
		}

		if {$flag == 1} {
                    ## if no blockage restriction, append intersecting points to this "intersections"
		    lappend intersections "$a2 $a1"
		}
	    }
        }

    } elseif {$layer1_direction == "ver" && [get_dir $l2] == "hor"} {
        ##Second case of orthogonal intersection, similar criteria as above, but just flip of coordinates to find intersections
	foreach l1_str $::orig_stripe_locs($l1,$tag) {
	    set n1  [expr {[lindex $l1_str 0]}]
            
	    foreach l2_str $::orig_stripe_locs($l2,$tag) {
		set flag 1
		set n2	[expr {[lindex $l2_str 1]}]
                # Ignore if outside the area
                if {!($n1 >= [lindex $area 0] && $n1 <= [lindex $area 2] && $n2 >= [lindex $area 1] && $n2 <= [lindex $area 3])} {continue}
	        if {$n2 > [lindex $l1_str 2] || $n2 < [lindex $l1_str 1]} {continue}
	        if {$n1 > [lindex $l2_str 2] || $n1 < [lindex $l2_str 0]} {continue}
			
		foreach blk1 $blockage {
			set b1 [expr {[lindex $blk1 0] * $::def_units }]
			set b2 [expr {[lindex $blk1 1] * $::def_units }]
			set b3 [expr {[lindex $blk1 2] * $::def_units }]
			set b4 [expr {[lindex $blk1 3] * $::def_units }]
			if {  [lsearch $::macro_blockage_layer_list $l1] >= 0 || [lsearch $::macro_blockage_layer_list $l2] >= 0 } {
				if {($n1 >= $b1 && $n1 <= $b3 && $n2 >= $b2 && $n2 <= $b4)} {
					set flag 0	
				}
			}
		}

		if {$flag == 1} {
			lappend intersections "$n1 $n2"
		}


	    }
        }
    } else { 
	#Check if stripes have orthogonal intersections. If not, exit
	puts "ERROR: Adding vias between same direction layers is not supported yet. EXITING....."
	exit
    }

    return [list layer1 $l1 layer2 $l2 rules [dict get $grid_data vias] intersections $intersections]
}

# proc to generate follow pin layers or standard cell rails

proc generate_lower_metal_followpin_rails {tag area} {
	#Assumes horizontal stripes
	set lay $::rails_mlayer

	if {$tag == $::rails_start_with} { ;#If starting from bottom with this net, 
		set lly [lindex $area 1]
	} else {
		set lly [expr {[lindex $area 1] + $::row_height}]
	}
	lappend ::stripe_locs($lay,$tag) "[lindex $area 0] $lly [lindex $area 2]"
	lappend ::orig_stripe_locs($lay,$tag) "[lindex $area 0] $lly [lindex $area 2]"


	#Rail every alternate rows - Assuming horizontal rows and full width rails
	for {set y [expr {$lly + (2 * $::row_height)}]} {$y <= [lindex $area 3]} {set y [expr {$y + (2 * $::row_height)}]} {
	    lappend ::stripe_locs($lay,$tag) "[lindex $area 0] $y [lindex $area 2]"
	    lappend ::orig_stripe_locs($lay,$tag) "[lindex $area 0] $y [lindex $area 2]"
	}
}


# proc for creating pdn mesh for upper metal layers
proc generate_upper_metal_mesh_stripes {tag layer area} {
	if {[get_dir $layer] == "hor"} {
		set offset [expr [lindex $area 1] + $::boffset($layer)]
		if {$tag != $::stripes_start_with} { ;#If not starting from bottom with this net, 
			set offset [expr {$offset + ($::pitches($layer) / 2)}]
		}
		for {set y $offset} {$y < [expr {[lindex $area 3] - $::widths($layer)}]} {set y [expr {$::pitches($layer) + $y}]} {
			lappend ::stripe_locs($layer,$tag) "[lindex $area 0] $y [lindex $area 2]"
			lappend ::orig_stripe_locs($layer,$tag) "[lindex $area 0] $y [lindex $area 2]"
		
		}
	} elseif {[get_dir $layer] == "ver"} {
		set offset [expr [lindex $area 0] + $::loffset($layer)]

		if {$tag != $::stripes_start_with} { ;#If not starting from bottom with this net, 
			set offset [expr {$offset + ($::pitches($layer) / 2)}]
		}
		for {set x $offset} {$x < [expr {[lindex $area 2] - $::widths($layer)}]} {set x [expr {$::pitches($layer) + $x}]} {
			lappend ::stripe_locs($layer,$tag) "$x [lindex $area 1] [lindex $area 3]"
			lappend ::orig_stripe_locs($layer,$tag) "$x [lindex $area 1] [lindex $area 3]"
		}
	} else {
		puts "ERROR: Invalid direction \"[get_dir $layer]\" for metal layer ${layer}. Should be either \"hor\" or \"ver\". EXITING....."
		exit
	}
}

# this proc chops down metal stripes wherever they are to be blocked
# inputs to this proc are layer name, domain (tag), and blockage bbox cooridnates

proc generate_metal_with_blockage {layer area tag b1 b2 b3 b4} {
	set ::temp_locs($layer,$tag) ""
	set ::temp_locs($layer,$tag) $::stripe_locs($layer,$tag)
	set ::stripe_locs($layer,$tag) ""
	foreach l_str $::temp_locs($layer,$tag) {
		set loc1 [lindex $l_str 0]
		set loc2 [lindex $l_str 1]
		set loc3 [lindex $l_str 2]
		location_stripe_blockage $loc1 $loc2 $loc3 $layer $area $tag $b1 $b2 $b3 $b4
	}
		
        set ::stripe_locs($layer,$tag) [lsort -unique $::stripe_locs($layer,$tag)]
}

# sub proc called from previous proc
proc location_stripe_blockage {loc1 loc2 loc3 lay area tag b1 b2 b3 b4} {
        set area_llx [lindex $area 0]
        set area_lly [lindex $area 1]
        set area_urx [lindex $area 2]
        set area_ury [lindex $area 3]

	if {[get_dir $lay] == "hor"} {
		##Check if stripe is passing through blockage
		##puts "HORIZONTAL BLOCKAGE "
		set x1 $loc1
		set y1 [expr max($loc2 - $::widths($lay)/2, [lindex $area 1])]
		set x2 $loc3
		set y2 [expr min($y1 +  $::widths($lay),[lindex $area 3])]
                #puts "segment:  [format {%9.1f %9.1f} $loc1 $loc3]"              
                #puts "blockage: [format {%9.1f %9.1f} $b1 $b3]"
		if {  ($y1 >= $b2) && ($y2 <= $b4) && ( ($x1 <= $b3 && $x2 >= $b3) || ($x1 <= $b1 && $x2 >= $b1)  || ($x1 <= $b1 && $x2 >= $b3) || ($x1 <= $b3 && $x2 >= $b1) )  } {

			if {$x1 <= $b1 && $x2 >= $b3} {	
				#puts "  CASE3 of blockage in between left and right edge of core, cut the stripe into two segments"
                                #puts "    $x1 $loc2 $b1"
                                #puts "    $b3 $loc2 $x2"
				lappend ::stripe_locs($lay,$tag) "$x1 $loc2 $b1"
				lappend ::stripe_locs($lay,$tag) "$b3 $loc2 $x2"	
			} elseif {$x1 <= $b3 && $x2 >= $b3} {	
				#puts "  CASE3 of blockage in between left and right edge of core, but stripe extending out only in one side (right)"
                                #puts "    $b3 $loc2 $x2"
				lappend ::stripe_locs($lay,$tag) "$b3 $loc2 $x2"	
			} elseif {$x1 <= $b1 && $x2 >= $b1} {	
				#puts "  CASE3 of blockage in between left and right edge of core, but stripe extending out only in one side (left)"
                                #puts "    $x1 $loc2 $b1"
				lappend ::stripe_locs($lay,$tag) "$x1 $loc2 $b1"
			} else {
                            #puts "  CASE5 no match - eliminated segment"
                            #puts "    $loc1 $loc2 $loc3"
                        }
 
		} else {
			lappend ::stripe_locs($lay,$tag) "$x1 $loc2 $x2"
			#puts "stripe does not pass thru any layer blockage --- CASE 4 (do not change the stripe location)"
		}
	}

	if {[get_dir $lay] == "ver"} {
		##Check if veritcal stripe is passing through blockage, same strategy as above
		set x1 $loc1
		set y1 $loc2
		set y2 $loc3
		set x2 [expr { $x1 +  $::widths($lay) }]
		if { ($x1 >= $b1)   && ($x2 <= $b3) && ( ($y1 <= $b2 && $y2 >= $b2) || ($y1 <= $b4 && $y2 >= $b4)  || ($y1 <= $b2 && $y2 >= $b4) || ($y1 <= $b4 && $y2 >= $b2)   ) } {

			if {$y1 <= $b2 && $y2 >= $b4} {	
				##puts "CASE3 of blockage in between top and bottom edge of core, cut the stripe into two segments
				lappend ::stripe_locs($lay,$tag) "$x1 $y1 $b2"
				lappend ::stripe_locs($lay,$tag) "$x1 $b4 $y2"	
			}
			if {$y1 <= $b4 && $y2 >= $b4} {	
				##puts "CASE3 of blockage in between top and bottom edge of core, but stripe extending out only in one side (right)"
				lappend ::stripe_locs($lay,$tag) "$x1 $b4 $y2"	
			}
			if {$y1 <= $b2 && $y2 >= $b2} {	
				##puts "CASE3 of blockage in between top and bottom edge of core, but stripe extending out only in one side (left)"
				lappend ::stripe_locs($lay,$tag) "$x1 $y1 $b2"
			} else {
                            #puts "  CASE5 no match - eliminated segment"
                            #puts "    $loc1 $loc2 $loc3"
                        }



 
		} else {
			lappend ::stripe_locs($lay,$tag) "$x1 $y1 $y2"
		}
	}
}


## this is a top-level proc to generate PDN stripes and insert vias between these stripes
proc generate_stripes_vias {tag net_name grid_data} {

        set area [dict get $grid_data area]
        set blockage [dict get $grid_data blockage]
        
	##puts -nonewline "Adding stripes for $net_name ..."
	foreach lay [dict get $grid_data layers] {

	    if {$lay == $::rails_mlayer} {
	        #Std. cell rails
	        generate_lower_metal_followpin_rails $tag $area

	        foreach blk1 $blockage {
		        set b1 [expr {[lindex $blk1 0] * $::def_units }]
		        set b2 [expr {[lindex $blk1 1] * $::def_units }]
		        set b3 [expr {[lindex $blk1 2] * $::def_units }]
		        set b4 [expr {[lindex $blk1 3] * $::def_units }]
		        generate_metal_with_blockage $::rails_mlayer $area $tag $b1 $b2 $b3 $b4
	        }

            } else {
	        #Upper layer stripes
		if {$::widths($lay) != [expr {-1 * $::def_units}] || $::pitches($lay) != [expr {-1 * $::def_units}]} {
			generate_upper_metal_mesh_stripes $tag $lay $area
		}

		if {  [lsearch $::macro_blockage_layer_list $lay] >= 0 } {
			foreach blk2 $blockage {
				set c1 [expr {[lindex $blk2 0] * $::def_units }]
				set c2 [expr {[lindex $blk2 1] * $::def_units }]
				set c3 [expr {[lindex $blk2 2] * $::def_units }]
				set c4 [expr {[lindex $blk2 3] * $::def_units }]

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
		lappend ::vias [list net_name $net_name connections $connections]
	}
	##puts " DONE \[Total elapsed walltime = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds\]"

}

    namespace export write_def write_vias
}
