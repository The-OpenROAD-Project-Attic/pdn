#
############################### Instructions #################################
# To generate the DEF, enter all details in inputs.tcl file
# Run 'tclsh create_pg_grid_v1.tcl'
# DEF with power grid will be dumped out with the given name
# Contact umallapp@ucsd.edu for any questions
##############################################################################

############################### To be improved ###############################
# 1. Currently, generate_lower_metal_followpin_rails() handles only 1 power 
#    net and 1 ground net
# 2. Need to add support for designs with macros and rectilinear floorplans.
# 3. Currently supports only one width and pitch per layer, for all 
#    specialnets in that layer
# 4. (a) Accepts only 1 viarule per metal layer and only adds that via per layer
#    (b) Vias added at every intersection between two layers. Need to allow for
#    	 a 'pitch' setting
# 5. If offsets/widths/pitches result in off-track, enable snapping to grid
# 6. Handle mask information
##############################################################################
###########################Improved in this version ##########################
# 1. Separate inputs file
# 2. Process techfile to get BEOL information
# 3. Can create PG stripes, given a particular area of the chip
##############################################################################
##############################################################################

namespace eval ::pdn {
    variable block
    variable tech
    variable libs
    variable design_data {}
    variable default_grid_data {}
    variable def_output
    variable widths
    variable pitches
    variable loffset
    variable boffset
    variable site
    variable site_width
    variable site_name
    variable row_height
    variable metal_layers {}
    variable metal_layers_dir {}
    
    ## procedure for file existence check, returns 0 if file does not exist or file exists, but empty
    proc -s {filename} {
      return [expr {([file exists $filename]) && ([file size $filename] > 0)}]
    }

    proc get {args} {
        variable design_data
        
        return [dict get $design_data {*}$args]
    }
    proc get_macro_power_pins {inst_name} {
        set specification [select_instance_specification $inst_name]
        if {[dict exists $specification power_pins]} {
            return [dict get $specification power_pins]
        }
        return "VDDPE VDDCE"
    }
    proc get_macro_ground_pins {inst_name} {
        set specification [select_instance_specification $inst_name]
        if {[dict exists $specification ground_pins]} {
            return [dict get $specification ground_pins]
        }
        return "VSSE"
    }
    
    proc transform_box {xmin ymin xmax ymax origin orientation} {
        switch -exact $orientation {
            R0    {set new_box [list $xmin $ymax $ymin $xmax]}
            R90   {set new_box [list [expr -1 * $ymax] $xmin [expr -1 * $ymin] $xmax]}
            R180  {set new_box [list [expr -1 * $xmax] [expr -1 * $ymax] [expr -1 * $xmin] [expr -1 * $ymin]]}
            R270  {set new_box [list $ymin [expr -1 * $xmax] $ymax [expr -1 * $xmin]]}
            MX    {set new_box [list $xmin [expr -1 * $ymax] $xmax [expr -1 * $ymin]]}
            MY    {set new_box [list [expr -1 * $xmax] $ymin [expr -1 * $xmin] $ymax]}
            MXR90 {set new_box [list $ymin $xmin $ymax $xmax]}
            MYR90 {set new_box [list [expr -1 * $ymax] [expr -1 * $xmax] [expr -1 * $ymin] [expr -1 * $xmin]]}
            default {error "Illegal orientation $orientation specified"}
        }
        return [list \
            [expr [lindex $new_box 0] + [lindex $origin 0]] \
            [expr [lindex $new_box 1] + [lindex $origin 1]] \
            [expr [lindex $new_box 2] + [lindex $origin 0]] \
            [expr [lindex $new_box 3] + [lindex $origin 1]] \
        ]
    }
    
    proc get_memory_instance_pg_pins {} {
        variable block
        variable orig_stripe_locs

        foreach inst [$block getInsts] {
            set inst_name [$inst getName]
            set master [$inst getMaster]

            if {[$master getType] == "CORE"} {continue}
            if {[$master getType] == "IO"} {continue}
            if {[$master getType] == "SPACER"} {continue}
            if {[$master getType] == "NONE"} {continue}
            if {[$master getType] == "ENDCAP_PRE"} {continue}
            if {[$master getType] == "ENDCAP"} {continue}
            if {[$master getType] == "CORE_SPACER"} {continue}
            if {[$master getType] == "CORE_TIEHIGH"} {continue}
            if {[$master getType] == "CORE_TIELOW"} {continue}

            foreach term_name [concat [get_macro_power_pins $inst_name] [get_macro_ground_pins $inst_name]] {
                set inst_term [$inst findITerm $term_name]
                if {$inst_term == "NULL"} {continue}
                
                set mterm [$inst_term getMTerm]
                set type [$mterm getSigType]

                foreach mPin [$mterm getMPins] {
                    foreach geom [$mPin getGeometry] {
                        set layer [[$geom getTechLayer] getName]
                        set box [transform_box [$geom xMin] [$geom yMin] [$geom xMax] [$geom yMax] [$inst getOrigin] [$inst getOrient]]

                        set width  [expr abs([lindex $box 2] - [lindex $box 0])]
                        set height [expr abs([lindex $box 3] - [lindex $box 1])]

                        if {$width > $height} {
                            set xl [lindex $box 0]
                            set xu [lindex $box 2]
                            set y  [expr ([lindex $box 1] + [lindex $box 3])/2]
                            set width [expr [lindex $box 3] - [lindex $box 1]]
                            lappend orig_stripe_locs(${layer}_PIN_hor,$type) [list $xl $y $xu $width]
                        } else {
                            set x  [expr ([lindex $box 0] + [lindex $box 2])/2]
                            set yl [lindex $box 1]
                            set yu [lindex $box 3]
                            set width [expr [lindex $box 2] - [lindex $box 0]]
                            lappend orig_stripe_locs(${layer}_PIN_ver,$type) [list $x $yl $yu $width]
                        }
                    }
                }
            }    
        }
#        puts "Total walltime till macro pin geometry creation = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds"
    }

    proc init {opendb_block {PDN_cfg "PDN.cfg"}} {
        variable db
        variable block
        variable tech
        variable libs
        variable design_data
        variable def_output
        variable default_grid_data
        variable stripe_locs
        variable design_name
        variable site
        variable row_height
        variable site_width
        variable site_name
        variable metal_layers
        variable def_units
        
#        set ::start_time [clock clicks -milliseconds]
        if {![-s $PDN_cfg]} {
          puts "File $PDN_cfg does not exist, or exists but empty"
          exit 1
        }

        source $PDN_cfg

        set block $opendb_block
        set def_units [$block getDefUnits]
        set design_name [$block getName]
        set db [$block getDataBase]
        set tech [$db getTech]
        set libs [$db getLibs]

        init_metal_layers
        init_via_tech
        
        set die_area [$block getDieArea]
        puts "Design Name is $design_name"
        set def_output "${design_name}_pdn.def"
        
        if {[info vars ::power_nets] == ""} {
            set ::power_nets "VDD"
        }
        
        if {[info vars ::ground_nets] == ""} {
            set ::ground_nets "VSS"
        }

        dict set design_data power_nets $::power_nets
        dict set design_data ground_nets $::ground_nets

        # Sourcing user inputs file
        #
        set sites {}
        foreach lib $libs {
            set sites [concat $sites [$lib getSites]]
        }
        set site [lindex $sites 0]

        set site_name [$site getName]
        set site_width [$site getWidth] 
        
        set row_height [$site getHeight]

        ##### Get information from BEOL LEF
        puts "Reading BEOL LEF and gathering information ..."

#        puts " DONE \[Total elapsed walltime = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds\]"

        if {[info vars ::layers] != ""} {
            foreach layer $::layers {
                if {[dict exists $::layers $layer widthtable]} {
                    dict set ::layers $layer widthtable [lmap x [dict get $::layers $layer widthtable] {expr $x * $def_units}]
                }
            }
            set_layer_info $::layers
        }

        if {[info vars ::halo] != ""} {
            if {[llength $::halo] == 1} {
                set default_halo "$::halo $::halo $::halo $::halo"
            } elseif {[llength $::halo] == 2} {
                set default_halo "$::halo $::halo"
            } elseif {[llength $::halo] == 4} {
                set default_halo $::halo
	    } else {
                error "ERROR: Illegal number of elements defined for ::halo \"$::halo\""
            }
        } else {
            set default_halo "0 0 0 0"
        }

        dict set design_data config [list \
            def_output   $def_output \
            design       $design_name \
            core_area    [list $::core_area_llx $::core_area_lly $::core_area_urx $::core_area_ury] \
            die_area     [list [$die_area xMin]  [$die_area yMin] [$die_area xMax] [$die_area yMax]] \
            default_halo [lmap x $default_halo {expr $x * $def_units}] \
        ]
                   
        foreach lay $metal_layers { 
	    set stripe_locs($lay,POWER) ""
	    set stripe_locs($lay,GROUND) ""
        }

        ########################################
        # Creating blockages based on macro locations
        #######################################
        pdn read_macro_boundaries

        pdn get_memory_instance_pg_pins

        if {$default_grid_data == {}} {
            set default_grid_data [lindex [dict get $design_data grid stdcell] 0]
        }

        ##### Basic sanity checks to see if inputs are given correctly
        if {[lsearch $metal_layers [get_rails_layer]] < 0} {
	    error "ERROR: Layer specified for std. cell rails not in list of layers."
        }

#        puts "Total walltime till PDN setup = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds"

        return $design_data
    }
    
    proc specify_grid {type specification} {
        variable design_data

        set specification [list $specification]
        if {[dict exists $design_data grid $type]} {
            set specification [concat [dict get $design_data grid $type] $specification]
        }
        dict set design_data grid $type $specification
    }
    
    proc add_grid {grid_data} {
        variable design_data
        variable def_units
        variable widths
        variable pitches
        variable loffset
        variable boffset
        
        ##### Creating maps for directions, widths and pitches
        set area [dict get $grid_data area]

        foreach lay [dict keys [dict get $grid_data layers]] { 
	    set widths($lay)    [expr round([dict get $grid_data layers $lay width] * $def_units)] 
	    set pitches($lay)   [expr round([dict get $grid_data layers $lay pitch] * $def_units)]
	    set loffset($lay)   [expr round([dict get $grid_data layers $lay offset] * $def_units)]
	    set boffset($lay)   [expr round([dict get $grid_data layers $lay offset] * $def_units)]
        }

        ## Power nets
        foreach pwr_net [dict get $design_data power_nets] {
	    set tag "POWER"
	    generate_stripes_vias $tag $pwr_net $grid_data
        }
        ## Ground nets
        foreach gnd_net [dict get $design_data ground_nets] {
	    set tag "GROUND"
	    generate_stripes_vias $tag $gnd_net $grid_data
        }

    }

    proc select_instance_specification {instance} {
        variable design_data
        variable instances

        set macro_specifications [dict get $design_data grid macro]
        
        # If there is a specifcation that matches this instance name, use that
        foreach specification $macro_specifications {
            if {![dict exists $specification instance]} {continue}
            if {[dict get $specification instance] == $instance} {
                return $specification
            }
        }
        
        # If there is a specification that matches this macro name, use that
        if {[dict exists $instances $instance]} {
            set instance_macro [dict get $instances $instance macro]

            # If there are orientation based specifcations for this macro, use the appropriate one if available
            foreach spec $macro_specifications {
                if {!([dict exists $spec macro] && [dict get $spec orient] && [dict get $spec macro] == $instance_macro)} {continue}
                if {[lsearch [dict get $spec orient] [dict get $instances $instance orient]] != -1} {
                    return $spec
                }
            }
        
            # There should only be one macro specific spec that doesnt have an orientation qualifier
            foreach spec $macro_specifications {
                if {!([dict exists $spec macro] && [dict get $spec macro] == $instance_macro)} {continue}
                if {[lsearch [dict get $spec orient] [dict get $instances $instance orient]] != -1} {
                    return $spec
                }
            }

            # If there are orientation based specifcations, use the appropriate one if available
            foreach spec $macro_specifications {
                if {!(![dict exists $spec macro] && ![dict exists $spec instance] && [dict exists $spec orient])} {continue}
                if {[lsearch [dict get $spec orient] [dict get $instances $instance orient]] != -1} {
                    return $spec
                }
            }
        }

        # There should only be one macro specific spec that doesnt have an orientation qualifier
        foreach spec $macro_specifications {
            if {!(![dict exists $spec macro] && ![dict exists $spec instance])} {continue}
            return $spec
        }

        error "Error: no matching grid specification found for $instance"
    }
    
    proc get_instance_specification {instance} {
        variable instances

        set specification [select_instance_specification $instance]

        if {![dict exists $specification blockage]} {
            dict set specification blockage {}
        }
        dict set specification area [dict get $instances $instance macro_boundary]
        
        return $specification
    }
    
    proc init_metal_layers {} {
        variable tech
        variable metal_layers
        variable metal_layers_dir

        set metal_layers {}        
        set metal_layers_dir {}
        
        foreach layer [$tech getLayers] {
            if {[$layer getType] == "ROUTING"} {
                lappend metal_layers [$layer getName]
                lappend metal_layers_dir [$layer getDirection]
            }
        }
    }

    proc get_instance_llx {instance} {
        variable instances
        return [lindex [dict get $instances $instance halo_boundary] 0]
    }
    
    proc get_instance_lly {instance} {
        variable instances
        return [lindex [dict get $instances $instance halo_boundary] 1]
    }
    
    proc get_instance_urx {instance} {
        variable instances
        return [lindex [dict get $instances $instance halo_boundary] 2]
    }
    
    proc get_instance_ury {instance} {
        variable instances
        return [lindex [dict get $instances $instance halo_boundary] 3]
    }
    
    proc get_macro_blockage_layers {instance} {
        variable metal_layers
        
        set specification [select_instance_specification $instance]
        if {[dict exists $specification blockage]} {
            return [dict get $specification blockage]
        }
        return [lrange $metal_layers 0 3]
    }
    
    proc print_strategy {type specification} {
        puts "Type: $type"
        if {[dict exists $specification rails]} {
            puts "    Follow Pins Layer: [dict get $specification rails]"
        }
        if {[dict exists $specification instance]} {
            puts "    Instance: [dict get $specification orient]"
        }
        if {[dict exists $specification macro]} {
            puts "    Macro: [dict get $specification orient]"
        }
        if {[dict exists $specification orient]} {
            puts "    Macro orientation: [dict get $specification orient]"
        }
        dict for {layer_name layer} [dict get $specification layers] {
            puts [format "    Layer: %s, Width: %f Pitch: %f Offset: %f" $layer_name [dict get $layer width]  [dict get $layer pitch] [dict get $layer offset]]
        }
        puts "    Connect: [dict get $specification connect]"
    }
    
    proc plan_grid {} {
        variable design_data
        variable instances
        variable default_grid_data
        variable def_units

        ################################## Main Code #################################

        puts "****** INFO ******"
        foreach specification [dict get $design_data grid stdcell] {
            print_strategy stdcell $specification
        }
        foreach specification [dict get $design_data grid macro] {
            print_strategy macro $specification
        }
        puts "**** END INFO ****"

        foreach specification [dict get $design_data grid stdcell] {
            dict set specification blockage [dict keys $instances]
            if {![dict exists $specification area]} {
                dict set specification area [lmap x [dict get $design_data config core_area] {expr round($x * $def_units)}]
            }
            pdn add_grid $specification
            if {$default_grid_data == {}} {
                set default_grid_data $specification
            }

        }
        
        foreach instance [dict keys $instances] {
            pdn add_grid [get_instance_specification $instance]
        }
    }
    
    proc opendb_update_grid {} {
        write_opendb_vias
        write_opendb_specialnets
        write_opendb_rows
    }
        
    proc apply {block config} {
        pdn init $block $config

        puts "##Power Delivery Network Generator: Generating PDN DEF"
#         set ::start_time [clock clicks -milliseconds]

        pdn plan_grid
        opendb_update_grid

#        puts "Total walltime to generate PDN DEF = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds"
    }

    namespace export init apply get_memory_instance_pg_pins 
    namespace export specify_grid plan_grid add_grid get
    namespace ensemble create
}

package provide pdn 0.3.0
