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
    variable design_data {}
    variable default_grid_data {}
    variable def_output
    variable widths
    variable pitches
    variable loffset
    variable boffset
    
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
    
    proc get_memory_instance_pg_pins {} {
        variable orig_stripe_locs
        ########################################
        # Creating run.param file for PdnPinDumper
        #
        #
        ########################################

        puts "##Power Delivery Network Generator: Generating inputs for PDN Gen"

        if {![-s $::FpOutDef]} {
          puts "File $::FpOutDef does not exist, or exists but empty"
          exit 1
        }

        set lef_file_errors 0
        foreach lef_file $::lef_files {
            if {![-s $lef_file]} {
              puts "File $lef_file does not exist, or exists but empty."
              inc lef_file_errors
            }
        }
        if {${lef_file_errors} > 0} {
            puts "Please check PDN.cfg"
            exit $lef_file_errors
        }
        
        set cmd "exec touch dummy.guide"
        catch {eval $cmd}

        set OP1 [open run.param w]
        foreach lef_file $::lef_files {
            puts $OP1 "lef:$lef_file"
        }
        puts $OP1 "def:$::FpOutDef"
        puts $OP1 "guide:dummy.guide"
        puts $OP1 "output:dummy.def"
        puts $OP1 "macroList:macrocell.list"
        puts $OP1 "threads:16"
        puts $OP1 "cpxthreads:8"
        puts $OP1 "verbose:2"
        puts $OP1 "gap:0"
        puts $OP1 "timeout:2400"
        close $OP1

        pdn write_macrocell_list "macrocell.list"

        #set cmd "exec ${wd}/scripts/pdn_input_gen $design"
        #eval $cmd
        #exec "PdnPinDumper run.param > pin.loc"
        #set ch [open "pin.loc"]
        set ch [open "|PdnPinDumper run.param | tee pin_dumper.log"]

        set mem_pins_vdd_hor {}
        set mem_pins_vdd_ver {}
        set mem_pins_vss_hor {}
        set mem_pins_vss_ver {}

        while {![eof $ch]} {
            set line [gets $ch]
            set line [regsub -all {[\(\)\-\,]} $line {}]

            if {[regexp {^instName:\s+(.*);\s+(.*)/([^/]*):} $line - inst_name macro_name pin_name]} {
                if {[lsearch [get_macro_power_pins $inst_name] $pin_name] != -1} {
                    set net vdd
                } elseif {[lsearch [get_macro_ground_pins $inst_name] $pin_name] != -1} {
                    set net vss
                } else {
                    if {[info vars net] != ""} {
                        unset net
                    }
                }
            }

            if {[info vars net] != "" && [llength $line] == 5} {
                set width  [expr abs([lindex $line 2] - [lindex $line 0])]
                set height [expr abs([lindex $line 3] - [lindex $line 1])]

                if {$width > $height} {
                   lappend mem_pins_${net}_hor $line
                } else {
                   lappend mem_pins_${net}_ver $line
                }
            }    
        }

        if {[catch {close $ch} msg]} {
            puts "ERROR: PdnPinDumper failed - check pin_dumper.log"
            puts $msg
            exit
        }

        foreach pin $mem_pins_vdd_hor {
            set xl [lindex $pin 0]
            set xu [lindex $pin 2]
            set y  [expr ([lindex $pin 1] + [lindex $pin 3])/2]
            set width [expr [lindex $pin 3] - [lindex $pin 1]]
            lappend orig_stripe_locs([lindex $pin 4]_PIN_hor,POWER) [list $xl $y $xu $width]
        }

        foreach pin $mem_pins_vdd_ver {
            set x  [expr ([lindex $pin 0] + [lindex $pin 2])/2]
            set yl [lindex $pin 1]
            set yu [lindex $pin 3]
            set width [expr [lindex $pin 2] - [lindex $pin 0]]
            lappend orig_stripe_locs([lindex $pin 4]_PIN_ver,POWER) [list $x $yl $yu $width]
        }

        foreach pin $mem_pins_vss_hor {
            set xl [lindex $pin 0]
            set xu [lindex $pin 2]
            set y  [expr ([lindex $pin 1] + [lindex $pin 3])/2]
            set width [expr [lindex $pin 3] - [lindex $pin 1]]
            lappend orig_stripe_locs([lindex $pin 4]_PIN_hor,GROUND) [list $xl $y $xu $width]
        }

        foreach pin $mem_pins_vss_ver {
            set x  [expr ([lindex $pin 0] + [lindex $pin 2])/2]
            set yl [lindex $pin 1]
            set yu [lindex $pin 3]
            set width [expr [lindex $pin 2] - [lindex $pin 0]]
            lappend orig_stripe_locs([lindex $pin 4]_PIN_ver,GROUND) [list $x $yl $yu $width]
        }

        puts "Total walltime till macro pin geometry creation = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds"
    }

    proc init {{PDN_cfg "PDN.cfg"}} {
        variable design_data
        variable def_output
        variable vias
        variable default_grid_data
        variable stripe_locs
        
        set ::start_time [clock clicks -milliseconds]

        if {![-s $PDN_cfg]} {
          puts "File $PDN_cfg does not exist, or exists but empty"
          exit 1
        }

        source $PDN_cfg

        puts "Design Name is $::design"
        set def_output "${::design}_pdn.def"
        
        if {[info vars ::power_nets] == ""} {
            set ::power_nets "VDD"
        }
        if {[info vars ::ground_nets] == ""} {
            set ::ground_nets "VDD"
        }

        dict set design_data power_nets $::power_nets
        dict set design_data ground_nets $::ground_nets

        # Sourcing user inputs file
        #
        set ::row_height [expr {$::def_units * $::row_height}]

        ##### Get information from BEOL LEF
        puts "Reading BEOL LEF and gathering information ..."

        puts " DONE \[Total elapsed walltime = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds\]"

        set vias {}
        if {[info vars ::halo] != ""} {
            if {[llength $::halo] == 1} {
                set default_halo "$::halo $::halo $::halo $::halo"
            } elseif {[llength $::halo] == 2} {
                set default_halo "$::halo $::halo"
            } elseif {[llength $::halo] == 4} {
                set default_halo $::halo
	    } else {
                puts "ERROR: Illegal number of elements defined for ::halo \"$::halo\""
                exit
            }
        } else {
            set default_halo "0 0 0 0"
        }

        dict set design_data config [list \
            def_output   $def_output \
            design       $::design \
            def_units    $::def_units \
            core_area    [list $::core_area_llx $::core_area_lly $::core_area_urx $::core_area_ury] \
            die_area     [list $::die_area_llx  $::die_area_lly  $::die_area_urx  $::die_area_ury] \
            default_halo [lmap x $default_halo {expr $x * $::def_units}] \
        ]
                   
        foreach lay [get_metal_layers] { 
	    set stripe_locs($lay,POWER) ""
	    set stripe_locs($lay,GROUND) ""
        }

        ########################################
        # Creating blockages based on macro locations
        #######################################
        pdn read_macro_boundaries $::FpOutDef $::lef_files

        pdn get_memory_instance_pg_pins

        if {$default_grid_data == {}} {
            set default_grid_data [lindex [dict get $design_data grid stdcell] 0]
        }

        ##### Basic sanity checks to see if inputs are given correctly
        if {[lsearch [get_metal_layers] [get_rails_layer]] < 0} {
	        puts "ERROR: Layer specified for std. cell rails not in list of layers. EXITING....."
	        exit
        }

        puts "Total walltime till PDN setup = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds"

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
        variable widths
        variable pitches
        variable loffset
        variable boffset
        
        ##### Creating maps for directions, widths and pitches
        set def_units [dict get $design_data config def_units]
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
        foreach specification [lmap spec $macro_specifications {expr {[dict exists $spec instance] ? $spec : [break]}}] {
            if {[dict get $specification instance] == $instance} {
                return $specification
            }
        }
        
        # If there is a specification that matches this macro name, use that
        set instance_macro [dict get $instances $instance macro]

        set macro_specs [lmap spec $macro_specifications {expr {[dict exists $spec macro] && [dict get $spec macro] == $instance_macro ? $spec : [break]}}]
        set oriented_macro_specs [lmap spec $macro_specs  {expr {[dict exists $spec orient] ? $spec : [break]}}]

        # If there are orientation based specifcations for this macro, use the appropriate one if available
        foreach specification $oriented_macro_specs {
            if {[lsearch [dict get $specification orient] [dict get $instances $instance orient]] != -1} {
                return $specification
            }
        }
        
        # There should only be one macro specific spec that doesnt have an orientation qualifier
        set nonoriented_macro_specs [lmap spec $macro_specs  {expr {![dict exists $spec orient] ? $spec : [break]}}]
        if {[llength $nonoriented_macro_specs] > 0} {
            return [lindex $nonoriented_macro_specs 0]
        }

        # Other wise, use a strategy that specifies neither instance nor macro
        set generic_specs [lmap spec $macro_specifications {expr {(![dict exists $spec macro] && ![dict exists $spec instance]) ? $spec : [break]}}]
        set oriented_generic_specs [lmap spec $generic_specs  {expr {[dict exists $spec orient] ? $spec : [break]}}]

        # If there are orientation based specifcations, use the appropriate one if available
        foreach specification $oriented_generic_specs {
            if {[lsearch [dict get $specification orient] [dict get $instances $instance orient]] != -1} {
                return $specification
            }
        }

        # There should only be one macro specific spec that doesnt have an orientation qualifier
        set nonoriented_generic_specs [lmap spec $oriented_generic_specs  {expr {![dict exists $spec orient] ? $spec : [break]}}]
        if {[llength $nonoriented_generic_specs] > 0} {
            return [lindex $nonoriented_generic_specs 0]
        }

        puts "Error: no matching grid specification found for $instance"
        exit -1
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
    
    proc get_metal_layers {} {
        return $::met_layer_list
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
        set specification [select_instance_specification $instance]
        if {[dict exists $specification blockage]} {
            return [dict get $specification blockage]
        }
        return [lrange [get_metal_layers] 0 3]
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
            puts "    Layer: $layer_name"
            puts "        Width:  [dict get $layer width]"
            puts "        Pitch:  [dict get $layer pitch]"
            puts "        Offset: [dict get $layer offset]"
        }
        puts "    Connect: [dict get $specification connect]"
    }
    
    proc power_grid {} {
        variable design_data
        variable instances
        variable default_grid_data

        ################################## Main Code #################################

        set def_units [dict get $design_data config def_units]
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
                dict set specification area [lmap x [dict get $design_data config core_area] {expr round($x * $::def_units)}]
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
    
    namespace export init get_memory_instance_pg_pins 
    namespace export specify_grid power_grid add_grid get
    namespace ensemble create
}

package provide pdn 0.2.0
