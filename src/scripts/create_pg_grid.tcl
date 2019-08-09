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
    
    ## procedure for file existence check, returns 0 if file does not exist or file exists, but empty
    proc -s {filename} {
      return [expr {([file exists $filename]) && ([file size $filename] > 0)}]
    }

    proc get {args} {
        variable design_data
        
        return [dict get $design_data {*}$args]
    }
    
    proc get_memory_instance_pg_pins {} {
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

        if {[info vars ::macro_power_pins] == ""} {
             set ::macro_power_pins "VDDPE VDDCE"
        }
        if {[info vars ::macro_ground_pins] == ""} {
            set ::macro_ground_pins "VSSE"
        }
        
        while {![eof $ch]} {
            set line [gets $ch]
            set line [regsub -all {[\(\)\-\,]} $line {}]

            if {[regexp {^instName.*/([^/]*):} $line - pin_name]} {
                if {[lsearch $::macro_power_pins $pin_name] != -1} {
                    set net vdd
                } elseif {[lsearch $::macro_ground_pins $pin_name] != -1} {
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
            lappend ::orig_stripe_locs([lindex $pin 4]_PIN_hor,POWER) [list $xl $y $xu $width]
        }

        foreach pin $mem_pins_vdd_ver {
            set x  [expr ([lindex $pin 0] + [lindex $pin 2])/2]
            set yl [lindex $pin 1]
            set yu [lindex $pin 3]
            set width [expr [lindex $pin 2] - [lindex $pin 0]]
            lappend ::orig_stripe_locs([lindex $pin 4]_PIN_ver,POWER) [list $x $yl $yu $width]
        }

        foreach pin $mem_pins_vss_hor {
            set xl [lindex $pin 0]
            set xu [lindex $pin 2]
            set y  [expr ([lindex $pin 1] + [lindex $pin 3])/2]
            set width [expr [lindex $pin 3] - [lindex $pin 1]]
            lappend ::orig_stripe_locs([lindex $pin 4]_PIN_hor,GROUND) [list $xl $y $xu $width]
        }

        foreach pin $mem_pins_vss_ver {
            set x  [expr ([lindex $pin 0] + [lindex $pin 2])/2]
            set yl [lindex $pin 1]
            set yu [lindex $pin 3]
            set width [expr [lindex $pin 2] - [lindex $pin 0]]
            lappend ::orig_stripe_locs([lindex $pin 4]_PIN_ver,GROUND) [list $x $yl $yu $width]
        }

        puts "Total walltime till macro pin geometry creation = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds"
    }

    proc init {{PDN_cfg "PDN.cfg"}} {
        variable design_data
        variable def_output
        variable vias
        
        set ::start_time [clock clicks -milliseconds]

        if {![-s $PDN_cfg]} {
          puts "File $PDN_cfg does not exist, or exists but empty"
          exit 1
        }

        source $PDN_cfg

        puts "Design Name is $::design"
        set def_output "${::design}_pdn.def"
        
        # Sourcing user inputs file
        #
        set ::row_height [expr {$::def_units * $::row_height}]

        ##### Get information from BEOL LEF
        puts "Reading BEOL LEF and gathering information ..."

        puts " DONE \[Total elapsed walltime = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds\]"

        ##puts -nonewline "Doing sanity checks, gathering information and creating DEF headers ..."
        ##### Basic sanity checks to see if inputs are given correctly
        if {[lsearch $::met_layer_list $::rails_mlayer] < 0} {
	        puts "ERROR: Layer specified for std. cell rails not in list of layers. EXITING....."
	        exit
        }

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
                   
        foreach lay $::met_layer_list { 
	    set ::stripe_locs($lay,POWER) ""
	    set ::stripe_locs($lay,GROUND) ""
	    set ::stripe_locs_blk($lay,POWER) ""
	    set ::stripe_locs_blk($lay,GROUND) ""	
        }

        ########################################
        # Creating blockages based on macro locations
        #######################################
        pdn read_macro_boundaries $::FpOutDef $::lef_files

        pdn get_memory_instance_pg_pins

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

        ##### Creating maps for directions, widths and pitches
        set def_units [dict get $design_data config def_units]
        set area [dict get $grid_data area]

        set idx 0
        foreach lay [dict get $grid_data layers] { 
	    set ::widths($lay)    [expr round([lindex [dict get $grid_data widths]  $idx] * $def_units)] 
	    set ::pitches($lay)   [expr round([lindex [dict get $grid_data pitches] $idx] * $def_units)]
	    set ::loffset($lay)   [expr round([lindex [dict get $grid_data loffset] $idx] * $def_units)]
	    set ::boffset($lay)   [expr round([lindex [dict get $grid_data boffset] $idx] * $def_units)]
	    incr idx
        }

        ## Power nets
        set ::row_index 1
        foreach pwr_net $::power_nets {
	    set tag "POWER"
	    generate_stripes_vias $tag $pwr_net $grid_data
        }
        ## Ground nets
        foreach gnd_net $::ground_nets {
	    set tag "GROUND"
	    generate_stripes_vias $tag $gnd_net $grid_data
        }

    }

    proc get_instance_specification {instance} {
        variable design_data
        variable instances

        foreach specification [dict get $design_data grid macro] {
            if {![dict exists $specification blockage]} {
                dict set specification blockage {}
            }
            if {[dict exists $specification instance]} {
                if {[dict get $specification instance] == $instance} {
                    dict set specification area [dict get $instances $instance macro_boundary]
                    return $specification
                }
            }
        }
        
        foreach specification [dict get $design_data grid macro] {
            if {![dict exists $specification blockage]} {
                dict set specification blockage {}
            }
            if {[dict exists $specification macro]} {
                if {[dict get $instances $instance macro] == [dict get $specification macro]} {
                    dict set specification area [dict get $instances $instance macro_boundary]
                    return $specification
                }
            }
        }

        foreach specification [dict get $design_data grid macro] {
            if {![dict exists $specification blockage]} {
                dict set specification blockage {}
            }
            if {![dict exists $specification instance] && ![dict exists $specification macro]} {
                dict set specification area [dict get $instances $instance macro_boundary]
                return $specification
            }
        }
        puts "Error: no matching grid specification found for $instance"
        exit -1
    }
    
    proc power_grid {} {
        variable design_data
        variable instances
        variable default_grid_data

        ################################## Main Code #################################

        set def_units [dict get $design_data config def_units]
        puts "****** INFO ******"
        puts "**** END INFO ****"

        foreach specification [dict get $design_data grid stdcell] {
            dict set specification blockage [get_macro_halo_boundaries]
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

package provide pdn 0.1.0
