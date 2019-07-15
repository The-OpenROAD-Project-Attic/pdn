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

        set OP1 [open run.param w]

        set cmd "exec touch dummy.guide"
        catch {eval $cmd}

        if {[-s $::FpOutDef]} {
          puts "MACRO Packed DEF File $::FpOutDef  exists and not empty.."
        } else {
          puts "File $::FpOutDef does not exist, or exists but empty"
          exit
        }


        if {[-s ${::flatLef}]} {
          puts "Flat LEF File ${::flatLef}  exists and not empty.."
        } else {
          puts "File ${::flatLef} does not exist, or exists but empty. Please check PDN.cfg"
          exit
        }
        pdn write_macrocell_list "macrocell.list"

        puts $OP1 "lef:${::flatLef}"
        puts $OP1 "def:${::FpOutDef}"
        puts $OP1 "guide:dummy.guide"
        puts $OP1 "output:dummy.def"
        puts $OP1 "macroList:macrocell.list"
        puts $OP1 "threads:16"
        puts $OP1 "cpxthreads:8"
        puts $OP1 "verbose:2"
        puts $OP1 "gap:0"
        puts $OP1 "timeout:2400"
        close $OP1

        puts "Total walltime till macro pin geometry creation = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds"

        puts "##Power Delivery Network Generator: Generating inputs for PDN Gen"


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
            lappend ::orig_stripe_locs([lindex $pin 4]_PIN_hor,POWER) [list $xl $y $xu]
        }

        foreach pin $mem_pins_vdd_ver {
            set x  [expr ([lindex $pin 0] + [lindex $pin 2])/2]
            set yl [lindex $pin 1]
            set yu [lindex $pin 3]
            lappend ::orig_stripe_locs([lindex $pin 4]_PIN_ver,POWER) [list $x $yl $yu]
        }

        foreach pin $mem_pins_vss_hor {
            set xl [lindex $pin 0]
            set xu [lindex $pin 2]
            set y  [expr ([lindex $pin 1] + [lindex $pin 3])/2]
            lappend ::orig_stripe_locs([lindex $pin 4]_PIN_hor,GROUND) [list $xl $y $xu]
        }

        foreach pin $mem_pins_vss_ver {
            set x  [expr ([lindex $pin 0] + [lindex $pin 2])/2]
            set yl [lindex $pin 1]
            set yu [lindex $pin 3]
            lappend ::orig_stripe_locs([lindex $pin 4]_PIN_ver,GROUND) [list $x $yl $yu]
        }
    }

    proc init {PDN_cfg} {
        variable design_data
        variable def_output

        if {[-s $PDN_cfg]} {
          puts "File $PDN_cfg exists and not empty.."
        } else {
          puts "File $PDN_cfg does not exist, or exists but empty"
          exit
        }

        source $PDN_cfg

        puts "Design Name is $::design"
        set def_output "${::design}_pdn.def"
        
        # Sourcing user inputs file
        #
        set ::row_height [expr {$::def_units * $::row_height}]

        ##### Get information from BEOL LEF
        puts -nonewline "Reading BEOL LEF and gathering information ..."
        ##get_info_from_techlef
        puts " DONE \[Total elapsed walltime = [expr {[expr {[clock clicks -milliseconds] - $::start_time}]/1000.0}] seconds\]"

        ##puts -nonewline "Doing sanity checks, gathering information and creating DEF headers ..."
        ##### Basic sanity checks to see if inputs are given correctly
        if {[lsearch $::met_layer_list $::rails_mlayer] < 0} {
	        puts "ERROR: Layer specified for std. cell rails not in list of layers. EXITING....."
	        exit
        }

        set ::vias {}
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
        pdn read_macro_boundaries $::FpOutDef $::cellLef

        pdn get_memory_instance_pg_pins

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
	        set ::dir($lay) [lindex [dict get $grid_data dir] $idx]
	        set ::widths($lay) [expr [lindex [dict get $grid_data widths] $idx] * $def_units] 
	        set ::pitches($lay) [expr [lindex [dict get $grid_data pitches] $idx] * $def_units]
	        set ::liooffset($lay) [expr ([lindex [dict get $grid_data loffset] $idx] * $def_units)]
	        set ::biooffset($lay) [expr ([lindex [dict get $grid_data boffset] $idx] * $def_units)]
	        set ::loffset($lay) [expr ([lindex [dict get $grid_data loffset] $idx]) * $def_units]
	        set ::boffset($lay) [expr ([lindex [dict get $grid_data boffset] $idx]) * $def_units]
	        incr idx
	        ##puts "OFFSET: layer is $lay, $::biooffset($lay), $::liooffset($lay)"
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

    proc power_grid {} {
        variable design_data
        variable instances
        
        ################################## Main Code #################################

        set def_units [dict get $design_data config def_units]
        puts "****** INFO ******"
        puts "**** END INFO ****"

        foreach specification [dict get $design_data grid stdcell] {
            dict set specification blockage [get_macro_halo_boundaries]
            if {![dict exists $specification area]} {
                dict set specification area [lmap x [list $::core_area_llx $::core_area_lly $::core_area_urx $::core_area_ury] {expr $x * $::def_units}]
            }
            pdn add_grid $specification
        }
        
        foreach instance [dict keys $instances] {
            foreach specification [dict get $design_data grid macro] {
                if {![dict exists $specification blockage]} {
                    dict set specification blockage {}
                }
                if {[dict exists $specification instance]} {
                    if {$instance == [dict get $specification instance]} {
                        dict set specification area [lmap x [dict get $instances $instance macro_boundary] {expr $x * $::def_units}]
                        pdn add_grid $specification
                        break
                    }
                } elseif {[dict exists $specification macro]} {
                    if {[dict get $instances $instance macro] == [dict get $specification macro]} {
                        dict set specification area [lmap x [dict get $instances $instance macro_boundary] {expr $x * $::def_units}]
                        pdn add_grid $specification
                        break
                    }
                } else {
                    dict set specification area [lmap x [dict get $instances $instance macro_boundary] {expr $x * $::def_units}]
                    pdn add_grid $specification
                    break
                }
            }
        }
    }
    
    namespace export init get_memory_instance_pg_pins 
    namespace export specify_grid power_grid add_grid get
    namespace ensemble create
}

package provide pdn 0.1.0
