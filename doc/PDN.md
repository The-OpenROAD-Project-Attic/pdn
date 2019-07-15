# Elements of a configuration file

A configuration file for apply_pdn consists of the following parts
1. Global variable definitions
1. Call a TCL procedure to create grid specifications
1. Define a TCL procedure to write out the required set of VIAs

## Required Global Variables
Global variables are prefixed by ```::``` and force the variable to be declared in the global scope, rather than the local scope. This make the values of these variables available everywhere in the TCL code.
The ```apply_pdn``` command requires the following global variables to exist

| Variable Name | Description |
|:---|:---|
|  ```::design``` |  Name of the design |
|  ```::def_units``` | Number of database unit that make 1 micron |
| ```::FpOutDef``` | Name of the floorplan DEF containing macro and pin placement |
| ```::flatLef``` | File containing technology and library cell data |
| ```::macro_power_pins``` | List of names of power pins on macros |
| ```::macro_ground_pins``` | List of names of ground pins on macros |
| ```::site_name``` | Name of the stdcell site  |
| ```::site_width``` | Width of the stdcell site |
| ```::row_height``` | Height of a stdcell row |
| ```::core_area_llx``` | Core area llx |
| ```::core_area_lly``` | Core area lly |
| ```::core_area_urx``` | Core area urx |
| ```::core_area_ury``` | Core area ury |
| ```::die_area_llx``` | Die area llx |
| ```::die_area_llx``` | Die area lly |
| ```::die_area_llx``` | Die area urx |
| ```::die_area_llx``` | Die area ury |
| ```::power_nets``` | Name of the power net |
| ```::ground_nets``` | Name of the ground net |
| ```::macro_blockage_layer_list``` | List of metal layer names blocked by macros |
| ```::met_layer_list``` | List of metal layer names - bottom to top |
| ```::met_layer_dir``` | List of preferred direction for the layer names - bottom to top |
| ```::rails_mlayer``` | Layer used as stdcell rail |
| ```::rails_start_with``` | POWER\|GROUND |
| ```::stripes_start_with``` | POWER\|GROUND |

## Optional Global Variables

| Variable Name | Description |
|---|:---|
| ```::halo``` | Halo to apply around macros. Specify one, two or four values. If a HALO is defined in the floorplan DEF, then this will be ignored. |


## Call TCL procedure
A set of power grid specifications are provided by calling the ```pdn specify_grid``` command. 
At least one power grid specification must be defined.

The command has the following form

```TCL
pdn specify_grid (stdcell|macro) <specification>
```

Where specification is a list of key value pairs

| Key | Description |
|:---|:---|
| ```layers``` | List of layers on which to draw stripes |
| ```dir``` | List of directions of layers |
| ```widths``` | List of width of stripes  |
| ```pitches``` | List of pitches of stripes |
| ```loffset``` | List of left hand offsets of stripes |
| ```boffset``` | List of bottom offsets of stripes |
| ```connect``` | List of connections to be made between layers |
| ```vias``` | List of vias to use to connect layers - match ::met_layer_list |
| ```instance``` |  |
| ```macro``` |  |

The lists for dir, widths, pitches, loffset, boffset must all be the same length as the list for layers
such that the nth element in the list gives the values for the layer whose name is also nth in the
list of layers

The key connect specifies two layers to be connected together where the stripes the same net of the first layer 
overlaps with stripes of the second layer

Macro pins are extracted from the LEF/DEF and are specified on a layer called ```\<layer_name>_PIN_\<dir>```, where ```\<layer_name>```
is the name of the layer as specified in ::met_layer_list, and '''\<dir>``` is hor to indicate horizontal pins in the floorplan and is 
```ver``` to indicate that the pins are oriented vertically in the floorplan.

The key vias contains a list of vias to be used to connect to the next higher layer. The first element of the 
list is for M1 to M2 VIAs, the second element for M2 to M3 vias etc. Hence, this list should be 1 element shorter
than the list ::met_layer_list

A separate grid is built for each macro. The list of macro specification sthat have been defined are searched to 
find a specification with a matcing instance name key, failing that a macro specification with a matching macro name key, 
or else the first specification with neither an instance or macro key is used.

### Examples of grid specifications

1. Stdcell grid specification
```TCL
pdn specify_grid stdcell {
    layers    "M1 M4 M7" 
    dir       "hor ver hor" 
    widths    "0.64 0.93 0.93" 
    pitches   "2.40 40.0 40.0" 
    loffset   "0 2 2" 
    boffset   "0 2 2" 
    connect   "{M1 M4} {M4 M7}" 
    vias      "VIA1_RULE_1 VIA2_RULE_1 VIA3_RULE_1 VIA4_RULE_1 VIA5_RULE_264 VIA6_RULE_18 VIA7_RULE_18 VIA8_RULE_1"
}
```
This specification adds a grid over the stdcell area, with an M1 horizontal followpin width of 0.64,connecting to 
M4 vertical stripes of 0.93 every 40.0, connecting in turn to horizontal M7 stripes, also 0.93 wide and 40.0 pitch

2. Macro grid specification
```TCL
pdn specify_grid macro {
    layers    "M6" 
    dir       "ver"
    widths    "0.93"
    pitches   "40" 
    loffset   2 
    boffset   0 
    connect   "{M4_PIN_hor M6} {M6 M7}"
    vias      "VIA1_RULE_1 VIA2_RULE_1 VIA3_RULE_1 VIA4_RULE_293 VIA5_RULE_360 VIA6_RULE_21 VIA7_RULE_18 VIA8_RULE_1"
}
```

If this the only macro grid specification defined, then it will be applied over all the macros in the design.

All horizontal M4 pins on the macros are connected to vertical M6, which is then connected to horizontal M7. In this way macros that
have their pins oriented in non-preferred routing directions can still be connected up to the power grid.

## Define TCL Procedure
A function call generate_vias must be created that writes out a definition of VIAs that are referenced by the grid specfications, but do not already exist in the technology LEF.

It is unlikely that the VIAs needed to connect the layers of the power grid will already exist in the technology LEF. The technology LEF provides a mechanism to create VIAs using VIARULES. VIAs referenced in the grid specifications that do not already exist in the technology file are must be created using available VIARULEs. 

The function calls the ```pdn def_out``` function to write DEF for the required VIAs.

