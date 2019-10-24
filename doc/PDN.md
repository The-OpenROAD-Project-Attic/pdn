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
| ```::rails_start_with``` | POWER\|GROUND |
| ```::stripes_start_with``` | POWER\|GROUND |

#### Technology configuration

| Variable Name | Description |
|:---|:---|
| ```::lef_files``` | File containing technology and library cell data |
| ```::site_name``` | Name of the stdcell site  |
| ```::site_width``` | Width of the stdcell site |
| ```::row_height``` | Height of a stdcell row |
| ```::met_layer_list``` | List of metal layer names - bottom to top |
| ```::met_layer_dir``` | List of preferred direction for the layer names - bottom to top |
| ```::via_tech``` | List of VIARULES present in the technology LEF. Each viarule has 3 elements, lower, upper and cut. Entries for lower and upper define the layer name and enclosure settings. The cut entry specifies layer_name, size and spacing |

## Optional Global Variables

| Variable Name | Description |
|---|:---|
| ```::halo``` | Halo to apply around macros. Specify one, two or four values. If a HALO is defined in the floorplan DEF, then this will be ignored. |


## Power grid strategy definitions
A set of power grid specifications are provided by calling the ```pdn specify_grid``` command. 
At least one power grid specification must be defined.

The command has the following form

```TCL
pdn specify_grid (stdcell|macro) <specification>
```

Where specification is a list of key value pairs

| Key | Description |
|:---|:---|
| ```layers``` | List of layers on which to draw stripes. Each layer will specify a value for width, pitch and offset |
| ```rails``` | The layer which should be used to draw the horizontal stdcell rails |
| ```connect``` | List of connections to be made between layers. Macro pin connections are on layer \<layer_name>_PIN_\<direction> |

Additionally, for macro grids
| Key | Description |
|:---|:---|
| ```orient``` | If the orientation of the macro matches an entry in this list, then apply this grid specification
| ```instance``` | If the instance name of the macro matches an entry in this list, then apply this grid specification  |
| ```macro``` | If the macro name of the macro matches an entry in this list, then apply this grid specification |
| ```power_pins``` | List of power pins on the macro to connect to the grid |
| ```ground_pins``` | List of ground pins on the macro to connect to the grid |
| ```blockages``` | Layers which are blocked by a macro using this grid specification |

The key connect specifies two layers to be connected together where the stripes the same net of the first layer overlaps with stripes of the second layer

Macro pins are extracted from the LEF/DEF and are specified on a layer called ```<layer_name>_PIN_<dir>```, where ```<layer_name>``` is the name of the layer as specified in ::met_layer_list, and ```<dir>``` is ```hor``` to indicate horizontal pins in the floorplan and is ```ver``` to indicate that the pins are oriented vertically in the floorplan.

A separate grid is built for each macro. The list of macro specifications that have been defined are searched to find a specification with a matcing instance name key, failing that a macro specification with a matching macro name key, or else the first specification with neither an instance or macro key is used. Furthermore, if orient is specified, then the orientation of the macro must match one of the entries in the orient field

### Examples of grid specifications

1. Stdcell grid specification
```TCL
pdn specify_grid stdcell {
    rails metal1
    layers {
        metal1 {width 0.17 pitch  2.4 offset 0} 
        metal4 {width 0.48 pitch 56.0 offset 2}
        metal7 {width 1.40 pitch 40.0 offset 2}
    }
    connect {{metal1 metal4} {metal4 metal7}}
}
```
This specification adds a grid over the stdcell area, with an metal1 followpin width of 0.17,connecting to metal4 stripes of 0.48um every 56.0um, connecting in turn to metal7 stripes, also 1.40um wide and 40.0 pitch

2. Macro grid specification
```TCL
pdn specify_grid macro {
    orient {N FN S FS}
    power_pins "VDDPE VDDCE"
    ground_pins "VSSE"
    blockages "metal1 metal2 metal3 metal4"
    layers {
        metal5 {width 0.93 pitch 40.0 offset 2}
        metal6 {width 0.93 pitch 40.0 offset 2}
    } 
    connect {{metal4_PIN_ver metal5} {metal5 metal6} {metal6 metal7}}
}
```

If this the only macro grid specification defined, then it will be applied over all the macros in the design that match one of the entries in the orient field.

All vertical metal4 pins on the macros are connected to metal5, which is then connected to metal6, which is connected in turn to metal7.

For macros that have their pins oriented in non-preferred routing direction the grid specification would be as follows.

```TCL
pdn specify_grid macro {
    orient {E FE W FW}
    power_pins "VDDPE VDDCE"
    ground_pins "VSSE"
    blockages "metal1 metal2 metal3 metal4"
    layers {
        metal6 {width 0.93 pitch 40.0 offset 2}
    }
    connect {{metal4_PIN_hor metal6} {metal6 metal7}}
}
```

Macros with orientations E, FE, W or FW will have their metal4 pins in the vertical (non-preferred) direction - this specification connects these pins directly to the metal6 layer, then on to metal7.

In both of thees cases we have specified that the macrcos have power pins VDDPE and VDDCE, ground pins VSSE and create blockages in layers metal1 to metal4

