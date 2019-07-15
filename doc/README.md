# OpenROAD-pdn

This utility aims to simplify the process of adding a power grid into a floorplan. The aim is to specify a small set of power grid policies to be applied to the design, such as layers to use, stripe width and spacing, then have the utility generate the actual metal straps. Grid policies can be defined over the stdcell area, and over areas occupied by macros.

## Installation and Setup

This package contains the following subdirectories
* src/ containing source code and executables to run power grid insertion
* doc/ detailed documentation and examples

To use the src/scripts directory must be added to the PATH environment variable, and to the TCLLIBPATH environment variable (a space delimited list of directories)

```
% setenv PATH "${PATH}:<pdnDir>/src/scripts"
% setenv TCLLIBPATH "$TCLLIBPATH <pdnDir>/src/scripts"
```

where \<pdnDir> is the path to where the installation can be found.
 
## Usage

```
% apply_pdn <configuration_file>
```

All inputs and power grid policies are specified in a TCL format \<configuration file> 

e.g.

```
% apply_pdn PDN.cfg
```

## Config File

For further information on the config file, and to review an example config see the following:

* [PDN.md](PDN.md)
* [example_PDN.cfg](example_PDN.cfg)

## Assumptions and Limitations

Currently the following assumptions are made:

1. The design is rectangular
1. The input floorplan def file defines the size of the design, includes the placement of all macro blocks and IO pins
1. The user must provide a function that will write out the VIAs that are required to connect the layers together. In order to get a DRC clean power grid, it is important that these VIA definitions are themselves DRC clean, and will cause no DRC violations when instantiated in the design.

