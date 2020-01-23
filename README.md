# OpenROAD-pdn

This directory contains power grid insertion flow for OpenROAD

If you want to use this as part of the OpenROAD project you should build it and use it from inside the integrated [openroad app](https://github.com/The-OpenROAD-Project/OpenROAD). The standalone version is available as legacy code.

This utility aims to simplify the process of adding a power grid into a floorplan. 
The aim is to specify a small set of power grid policies to be applied to the design, such as layers to use,
stripe width and spacing, then have the utility generate the actual metal straps. Grid policies can be defined
over the stdcell area, and over areas occupied by macros.

For more details refer to [doc/README.md](doc/README.md)
