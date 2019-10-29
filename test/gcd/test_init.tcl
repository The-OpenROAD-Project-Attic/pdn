set dir pdn/src/scripts
source pdn/src/scripts/pkgIndex.tcl

set db [dbDatabase_create]
set lef_parser [new_lefin $db true]
set def_parser [new_defin $db]

$lef_parser createTechAndLib nangate45 ./OpenDB/tests/data/Nangate45/NangateOpenCellLibrary.mod.lef 

set chip [$def_parser createChip [$db getLibs] ./OpenDB/tests/data/gcd/floorplan.def]

set block [$chip getBlock]

