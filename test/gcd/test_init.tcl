set dir pdn/src/scripts
set config pdn/test/gcd/PDN.cfg

source $dir/pkgIndex.tcl
package require pdn

set db [dbDatabase_create]
set lef_parser [new_lefin $db true]
set def_parser [new_defin $db]

$lef_parser createTechAndLib nangate45 ./OpenDB/tests/data/Nangate45/NangateOpenCellLibrary.mod.lef 

set chip [$def_parser createChip [$db getLibs] ./OpenDB/tests/data/gcd/floorplan.def]

set block [$chip getBlock]

pdn apply $block $config

set def_writer [new_defout]
$def_writer writeBlock $block [$block getName]_postT8.def

