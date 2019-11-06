set dir ../../src/scripts
source $dir/pkgIndex.tcl

package require pdn

set db [dbDatabase_create]
import_db $db $env(DESIGN).fp.odb

set block [[$db getChip] getBlock]
pdn apply $block PDN.cfg

write_def $block $env(DESIGN)_pdn.def DEF_5_6
