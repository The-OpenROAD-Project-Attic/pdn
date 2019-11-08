set db [dbDatabase_create]

# set libs [read_lef $db $env(LEF_FILES)]
# set chip [read_def $libs $env(DEF_FILE)]

set lef_parser [new_lefin $db true]
set def_parser [new_defin $db]

$lef_parser createTechAndLib tech $env(LEF_FILES)
$def_parser createChip [$db getLibs] $env(DEF_FILE)

export_db $db $env(DESIGN).fp.odb

