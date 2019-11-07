set db [dbDatabase_create]

set libs [read_lef $db $env(LEF_FILES)]
set chip [read_def $libs $env(DEF_FILE)]

export_db $db $env(DESIGN).fp.odb

