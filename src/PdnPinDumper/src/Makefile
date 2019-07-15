# subdir and objects in current dir
#SUBDIRS	= db io ta dr drc
SUBDIRS	= db io ta dr drc
#OBJECTS	= main.o
SRC_DIR := .
SRC_FILES := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS = $(patsubst $(SRC_DIR)/%.cpp, $(SRC_DIR)/%.o, $(SRC_FILES))


all:subdirs ${OBJECTS}

	
clean:cleansubdirs
	rm -f ${OBJECTS}
	
include ${MAKEINCLUDE}
