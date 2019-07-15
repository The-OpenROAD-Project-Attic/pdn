# subdir and objects in current dir
SUBDIRS	= 
#OBJECTS	= io.o io_parser_helper.o defw.o frGuidePrep.o frPinPrep.o frShapeUtil.o frUtil.o
OBJECTS	= io.o io_parser_helper.o defw.o io_guide.o frPinPrep.o frShapeUtil.o io_pin.o frAPG.o


all:subdirs ${OBJECTS}

	
clean:cleansubdirs
	rm -f ${OBJECTS}
	
include ${MAKEINCLUDE}
