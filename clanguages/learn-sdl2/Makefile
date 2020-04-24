OBJECTS = learning.c
RES_FILE = learning.res

#CC specifies which compiler we're using
CC = gcc

INCLUDE_PATHS = -IC:\tools\install-free\mingw17.1\include\SDL2

LIBRARY_PATHS = -LC:\tools\install-free\mingw17.1\lib

# -w suppresses all warnings
# -Wl,-subsystem,windows gets rid of the console window
COMPILER_FLAGS = -w -Wl,-subsystem,windows

#LINKER_FLAGS specifies the libraries we're linking against
#Original tutorial version below:
#LINKER_FLAGS = -lmingw32 -lSDL2main -lSDL2_mixer -lSDL2
LINKER_FLAGS = -lmingw32 -lSDL2main -lSDL2_mixer -lSDL2\
	-loleaut32 -limm32 -lversion -lole32\
	-lgdi32 -lsetupapi -lwinmm -lws2_32

EXE_NAME = test

COMBINED_COMPILE_CMD = $(CC) $(OBJECTS) -o $(EXE_NAME)\
	$(INCLUDE_PATHS) $(LIBRARY_PATHS)\
	$(COMPILER_FLAGS) $(LINKER_FLAGS) $(RES_FILE)

#This is the target that compiles our executable
${EXE_NAME} : $(OBJECTS) $(RES_FILE)
	$(COMBINED_COMPILE_CMD)

learning.res : learning.rc
	windres learning.rc -O coff -o $(RES_FILE)

.PHONY: clean
clean:
	del $(RES_FILE)