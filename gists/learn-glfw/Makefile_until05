#OBJS specifies which files to compile as part of the project
OBJS = learning.cpp

#CC specifies which compiler we're using
CC = g++

#INCLUDE_PATHS specifies the additional include paths we'll need
INCLUDE_PATHS = -IC:\tools\mingw17.1\include\SDL2
# -IC:\mingw_dev_lib\include -IC:\tools\mingw17.1\x86_64-w64-mingw32\include

#LIBRARY_PATHS specifies the additional library paths we'll need
LIBRARY_PATHS = -LC:\tools\mingw17.1\lib 
# -LC:\tools\mingw17.1\x86_64-w64-mingw32\lib

#COMPILER_FLAGS specifies the additional compilation options we're using
# -w suppresses all warnings
# -Wl,-subsystem,windows gets rid of the console window
COMPILER_FLAGS = -w -Wl,-subsystem,windows

#LINKER_FLAGS specifies the libraries we're linking against
#Original tutorial version below:
#LINKER_FLAGS = -lmingw32 -lSDL2main -lSDL2_mixer -lSDL2
LINKER_FLAGS = -lmingw32 -lSDL2main -lSDL2_mixer -lSDL2 -loleaut32 -limm32 -lversion -lole32 -lgdi32 -lsetupapi -lwinmm -lws2_32

#OBJ_NAME specifies the name of our exectuable
OBJ_NAME = test

#This is the target that compiles our executable
all : $(OBJS)
	$(CC) $(OBJS) -o $(OBJ_NAME) $(INCLUDE_PATHS) $(LIBRARY_PATHS) $(COMPILER_FLAGS) $(LINKER_FLAGS)