# Makefile for Umbrella

# ==============================
# Platform-specific library flags
# ==============================
PLATFORM := $(shell uname)

ifeq "$(PLATFORM)" "MINGW32_NT-6.1"
	SYSTEM_LIBS = -lmingw32 -mwindows

	SDL_FLAGS = -IC:/MinGW/include/SDL
	SDL_LIBS = -LC:/MinGW/lib/SDL -mwindows -lmingw32 -lSDLmain -lSDL_mixer -lSDL

	SDL_MIXER_FLAGS =
	SDL_MIXER_LIBS = -LC:/MinGW/lib/SDL -lSDL_mixer

	GL_LIBS = -lopengl32 -lglu32
endif

ifeq "$(PLATFORM)" "Darwin"
	SYSTEM_LIBS = -framework Cocoa

	SDL_FLAGS = -I/Library/Frameworks/SDL.framework/Headers
	SDL_LIBS = -framework SDL

	SDL_MIXER_FLAGS = -I/Library/Frameworks/SDL_mixer.framework/Headers
	SDL_MIXER_LIBS = -framework SDL_mixer

	GL_LIBS = -framework OpenGL
endif

# TODO: Never compiled on Ubuntu.
ifeq "$(PLATFORM)" "Linux"
	SDL_FLAGS = `sdl-config --cflags`
	SDL_LIBS  = `sdl-config --libs` 

	# SDL_IMAGE_FLAGS = 
	# SDL_IMAGE_LIBS = 

	# SDL_MIXER_FLAGS = 
	# SDL_MIXER_LIBS = 

	GL_LIBS += -lGL -lGLU
endif


# ==============================
# Project directories
# ==============================
SRC_DIR = src
OBJ_DIR = obj


# ==============================
# Source and object file list
# ==============================
CPP_FILES := $(wildcard $(SRC_DIR)/*/*.cpp) $(SRC_DIR)/main.cpp
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(CPP_FILES))

# Adds SDLMain.o as an object file.
ifeq "$(PLATFORM)" "Darwin"
OBJ_FILES += $(OBJ_DIR)/SDLMain.o
endif


# ==============================
# Compiler, flags, binary name
# ==============================
CXX = g++
WARNINGS = -Wall
CXXFLAGS = --std=c++11 $(WARNINGS) -I. -I$(SRC_DIR) $(SDL_FLAGS) $(SDL_MIXER_FLAGS)
LDFLAGS = $(SYSTEM_LIBS) $(SDL_MIXER_LIBS) $(SDL_LIBS) $(GL_LIBS)
BIN = ys
RM = rm -f

# Uses the MacPorts installation of g++ to avoid intefering with Xcode.
ifeq "$(PLATFORM)" "Darwin"
CXX = /opt/local/bin/g++
endif


# ==============================
# Targets
# ==============================
.PHONY: all run clean veryclean profile native

all: $(BIN)

run:
	$(MAKE)
	./$(BIN)

clean:
	$(RM) $(OBJ_FILES)

veryclean:
	$(RM) $(OBJ_FILES) $(BIN)

profile: CXXFLAGS += -pg
profile: all

# TODO: OS X doesn't support -march=native:
# http://stackoverflow.com/questions/10327939/erroring-on-no-such-instruction-while-assembling-project-on-mac-os-x-lion
#native: CXXFLAGS += -march=native -O2 -pipe
native: CXXFLAGS += -O2 -pipe
native: all


# ==============================
# Rules
# ==============================
$(BIN): $(OBJ_FILES)
	$(CXX) $^ $(LDFLAGS) -o $@

# Collects object files in a separate directory.
$(OBJ_DIR)/%.o : $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $^ -o $@

# Uses gcc to compile SDLMain.o from SDLMain.m.
# SDLMain.h/m is Objective-C glue code to produce a Cocoa application.
ifeq "$(PLATFORM)" "Darwin"
CC = gcc
CFLAGS = -Wall -I. -I$(SRC_DIR) $(SDL_FLAGS)
$(OBJ_DIR)/SDLMain.o: $(SRC_DIR)/SDLMain.m
	$(CC) $(CFLAGS) -c $^ -o $@
endif

