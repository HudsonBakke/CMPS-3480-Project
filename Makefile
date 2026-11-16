CXX := g++
CPPFLAGS := -I.
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra
LDLIBS := -lX11 -lGL -lGLU -lm

TARGET := project
SOURCES := project.cpp Framework.cpp Scene.cpp Character.cpp Math.cpp perlin.cpp
HEADERS := $(wildcard *.h)

.PHONY: all clean

all: $(TARGET)

# Compile and link together: no persistent .o/.d files or build subdirectory.
$(TARGET): $(SOURCES) $(HEADERS) libggfonts.a Makefile
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) $(SOURCES) libggfonts.a -o $@ $(LDLIBS)

# Also remove intermediate files left by the previous Makefile.
clean:
	$(RM) $(TARGET) *.o *.d
