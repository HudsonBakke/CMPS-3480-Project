CXX := g++
CPPFLAGS := -I.
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra
LDLIBS := -lX11 -lGL -lGLU -lm

TARGET := project
SOURCES := project.cpp Framework.cpp Scene.cpp Character.cpp Math.cpp perlin.cpp
HEADERS := $(wildcard *.h)

.PHONY: all clean test

all: $(TARGET)

# Compile and link together: no persistent .o/.d files or build subdirectory.
$(TARGET): $(SOURCES) $(HEADERS) libggfonts.a Makefile
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) $(SOURCES) libggfonts.a -o $@ $(LDLIBS)

# Numerical IK/contact tests do not require an X server or OpenGL headers.
.ik-tests: IKTests.cpp Character.cpp Math.cpp perlin.cpp Character.h Math.h Makefile
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -DCHARACTER_HEADLESS IKTests.cpp Character.cpp Math.cpp perlin.cpp -o $@

test: .ik-tests
	./.ik-tests

# Also remove intermediate files left by the previous Makefile.
clean:
	$(RM) $(TARGET) .ik-tests *.o *.d

