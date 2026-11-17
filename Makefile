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

# -D defines a preprocessor macro for THIS compiler invocation only.
# CHARACTER_HEADLESS excludes drawing code so the numerical tests below need
# neither OpenGL headers/libraries nor an X server. Normal `make` does not set it.
.ik-tests: IKTests.cpp Character.cpp Math.cpp perlin.cpp Character.h Math.h Makefile
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -DCHARACTER_HEADLESS IKTests.cpp Character.cpp Math.cpp perlin.cpp -o $@

test: .ik-tests
	./.ik-tests

# Also remove intermediate files left by the previous Makefile.
clean:
	$(RM) $(TARGET) .ik-tests *.o *.d


