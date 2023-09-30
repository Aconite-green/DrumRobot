# Declare variables
CC = g++
CFLAGS = -Wall -O2 -g
INCLUDE = -I./include
SRCDIR = ./src
BINDIR = ./

# Automatically include all .cpp files from the src directory
SOURCES := $(wildcard $(SRCDIR)/*.cpp)
OBJFILES := $(patsubst %.cpp, %.o, $(SOURCES))

# Phony targets
.PHONY: all clean

# Build target
all: $(BINDIR)/main.out

$(BINDIR)/main.out: $(OBJFILES)
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) -lm -lpthread

# Pattern rules
%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# Clean rule
clean:
	rm -f $(SRCDIR)/*.o $(BINDIR)/main.out
