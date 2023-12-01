# Declare variables
CC = g++
CFLAGS = -Wall -O2 -g -std=c++17 -fPIC
INCLUDE = -I./include -I./lib
LDFLAGS = -lm -lpthread -lstdc++fs -L./lib -lUSBIO_64
SRCDIR = ./src
BINDIR = ./src/main.out

# Automatically include all .cpp files from the src directory
SOURCES := $(wildcard $(SRCDIR)/*.cpp)
OBJFILES := $(patsubst %.cpp, %.o, $(SOURCES))

# Build target
all: $(BINDIR)

$(BINDIR): $(OBJFILES)
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) $(LDFLAGS)

# Pattern rules
%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# Clean rule
clean:
	rm -f $(SRCDIR)/*.o $(BINDIR)
