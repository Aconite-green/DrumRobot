# Declare variables
CC = g++
CFLAGS = -Wall -O2 -g -std=c++17  # C++17 표준 사용
INCLUDE = -I./include
LDFLAGS = -lm -lpthread -lstdc++fs  # 여기에 -lstdc++fs 추가
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
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) $(LDFLAGS)  # 여기서 LDFLAGS 사용

# Pattern rules
%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# Clean rule
clean:
	rm -f $(SRCDIR)/*.o $(BINDIR)/main.out
