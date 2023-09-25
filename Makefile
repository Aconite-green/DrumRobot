# Declare variables
CC = gcc
CFLAGS = -Wall -O2 -g
INCLUDE = -I./include
SRCDIR = ./src
BINDIR = ./

SOURCES := $(wildcard $(SRCDIR)/*.c)
OBJFILES := $(patsubst %.c, %.o, $(SOURCES))



# Phony targets
.PHONY: all clean

# Build target
all: $(BINDIR)/main.out 

$(BINDIR)/main.out: $(SRCDIR)/main.o $(OBJFILES)
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) -lm

# Pattern rules
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# Clean rule
clean:
	rm -f $(SRCDIR)/*.o $(BINDIR)/main.out 
