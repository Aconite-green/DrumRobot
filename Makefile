# Declare variables
CC = gcc
CFLAGS = -Wall -O2 -g
INCLUDE = -I./include
SRCDIR = ./src
BINDIR = ./
TESTSRCDIR = ./tests
SOURCES := $(wildcard $(SRCDIR)/*.c)
OBJFILES := $(patsubst %.c, %.o, $(SOURCES))
TEST_SOURCES := $(wildcard $(TESTSRCDIR)/*.c)
TEST_OBJFILES := $(patsubst %.c, %.o, $(TEST_SOURCES))
NO_MAIN_OBJFILES := $(filter-out $(SRCDIR)/main.o, $(OBJFILES))

# Phony targets
.PHONY: all clean

# Build target
all: $(BINDIR)/main.out $(TESTSRCDIR)/tmotor_path.out

$(BINDIR)/main.out: $(SRCDIR)/main.o $(OBJFILES)
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) -lm

$(TESTSRCDIR)/%.out: $(TESTSRCDIR)/%.o $(NO_MAIN_OBJFILES)
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) -lm

# Pattern rules
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# Clean rule
clean:
	rm -f $(SRCDIR)/*.o $(BINDIR)/main.out $(TESTSRCDIR)/*.o $(TESTSRCDIR)/*.out
