CC = g++
CFLAGS = -Wall -O2 -g -std=c++17 -fPIC `pkg-config --cflags Qt5Widgets`
INCLUDE = -I./include -I./lib `pkg-config --cflags Qt5Widgets`
LDFLAGS = -lm -lpthread -lstdc++fs -L./lib -lUSBIO_64 `pkg-config --libs Qt5Widgets`
SRCDIR = ./src
BINDIR = ./src/main.out

# Automatically include all .cpp files from the src directory
SOURCES := $(wildcard $(SRCDIR)/*.cpp)
OBJFILES := $(patsubst %.cpp, %.o, $(SOURCES))

# Qt MOC 처리
MOC = moc
MOC_HEADERS = $(wildcard ./include/*.hpp)  # Adjust to include all headers in include directory
MOC_SRC = $(patsubst %.hpp, %.moc.cpp, $(MOC_HEADERS))
MOC_OBJ = $(patsubst %.moc.cpp, %.moc.o, $(MOC_SRC))

# Build target
all: $(BINDIR)

$(BINDIR): $(OBJFILES) $(MOC_OBJ)
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) $(LDFLAGS)

# Pattern rules
%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# MOC 규칙
%.moc.cpp: %.hpp
	$(MOC) $< -o $@

%.moc.o: %.moc.cpp
	$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# Clean rule
clean:
	rm -f $(SRCDIR)/*.o $(SRCDIR)/*.moc.cpp $(SRCDIR)/*.moc.o $(BINDIR)
