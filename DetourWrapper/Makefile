# Compiler settings
CC = g++
CFLAGS = -Wall -g -O2 -fPIC -I$(SRC_DIR1) -I$(INC_DIR)

# Output binary
TARGET = libDetourWrapper.so

# Directory of the current Makefile
MAKEFILE_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# Source directories
SRC_DIR1 = $(MAKEFILE_DIR)
SRC_DIR2 = $(MAKEFILE_DIR)/Detour/Source
INC_DIR = $(MAKEFILE_DIR)/Detour/Include

# Source files
SRCS = $(SRC_DIR1)/Detour.cpp $(SRC_DIR1)/DllExport.cpp
SRCS += $(wildcard $(SRC_DIR2)/*.cpp)

# Object files
OBJS = $(SRCS:.cpp=.o)

# Default target
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -shared $(CFLAGS) -o $(TARGET) $(OBJS)

# Generic rule for compiling .cpp to .o
%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

# Clean target
clean:
	rm -f $(TARGET) $(OBJS)
