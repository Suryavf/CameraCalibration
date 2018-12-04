CC = g++
CFLAGS = -g -Wall -fopenmp -o3
SRCS = main.cpp structures.cpp mapping.cpp
PROG = out

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

build: main.cpp
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

clean:
	rm -rf out