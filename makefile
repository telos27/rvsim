# Makefile for rvsim project

CC=gcc
CFLAGS=-Wall
TARGET=rv-emulator

all: $(TARGET)

$(TARGET): rv-emulator.c
	$(CC) $(CFLAGS) rv-emulator.c -o $(TARGET)

clean:
	rm -f $(TARGET)

