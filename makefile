# Makefile for rvsim project

CC=gcc
CFLAGS=-Wall
TARGET=rv-emulator

all: $(TARGET)

$(TARGET): rv-emulator.c clint.c mmu.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(TARGET)

