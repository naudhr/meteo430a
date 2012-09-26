TARGET="p.hex"
CC=msp430-gcc
CFLAGS=-std=c99 -mmcu=msp430g2553 -O2


$(TARGET): a.c
	$(CC) $(CFLAGS) -o $(TARGET) "$<"

all: $(TARGET)

install:
	mspdebug rf2500 "prog $(TARGET)"
