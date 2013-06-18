TARGET="p.hex"
CC=msp430-gcc
CFLAGS=-std=c99 -mmcu=msp430g2553 -O2


all: $(TARGET)

$(TARGET): a.o pff2a/pff.o pff2a/diskio.o MMC_lib/mmc.o MMC_lib/hal_SPI.o
	$(CC) $(CFLAGS) -o $(TARGET) "$<"

.c.o:
	$(CC) $(CFLAGS) -o "$@" -c "$<"

install:
	mspdebug rf2500 "prog $(TARGET)"
