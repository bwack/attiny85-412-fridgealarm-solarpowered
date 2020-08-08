# assemble with AVR Assembler (AVRA)

MCU=attiny85
# run FOSC at 1 MHz
FUSE_L=0x62
# run FOSC at 128 kHz
#FUSE_L=0x44
FUSE_H=0xDF
FUSE_E=0xFF
F_CPU=1200000
AS=avra
AVRDUDE=avrdude
TARGET=main
SPEED=16
#SPEED=250
ASFLAGS=-I ../avra/include

SRCS = main.asm

all:
	${AS} ${ASFLAGS} -o ${TARGET} ${SRCS}

flash:
	${AVRDUDE} -p ${MCU} -c usbasp -B${SPEED} -U flash:w:${TARGET}.hex:i -F -P usb

fuse:
	$(AVRDUDE) -p ${MCU} -c usbasp -B${SPEED} -U hfuse:w:${FUSE_H}:m -U lfuse:w:${FUSE_L}:m efuse:w:${FUSE_L}:m

reset:
	$(AVRDUDE) -p $(MCU) -c usbasp -B300  -Ulfuse:v:0xE1:m

clean:
	rm -f *.asm~ *.hex *.obj *.cof
