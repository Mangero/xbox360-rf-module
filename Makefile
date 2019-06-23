# Filenames
BIN=360_receiver
IDIR=/usr/lib/avr/include
BUILD_DIR=./build
OBJ=${BIN}.o

# Programming
MCU=attiny85
CPU_FREQ=1000000UL
PORT=/dev/ttyACM0
PROGRAMMER=stk500v1
BAUDRATE=19200

# Compilation / File generation
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS= -I${IDIR} -Os -mmcu=${MCU} -DF_CPU=${CPU_FREQ} -Wall

${BUILD_DIR}/${BIN}.hex: ${BUILD_DIR}/${BIN}.elf
	${OBJCOPY} -j .text -j .data -O ihex $< $@

${BUILD_DIR}/${BIN}.elf: ${BUILD_DIR}/${OBJ}
	${CC} -mmcu=${MCU} -o $@ $^

${BUILD_DIR}/%.o: %.c
	test -d ${BUILD_DIR} || mkdir ${BUILD_DIR}
	${CC} -c ${CFLAGS} -o $@ $<

.PHONY: flash clean

flash: ${BUILD_DIR}/${BIN}.hex
	avrdude -c ${PROGRAMMER} -P ${PORT} -b ${BAUDRATE} -p ${MCU} -U flash:w:$<

clean:
	rm -f ${BUILD_DIR}/*.o ${BUILD_DIR}/*.elf ${BUILD_DIR}/*.hex
