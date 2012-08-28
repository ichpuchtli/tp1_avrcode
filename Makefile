# Makefile
CLFAGS= -mmcu=atmega48
CC=avr-gcc $(CFLAGS)

all : main.hex

main.hex : main.o
	avr-objcopy -j .text -j .data -O ihex main.o main.hex

main.o : main.c
	$(CC) -c -g -s -O -o main.o main.c 

clean :
	rm -f *.o *.hex *.cpp *.as 

uisp : all
	uisp -dprog=stk500 -dseria=/dev/ttyUSB0 -dpart=ATMEGA48 --erase --upload --verify if=main.hex

avrdude : all
	avrdude -F -y -c stk500 -p m48 -P /dev/ttyUSB0 -U flash:w:main.hex:i

ass :
	$(CC) -S -g -o main.as main.c

cpp : 
	$(CC) -E -g -o main.cpp main.c
