

all : main

main : main.c
	avr-gcc main.c -o main

clean :
	rm main
