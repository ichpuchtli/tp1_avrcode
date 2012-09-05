#include <avr/io.h>

// 8-bit timer
void init_timer0(void);

// 16-bit timer
// Triggers ISR(TIMER1_COMPA_vect) @ 1 Hz
void init_timer1(void);

// 8-bit timer
void init_timer2(void);
