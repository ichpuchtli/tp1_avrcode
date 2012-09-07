#include <avr/io.h>

/* Initialize Analog Comparator used for optical communication sync */
void init_comparator(void);

/* Initialize ADC */
void init_ADC(void);

/* Trigger and ADC conversion on given channel */
void trigger_ADC(uint8_t channel);

// 8-bit timer
void init_timer0(void);

// 16-bit timer
// Triggers ISR(TIMER1_COMPA_vect) @ 1 Hz
void init_timer1(void);

// 8-bit timer
void init_timer2(void);

/* Initialize Pin Change Interrupts */
void init_pcint(void);
