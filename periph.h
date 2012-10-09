#include <avr/io.h>

/* Initialize Analog Comparator used for optical communication sync */
void configure_comparator(void);

/* Initialize ADC */
void configure_ADC(void);

// 8-bit timer
void configure_timer0(void);

// 16-bit timer
// Triggers ISR(TIMER1_COMPA_vect) @ 1 Hz
void configure_timer1(void);

// Setup 8-bit timer2 to be an IR Message Interceptor
void start_IR_interceptor(void);

// 8-bit timer
void configure_timer2(void);

// Initialize Pin Change Interrupts
void configure_PC_interrupts(void);

// Begin conversion on the ADC channel
void trigger_ADC(uint8_t channel);
