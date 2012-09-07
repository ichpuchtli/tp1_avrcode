/*
 * File: main.c
 * Author(s): Samuel Macpherson, Blair Zanon
 * Description: AVR Code for Team Project 1 2012 (Clockwork Orange)
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "periph.h"
#include "AVRTime.h"

// Pins

#define PIEZO_PORT DDRD
#define PIEZO_PIN  (1 << 4)

#define PIEZO_ON (PiezoTicks && PiezoTicks--)

#define INVERT_PIN(PORT,PIN) (PORT ^= (1 << PIN))

#define LIGHT_SENSOR 0

// Initialize Onboard time
static volatile struct AVRTime_t AVRTimeNow;
static volatile struct AVRTime_t AVRAlarm;

static volatile uint8_t PiezoTicks = 0;
static volatile uint8_t AlarmOn = 0;


int main(void) {

    init_timer0();
    init_timer1();
    init_timer2();
    init_ADC();
    init_comparator();
    init_pcint();
    init_AVRTime(&AVRTimeNow);

    sei();

    for ( ; ; ) { }

    return 0;

}

ISR(PCINT1_vect){

    // Decode IR Message
    // __--_--___--__
}

ISR(ANALOG_COMP_vect){

    // Decode Optical Message
    // __--_--___--__

}

// 8-bit Timer0, 15Hz
ISR(TIMER0_OVF_vect){

    trigger_ADC(LIGHT_SENSOR);
    
    if(PIEZO_ON) INVERT_PIN(PIEZO_PORT,PIEZO_PIN);
}

// 16-bit Timer1, 1Hz
ISR(TIMER1_COMPA_vect){ 

    tick_AVRTime(&AVRTimeNow);

    if(AlarmOn && (comp_AVRTime(&AVRTimeNow, &AVRAlarm) == 0) ){
        PiezoTicks = 15; 
    }
}


// 8-bit Timer2, 500
ISR(TIMER2_COMPA_vect){
    // _CTRL_LED_(LEDSet[LEDSetCounter++ % LEDSetSize]);
    // Rotate LEDs
}

// 10-bit ADC value left adjusted
// 8-bit precision with ADCH
ISR(ADC_vect) {
    // ADCH = Vin * 255 / VREF
    uint8_t value = ADCH;
    
    if(value < 50){
        //Dim LEDs
    }
}


ISR(USART_RX_vect){
    //uint8_t byte = UDR;

}

