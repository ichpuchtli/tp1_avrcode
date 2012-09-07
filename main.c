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

// Multiplexing LEDs
//
#define MATRIX_COL_PORT PORTD // PD0..5
#define MATRIX_ROW_PORT PORTB // PD0..7

#define MATRIX_COL_DDR DDRD // PD0..5
#define MATRIX_ROW_DDR DDRB // PD0..7

#define PIEZO_PORT DDRD
#define PIEZO_PIN  4

#define PIEZO_ON (PiezoTicks && PiezoTicks--)

#define INVERT_PIN(PORT,PIN) (PORT ^= (1 << PIN))

#define LIGHT_SENSOR 0

#define _SELECT_LED_(DI)                                \
    do {                                                \
        if((DI) > 0 && (DI) < 32){                      \
            MATRIX_COL_PORT = ( 1 << ( DI % 5) );       \
            MATRIX_ROW_PORT = ( 1 << ( DI / 6) );       \
        }                                               \
    }while(0)

#define _SETUP_LED_MUX_()                   \
    do {                                    \
        MATRIX_COL_DDR = 0x1F;              \
        MATRIX_ROW_DDR = 0x3F;              \
    }while(0)


// Initialize Onboard time
static struct AVRTime_t AVRTimeNow = {0};
static struct AVRTime_t AVRAlarm;

static volatile uint8_t PiezoTicks = 0;
static volatile uint8_t AlarmOn = 0;

static volatile uint8_t LEDSet[32] = {0};
static volatile uint8_t LEDSetSize = 0;
static volatile uint8_t LEDSetIndex = 0;

int main(void) {

    init_timer0();
    init_timer1();
    init_timer2();
    init_ADC();
    init_comparator();
    init_pcint();

    _SETUP_LED_MUX_();

    sei();

    for ( ; ; ) {

        // Rotate LEDs (Multiplexing)
        _SELECT_LED_( LEDSet [ LEDSetIndex++ % LEDSetSize ] );
    }

    return 0;

}

ISR(PCINT1_vect){

    //init_timer2(freq,count,&bitbuffer);
}

ISR(ANALOG_COMP_vect){

    //init_timer2(freq,count,&bitbuffer);
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
        // Sound Alarm (Beep 15 Times)
        PiezoTicks = 15; 
        AlarmOn = 0;
    }

}

// 8-bit Timer2
ISR(TIMER2_COMPA_vect){

    // Bit Stream 
    //       0 1 2 3 4 5 6 7 
    //     / | | | | | | | |
    // ----|__--__--____--__
    //     ^ Falling Edge Triggers PCINT
    // 

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

