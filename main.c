/*
 * File: main.c
 * Author(s): Samuel Macpherson, Blair Zanon
 * Description: AVR Code for Team Project 1 2012 (Clockwork Orange)
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include "timers.h"

#define LED_CLK_PORT                PORTD
#define LED_CLK_DDR                 DDRD

#define _CTRL_LED_(A)                                   \
    do {                                                \
        LED_CLK_PORT = (uint8_t) CHARLIE_MAP[A];        \
        LED_CLK_DDR  = CHARLIE_MAP[A] >> 8;             \
    }while(0)

#define _MEM_ZERO_(BUFF,SIZE)                     \
    do {                                          \
        for(uint16_t i = 0; i < SIZE; i++){       \
            BUFF[i] = 0;                          \
        }                                         \
    }while(0)



// Initialize Onboard time
static volatile struct AVRTime_t OnBoardTime = {0};


int main(void) {

    init_clock_face();

    init_timer0();
    init_timer1();
    init_timer2();

    init_ADC();

    sei();

    for ( ;; ) { }

    return 0;

}

/* Interrupt Vectors */
/* Interrupt Vector 0 is the reset vector. */
//#define INT0_vect         _VECTOR(1)   /* External Interrupt Request 0 */
//#define INT1_vect         _VECTOR(2)   /* External Interrupt Request 1 */
//#define PCINT0_vect       _VECTOR(3)   /* Pin Change Interrupt Request 0 */
//#define PCINT1_vect       _VECTOR(4)   /* Pin Change Interrupt Request 0 */
//#define PCINT2_vect       _VECTOR(5)   /* Pin Change Interrupt Request 1 */
//#define WDT_vect          _VECTOR(6)   /* Watchdog Time-out Interrupt */
//#define TIMER2_COMPA_vect _VECTOR(7)   /* Timer/Counter2 Compare Match A */
//#define TIMER2_COMPB_vect _VECTOR(8)   /* Timer/Counter2 Compare Match A */
//#define TIMER2_OVF_vect   _VECTOR(9)   /* Timer/Counter2 Overflow */
//#define TIMER1_CAPT_vect  _VECTOR(10)  /* Timer/Counter1 Capture Event */
//#define TIMER1_COMPA_vect _VECTOR(11)  /* Timer/Counter1 Compare Match A */
//#define TIMER1_COMPB_vect _VECTOR(12)  /* Timer/Counter1 Compare Match B */ 
//#define TIMER1_OVF_vect   _VECTOR(13)  /* Timer/Counter1 Overflow */
//#define TIMER0_COMPA_vect _VECTOR(14)  /* TimerCounter0 Compare Match A */
//#define TIMER0_COMPB_vect _VECTOR(15)  /* TimerCounter0 Compare Match B */
//#define TIMER0_OVF_vect   _VECTOR(16)  /* Timer/Couner0 Overflow */
//#define SPI_STC_vect      _VECTOR(17)  /* SPI Serial Transfer Complete */
//#define USART_RX_vect     _VECTOR(18)  /* USART Rx Complete */
//#define USART_UDRE_vect   _VECTOR(19)  /* USART, Data Register Empty */
//#define USART_TX_vect     _VECTOR(20)  /* USART Tx Complete */
//#define ADC_vect          _VECTOR(21)  /* ADC Conversion Complete */
//#define EE_READY_vect     _VECTOR(22)  /* EEPROM Ready */
//#define ANALOG_COMP_vect  _VECTOR(23)  /* Analog Comparator */
//#define TWI_vect          _VECTOR(24)  /* Two-wire Serial Interface */
//#define SPM_READY_vect    _VECTOR(25)  /* Store Program Memory Read */


// 8-bit Timer0, freq = 15Hz
ISR(TIMER0_OVF_vect){
  
    // _SIGNAL_ADC_(PHOTODIODE1);
}

// 16-bit Timer1, freq = 1Hz
ISR(TIMER1_COMPA_vect){

    _UPDATE_AVR_TIME_(OnBoardTime);

}

// 8-bit Timer2, freq = (50 * LED_CLK_SET_SIZE) Hz
ISR(TIMER2_COMPA_vect){

}

// 10-bit ADC value left adjusted
// 8-bit precision with ADCH
ISR(ADC_vect) {

    // ADCH = Vin * 255 / VREF
    uint8_t value = ADCH;

}


ISR(USART_RX_vect){
    
    //uint8_t byte = UDR;

}

