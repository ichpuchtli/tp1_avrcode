/*
 * File: main.c
 * Author(s): Samuel Macpherson, Blair Zanon
 * Description: AVR Code for Team Project 1 2012 (Clockwork Orange)
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>


#define LED_ARRAY_MATRIX    LEDArray

#define LED_ARRAY_PORT      PORTD
#define LED_ARRAY_DDR       DDRD


#define _CTRL_LED_(A)                                   \
    do {                                                \
        LED_ARRAY_PORT = LED_ARRAY_MATRIX[A].PIO.PORT;  \
        LED_ARRAY_DDR  = LED_ARRAY_MATRIX[A].PIO.DDR;   \
    }while(0)

#define _KILL_LED_ARRAY()    \
    do {                     \
        _CTRL_LED_(1);       \
    }while(0)

#define _CTRL_ADC_(CH)                    \
    do {                                  \
        ADMUX  &= 0xF0;                   \
        ADMUX  |= ( channel & 0x0F );     \
        ADCSRA |= ( 1 << ADSC );          \
    }while(0)

#define HIGH 1
#define LOW  0

#define SECONDS     (ulTickSeconds)
#define MINUTES     (ulTickSeconds % 60)
#define HOURS       (ulTickSeconds % ( 60 * 60 ) )

typedef union LEDConfig {
    uint16_t wide;
    struct { uint8_t DDR; uint8_t PORT; } PIO;
} LED;

/* Time keeping counter */
static volatile unsigned int ulTickSeconds = 0; 

/*
 * PD0 ---------------------- 
 *        |     ^     |     |
 *        1     2     |     |
 *        v     |     |     ^
 * PD1 ----------     5     6
 *        |     ^     v     |
 *        3     4     |     |
 *        v     |     |     |
 * PD2 ---------------------- 
 */

static const LED LEDArray[30] = {

//  Mask                Off
    0b0011111100111111, 0b0000000000000000,

//  DO1                 D02                 D03                 D04
    0b0000001100000001, 0b0000010100000001, 0b0000100100000001, 0b0001000100000001,
//  D05                 D06                 D07                 D08
    0b0010000100000001, 0b0000001100000010, 0b0000011000000010, 0b0000101000000010,
//  D09                 D10                 D11                 D12
    0b0001001000000010, 0b0010001000000010, 0b0000010100000100, 0b0000011000000100,
//  D13                 D14                 D15                 D16
    0b0000110000000100, 0b0001010000000100, 0b0010010000000100, 0b0000100100001000,
//  D17                 D18                 D19                 D20
    0b0000101000001000, 0b0000110000001000, 0b0001100000001000, 0b0010100000001000,
//  D21                 D22                 D23                 D24
    0b0001000100010000, 0b0001001000010000, 0b0001010000010000, 0b0001100000010000,
//  D25                 D26                 D27                 D28
    0b0011000000010000, 0b0010000100100000, 0b0010001000100000, 0b0010010000100000
};

/* | Hour | LED's
 * | 0    | None
 * | 1    | 2
 * | 2    | 3
 * | 3    | 4
 * | 4    | 5
 * | 5    | 6
 * | 6    | 7
 * | 7    | 8
 * | 8    | 9
 * | 9    | 1
 * | 10   | 11
 * | 11   | 12
 * | 12   | 1
 * | 13   | 1,2
 * | 14   | 1,3
 * | 15   | 1,4
 * | 16   | 1,5
 * | 17   | 1,6
 * | 18   | 1,7
 * | 19   | 1,8
 * | 20   | 1,9
 * | 21   | 1,10
 * | 22   | 1,11
 * | 23   | 1,12
 */
 

/* Minutes
 * 00      | 13              |  05    | 14                | 10    | 15
 * 01      | 28              |  06    | 14,28             | 11    | 15,28
 * 02      | 27,28           |  07    | 14,27,29          | 12    | 15,27,28
 * 03      | 26,27,28        |  08    | 14,26,27,28       | 13    | 15,26,27,28
 * 04      | 25,26,27,28     |  09    | 14,25,26,27,28    | 14    | 15,25,26,27,28
 *
 * 15      | 16              |  20    | 17                | 25    | 18
 * 16      | 16,28           |  21    | 17,28             | 26    | 18,28
 * 17      | 16,27,28        |  22    | 17,27,29          | 27    | 18,27,28
 * 18      | 16,26,27,28     |  23    | 17,26,27,28       | 28    | 18,26,27,28
 * 19      | 16,25,26,27,28  |  24    | 17,25,26,27,28    | 29    | 18,25,26,27,28
 *
 * 30      | 19              |  35    | 20                | 40    | 21
 * 31      | 19,28           |  36    | 20,28             | 41    | 21,28
 * 32      | 19,27,28        |  37    | 20,27,29          | 42    | 21,27,28
 * 33      | 19,26,27,28     |  38    | 20,26,27,28       | 43    | 21,26,27,28
 * 34      | 19,25,26,27,28  |  39    | 20,25,26,27,28    | 44    | 21,25,26,27,28
 *
 * 45      | 22              |  50    | 23                | 55    | 24
 * 46      | 22,28           |  51    | 23,28             | 56    | 24,28
 * 47      | 22,27,28        |  52    | 23,27,29          | 57    | 24,27,28
 * 48      | 22,26,27,28     |  53    | 23,26,27,28       | 58    | 24,26,27,28
 * 49      | 22,25,26,27,28  |  54    | 23,25,26,27,28    | 59    | 24,25,26,27,28
 */

// 8bit Timer used to trigger ADC conversions
void init_timer0(void)
{
    // Freq = F_CPU / prescaler / 255 
    // Freq ~= 15 Hz 
    
    // CS02 CS01 CS00  Description
    //  0    0    0    No Clock Source (Timer/Counter stopped)
    //  0    0    1    No Prescaling
    //  0    1    0    clk/8
    //  0    1    1    clk/64
    //  1    0    0    clk/256
    //  1    0    1    clk/1024
    //  1    1    0    External T0 pin failing edge
    //  1    1    1    External T0 pin rising edge
    
    // Normal Port operation
    TCCR0A = 0x00;

    // 1024 Prescaler 
    TCCR0B = (1<<CS02)|(0<<CS01)|(1<<CS00);

    // Just use overflow interrupt
    TIMSK0 = (0<<OCIE0B)|(0<<OCIE0A)|(1<<TOIE0);
    
}

// 16bit Timer used to count seconds
void init_timer1(void){

    // CS12 CS11 CS10  Description
    //  0    0    0    No Clock Source (Timer/Counter stopped)      
    //  0    0    1    No Prescaling
    //  0    1    0    clk/8
    //  0    1    1    clk/64
    //  1    0    0    clk/256
    //  1    0    1    clk/1024
    //  1    1    0    External T1 pin failing edge
    //  1    1    1    External T1 pin rising edge
 
    // Freq = F_CPU / prescaler / OCR1A

    OCR1AH = (unsigned char) ( (F_CPU / 1024) >> 8); 
    OCR1AL = (unsigned char) (F_CPU / 1024);

    // Normal Port operation
    TCCR1A = 0x00;

    // 1024 Prescaler & Enable CTC E.G. reset timer on OCR match
    TCCR1B = (1<<WGM12)|(1<<CS12)|(0<<CS11)|(1<<CS10); 
    TCCR1C = 0x00;

    // Don't need overflow or OCRB Compare match interrupt
    TIMSK1 = (0<<OCIE1B)|(1<<OCIE1A)|(0<<TOIE1);

}

// 8-bit timer used for pwm of leds
void init_timer2(void){

    // OCR2A
    //   /|  /|  /|  /|
    //  / | / | / | / |
    // /  |/  |/  |/  |
    //
    // ___|---|___|---|_  -> LED_MATRIX
    
    // Freq = F_CPU / prescaler / 2 * OCR2A
    // Freq ~= 50 Hz 
    // OCR2A ~= F_CPU / prescaler / 50 / 2 
    // OCR2A ~= 195 but actuall clk speed less than F_CPU at room temp so use
    // OCR2A = 160 
    
    // This setup is the same as TCCR2A = 0; TIMSK2 = (1<<TOIE2)
    OCR2A = 160;

    // CS22 CS21 CS20  Description
    //  0    0    0    No Clock Source (Timer/Counter stopped)      
    //  0    0    1    No Prescaling
    //  0    1    0    clk/8
    //  0    1    1    clk/32
    //  1    0    0    clk/64 
    //  1    0    1    clk/128
    //  1    1    0    clk/256
    //  1    1    1    clk/1024
    
    // 1024 Prescaler
    TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20);

    // Normal port operation and enable CTC reset timer of OCR2A match
    TCCR2A = (1<<WGM21);

    // Just use OCR2A Compare match interrupt
    TIMSK2 = (0<<OCIE2B)|(1<<OCIE2A)|(0<<TOIE2);

}

void init_ADC(void){

    // ADC voltage reference
    // REFS1 REFS0 Description
    //  0     0    AREF
    //  0     1    AVcc
    //  1     0    Reserved
    //  1     1    Internal 1.1V
 
    // When ADLAR = 1 (Left Adjusted)
    //---------------------------------------------------------
    //| ADC9 | ADC8 | ADC7 | ADC6 | ADC5 | ADC4 | ADC3 | ADC2 | ADCH
    //---------------------------------------------------------
    //| ADC1 | ADC0 |      |      |      |      |      |      | ADCL
    //---------------------------------------------------------

    // ADCH = Vin * 1024 / VREF
    
    // Left adjust 10 bit ADC value just need to read ADCH 8-bit precision
    ADMUX = (0<<REFS1)|(0<<REFS0)|(1<<ADLAR);

    // ADC Prescaler Selections
    // ADPS2 ADPS1 ADPS0 Division Factor
    //   0     0     0          2
    //   0     0     1          2
    //   0     1     0          4
    //   0     1     1          8
    //   1     0     0         16
    //   1     0     1         32
    //   1     1     0         64
    //   1     1     1        128
    
    // Enable ADC, Enable Interrupt and 128 clk division factor
    ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

    // ADC Auto Trigger Sources
    // ADTS2 ADTS1 ADTS0 Trigger Source
    //  0    0     0    Free Running mode
    //  0    0     1    Analog Comparator
    //  0    1     0    External Interrupt Request 0
    //  0    1     1    Timer/Counter0 Compare Match A
    //  1    0     0    Timer/Counter0 Overflow
    //  1    0     1    Timer/Counter1 Compare Match B
    //  1    1     0    Timer/Counter1 Overflow
    //  1    1     1    Timer/Counter1 Capture Event

    // Timer/Counter 0 prepared for 38Hz ADC polling
    ADCSRB = (0<<ADTS2)|(0<<ADTS1)|(0<<ADTS0);

    // Disable all ADC
    DIDR0 = (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);
}


int main(void) {


    init_timer0();
    init_timer1();
    init_timer2();

    init_ADC();

    sei();

    for ( ; ; ) { }

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
  
    // _CTRL_ADC_(PHOTODIODE1);
}

// 16-bit Timer1, freq = 1Hz
ISR(TIMER1_COMPA_vect){

    ulTickSeconds++;

}

// 8-bit Timer2, freq = 50Hz
// Used for PWM leds 
ISR(TIMER2_COMPA_vect){

    // Flip LED on status

}

// 10-bit ADC value left adjusted
// 8-bit precision with ADCH
ISR(ADC_vect) {

    // ADCH = Vin * 255 / VREF
    uint8_t value = ADCH;

}
