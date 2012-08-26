#define __AVR_ATmega48P__

#include <avr/io.h>
#include <avr/interrupt.h>

// Notes
// PULL-UP resistors open drain?
// ADC clock speed, adc conversion speed request converison 38hz?

// Inputs
#define PHOTO_OPT ADC1D
#define PHOTO_DIM ADC2D
#define INFRA_RED ADC3D

#define LIGHT_THRESHOLD 0

// Outputs
#define LED_MATRIX // PORTD

#define HIGH 1
#define LOW  0


void start_ADC_conversion(int channel){

    ADMUX  &= 0xF0;
    ADMUX  |= ( channel & 0x0F );
    ADCSRA |= ( 1 << ADSC );

}


// 8bit Timer used to trigger ADC conversions
void init_timer0(void)
{
    // Freq = F_CPU / prescaler / 2 * 255 
    // Freq = 20000000 / 1024 / 512
    // Freq ~= 38 Hz 
    
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
    // Freq = 20000000 / 1024 / 19531
    // Freq ~= 1 Hz
    
    OCR1AL = (19531U >> 8);
    OCR1AH = (19531U << 8); 
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

    // TOP 0xFF
    // OCR2A
    //   /|  /|  /|  /|
    //  / | / | / | / |
    // /  |/  |/  |/  |
    //
    // ___|---|___|---|_  -> LED_MATRIX
    
    // Freq = F_CPU / prescaler / 2 * OCR2A
    // Freq = 20000000 / 1024 / 360
    // Freq ~= 54 Hz 
    
    // CS22 CS21 CS20  Description
    //  0    0    0    No Clock Source (Timer/Counter stopped)      
    //  0    0    1    No Prescaling
    //  0    1    0    clk/8
    //  0    1    1    clk/64
    //  1    0    0    clk/256
    //  1    0    1    clk/1024
    //  1    1    0    External T0 pin failing edge
    //  1    1    1    External T0 pin rising edge
    
    OCR2A = 180;

    // Normal port operation and enable CTC reset timer of OCR2A match
    TCCR2A = (1<<WGM21);

    // 1024 Prescaler
    TCCR2B = (1<<CS22)|(0<<CS21)|(1<<CS20);

    // Just use OCR2A Compare match interrupt
    TIMSK2 = (1<<OCIE2A);

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


// 8-bit Timer0, freq = 38Hz
ISR(TIMER0_OVF_vect){
  
    // start_ADC_conversion();

}

// 16-bit Timer1, freq = 1Hz
ISR(TIMER1_COMPA_vect){

    //seconds++;

}

// 8-bit Timer2, freq = 50Hz
// Used for PWM leds 
ISR(TIMER2_COMPA_vect){

    // Flip LED on status

}

// 10-bit ADC value left adjusted
// 8-bit precision with ADCH
ISR(ADC_vect) {

    // ADCH = Vin * 1024 / VREF
    char value = ADCH;

    switch(ADMUX & 0x0F){

        case PHOTO_OPT: break;

        case PHOTO_DIM:

            if( value < LIGHT_THRESHOLD) {
                 
            }

        break;
                        
        case INFRA_RED: break;
    }
}
