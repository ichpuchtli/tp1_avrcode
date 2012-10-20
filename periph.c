#include "periph.h"

void init_PC_interrupts(void){

    // PCIE0 => PCINT0..7 PORTB
    // PCIE1 => PCINT8..14 PORTC
    // PCIE2 => PCINT016..23 PORTD
    
    //Enable Interrupts on Set 1
    PCICR = (1 << PCIE0) | (1 << PCIE1) | (1 << PCIE2);

    // Enable PCINT6 for IR receiver and two software interrupts
    PCMSK0 = (1 << PCINT6);
    PCMSK1 = (1 << PCINT8);
    PCMSK2 = (1 << PCINT21);
}

void init_comparator(void){

    // Fire Interrupt on Rising Edge
    ACSR = (0 << ACO) | (1 << ACIE) | (0 << ACIS1) | (0 << ACIS0);  

    DIDR1 = (0 << AIN1D) | (0 << AIN0D);

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
    ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<ADLAR);
    ADMUX |= 2;

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
    ADCSRA = (1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(1<<ADIE)|(0<<ADPS2)|(0<<ADPS1)|(1<<ADPS0);

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
    DIDR0 = (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(0<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);

}

void trigger_ADC(uint8_t channel){

    // Clear MUX
    ADMUX  &= 0xF0; 

    // Enable Channel
    ADMUX  |= ( channel & 0x0F );

    ADCSRA |= (1<<ADSC);
}

// 8bit Timer used to trigger ADC conversions
void init_timer0(void)
{
    // Freq = F_CPU / prescaler / 255 
    // Freq = 20000000 / 256 / 255 = 306Hz 
    
    // CS02 CS01 CS00  Description
    //  0    0    0    No Clock Source (Timer/Counter stopped)
    //  0    0    1    No Prescaling
    //  0    1    0    clk/8
    //  0    1    1    clk/64
    //  1    0    0    clk/256
    //  1    0    1    clk/1024
    //  1    1    0    External T0 pin failing edge
    //  1    1    1    External T0 pin rising edge

    OCR0A = 255;

    // Normal Port operation
    TCCR0A = 0x00;

    // 256 Prescaler 
    TCCR0B = (0<<CS02)|(1<<CS01)|(0<<CS00);

    // Use overflow interrupt
    TIMSK0 = (0<<OCIE0B)|(1<<OCIE0A)|(0<<TOIE0);
    
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

    OCR1AH = (unsigned char) ( ((F_CPU / 1024) >> 8) + 1 ); 

    /* Ideal */
    /* OCR1AL = (unsigned char) (F_CPU / 1024); */

    /* Actuall */
    OCR1AL =  (unsigned char) 16;
        
    // Normal Port operation
    TCCR1A = 0x00;

    // 1024 Prescaler & Enable CTC E.G. reset timer on OCR match
    TCCR1B = (1<<WGM12)|(1<<CS12)|(0<<CS11)|(1<<CS10); 
    TCCR1C = 0x00;

    // Don't need overflow or OCRB Compare match interrupt
    TIMSK1 = (0<<OCIE1B)|(1<<OCIE1A)|(0<<TOIE1);
}

// 8-bit timer used intercept IR Stream
void init_timer2(void){

    // Freq = F_CPU / prescaler / 2 * OCR2A
    // OCR2A = F_CPU / prescaler / Freq / 2 

    // CS22 CS21 CS20  Description
    //  0    0    0    No Clock Source (Timer/Counter stopped)      
    //  0    0    1    No Prescaling
    //  0    1    0    clk/8
    //  0    1    1    clk/32
    //  1    0    0    clk/64 
    //  1    0    1    clk/128
    //  1    1    0    clk/256
    //  1    1    1    clk/1024
   
    // Freq = 125Hz
    OCR2A = 250;

    // Normal port operation and enable CTC reset timer of OCR2A match
    TCCR2A = 0x00;

    // 1024 prescaler, freq = F_CPU / 1024 / 256
    TCCR2B = (1<<CS22)|(1<<CS21)|(0<<CS20);

    // Use OCR2A Compare match interrupt
    TIMSK2 = (0<<OCIE2B)|(1<<OCIE2A)|(0<<TOIE2);
}

