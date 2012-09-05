#include "timers.h"

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
    TCCR1B = (1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10); 
    TCCR1C = 0x00;

    // Don't need overflow or OCRB Compare match interrupt
    TIMSK1 = (0<<OCIE1B)|(1<<OCIE1A)|(0<<TOIE1);

}

// 8-bit timer used to switch TriState PIO for charlieplexing
void init_timer2(void){

    // Freq = F_CPU / prescaler / 2 * OCR2A
    // OCR2A = F_CPU / prescaler / Freq / 2 
    
    //OCR2A = F_CPU / 1024 / ( 50 * LED_CLK_SET_SIZE ) / 2;
    OCR2A =  2;

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
    TCCR2B = (0<<CS22)|(0<<CS21)|(1<<CS20);

    // Normal port operation and enable CTC reset timer of OCR2A match
    TCCR2A = (1<<WGM21);

    // Just use OCR2A Compare match interrupt
    TIMSK2 = (0<<OCIE2B)|(1<<OCIE2A)|(0<<TOIE2);

}
