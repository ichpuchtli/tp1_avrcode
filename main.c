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

/////////////////////////////// OUTPUTS ///////////////////////////////////////
// Multiplexing LEDs
#define MATRIX_COL_PORT         PORTD // PORTD0..4
#define MATRIX_ROW_PORT         PORTB // PORTB0..5
#define MATRIX_COL_DDR          DDRD // PD0..4
#define MATRIX_ROW_DDR          DDRB // PD0..5

// Piezo Speaker
#define PIEZO_PORT              PORTC
#define PIEZO_DDR               DDRC
#define PIEZO_PIN               1
#define PIEZO_ON                (PiezoTicks && PiezoTicks--)

////////////////////////// SOFTWARE INTERRUPTS ////////////////////////////////
// PCIE0 => PCINT0..7 PORTB
#define SOFT_INT0_PORT          PINB
#define SOFT_INT0_PIN           6 // (PCINT6) 
#define SOFT_INT0_vect          PCINT0_vect

// Currently Used by IR Receiver to notify of incoming data
// PCIE1 => PCINT8..14 PORTC
#define SOFT_INT1_PORT          PINC
#define SOFT_INT1_PIN           0 // (PCINT8) 
#define SOFT_INT1_vect          PCINT1_vect

// PCIE2 => PCINT016..23 PORTD
#define SOFT_INT2_PORT          PIND
#define SOFT_INT2_PIN           5 // (PCINT21)
#define SOFT_INT2_vect          PCINT2_vect

// Trigger a software interrupt
#define FIRE_SOFT_INT(INT)      (SOFT_INT##INT_PORT |= (1 << SOFT_INT##INT_PIN))

/////////////////////////// IR COMMUNICATION //////////////////////////////////
#define IR_INCOMING_INT         SOFT_INT1_vect
#define IR_PORT                 PORTC 
#define IR_DDR                  DDRC
#define IR_PIN                  0

///////////////////////////// LIGHT SENSOR ////////////////////////////////////
#define LDR_ADC_CHANNEL         0   // PC0 ADC0
#define LIGHT_THRESHOLD         50  // Threshold Before Proportional Dimming

/////////////////////////////// Misc //////////////////////////////////////////

// Invert the state of a pin low->high or high->low
#define INVERT_PIN(PORT,PIN) (PORT ^= (1 << PIN))
// Returns 1 or 0 if pin is High or Low 
#define PROBE_PIN(PORT,PIN) ((PORT >> PIN) & 0x01)

// Initialize Onboard time
static struct AVRTime_t AVRTimeNow = {0};
static struct AVRTime_t AVRAlarm;

static volatile uint8_t PiezoTicks = 0;
static volatile uint8_t AlarmOn = 0;

static volatile uint8_t MinuteSet[4] = {0};
static volatile uint8_t MinuteSetSize = 0;
static volatile uint8_t MinuteSetIndex = 0;

static volatile uint8_t HourSet[3] = {0};
static volatile uint8_t HourSetSize = 0;
static volatile uint8_t HourSetIndex = 0;

volatile uint8_t PeriphIRBuff = 0;
volatile uint8_t PeriphIRBits = 0;

static inline void select_LED(uint8_t diode){
    
    if( diode != 0 ){
        MATRIX_COL_PORT = ( 1 << ( (diode & 0x1F) % 5) );
        MATRIX_ROW_PORT = ( 1 << ( (diode & 0x1F) / 6) );
    }

}

static inline void null_space(volatile uint8_t* buffer, uint16_t size){

    while(size--)
        buffer[size] = (uint8_t) 0x0;
}

static inline void trigger_ADC(uint8_t channel){

    // Select ADC channel
    ADMUX = channel;

    // Begin Conversion
    ADCSRA |= (1 << ADSC);

}

void process_packet(uint8_t buffer){

    //TODO
    return;
}

void configure_ports(void){

    // Multiplexing
    MATRIX_COL_DDR |= 0x1F;
    MATRIX_ROW_DDR |= 0x3F;

    MATRIX_COL_PORT &= 0xE0;
    MATRIX_ROW_PORT &= 0xC0;

    //Speaker (Alarm)
    PIEZO_PORT &= ~(1 << PIEZO_PIN);
    PIEZO_DDR |= (1 << PIEZO_PIN);

    //IR Receiver Pin Change Trigger
    IR_DDR &= ~(1 << IR_PIN);

}
void UpdateHourSet(uint8_t hour){

    null_space(HourSet,3);

    HourSetSize = 2;

    if( hour == 0 ){ return; }

    HourSet[0] = (hour % 12) + 13;
    HourSet[1] = 25;

    if( hour > 12 ) { // PM
        HourSet[2] = 28;
        HourSetSize = 3;
    }
}

void UpdateMinuteSet(uint8_t minute){

   null_space(MinuteSet,4);

   MinuteSetSize = 2;

   MinuteSet[0] = minute / 5 + 1;
   MinuteSet[1] = 26;

   if( minute % 5 == 0){
       return;
   }

   MinuteSet[2] = minute % 5 + 1;
   MinuteSet[3] = 27;

   MinuteSetSize = 4;
}

int main(void) {

    configure_timer0();
    configure_timer1();
    configure_timer2();

    configure_ADC();
    configure_comparator();
    configure_PC_interrupts();

    configure_ports();

    // GO!
    sei();

    for ( ; ; ) { }

    return 0;

}

// Software Interrupt 0
ISR(SOFT_INT0_vect){ }

// ISR Triggered by IR_RECEIVER (PCINT8) || SOFT_INT1
ISR(IR_INCOMING_INT){

    // Disable PCINT8 Interrupt
    // Don't won't another interrupt until buffer is full
    PCMSK1 = 0x0;

    PeriphIRBuff = 0x0;
    PeriphIRBits = 0x0;

    // Start Packet Interceptor
    start_IR_interceptor();
}


// Software Interrupt 2
ISR(SOFT_INT2_vect){ } 

// Optical Comm Trigger
ISR(ANALOG_COMP_vect){ }

// 8-bit Timer0, Variable Freq
ISR(TIMER0_OVF_vect){

    // Rotate LEDs (Multiplexing)
    // TODO Note: HourSet on avg smaller than minute set may
    // be brighter than minute configurations
    select_LED( HourSet [ HourSetIndex % HourSetSize ] );
    HourSetIndex++;

    _delay_ms(1);

    select_LED( MinuteSet [ MinuteSetIndex % MinuteSetSize ] );
    MinuteSetIndex++;
    
}

// 16-bit Timer1, 1Hz
ISR(TIMER1_COMPA_vect){ 

    tick_AVRTime(&AVRTimeNow);

    if(AlarmOn && (comp_AVRTime(&AVRTimeNow, &AVRAlarm) == 0) ){
        // Sound Alarm (Beep 15 Times)
        PiezoTicks = 15; 
        AlarmOn = 0;
    }

    if(PIEZO_ON) INVERT_PIN(PIEZO_PORT,PIEZO_PIN);

    trigger_ADC(LDR_ADC_CHANNEL);
}


// 8-bit Timer2
ISR(TIMER2_COMPA_vect){

    // Bit Stream 
    //       0 1 2 3 4 5 6 7 
    //     / | | | | | | | |
    // ----|__--__--____--__
    //     ^ Falling Edge Triggers PCINT
    
    PeriphIRBuff |= (PROBE_PIN(IR_PORT, IR_PIN) << PeriphIRBits++ );

    if( PeriphIRBits >= 8){

        // Disable Timer, by assigning no clock source
        TCCR2B = 0x0;

        process_packet(PeriphIRBuff);
        
        // Re-enable Pin Change Interrupt Ready for another message
        PCMSK1 = (1 << PCINT8);
    }
}

// 10-bit ADC value left adjusted
// 8-bit precision with ADCH
ISR(ADC_vect) {

    // ADCH = Vin * 255 / VREF
    uint8_t voltage = (ADCH * 1100)/255;
    
    if(voltage < LIGHT_THRESHOLD){
        //Dim LEDs
        //Slow Switch Speed
    }
}
