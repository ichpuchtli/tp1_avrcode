/*
 * File: main.c
 * Author(s): Samuel Macpherson, Blair Zanon
 * Description: AVR Code for Team Project 1 2012 (Clockwork Orange)
 * Licence: GPL/2.0 
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "periph.h"
#include "AVRTime.h"

/////////////////////////////// MULTIPLEXING //////////////////////////////////
// Multiplexing LEDs
#define MATRIX_COL_PORT         PORTD // PORTD0..4
#define MATRIX_ROW_PORT         PORTB // PORTB0..5
#define MATRIX_COL_DDR          DDRD // PORTD0..4
#define MATRIX_ROW_DDR          DDRB // PORTD0..5

#define MATRIX_COLS             0x05
#define MATRIX_ROWS             0x06

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

#define IR_DEF_0                0xA8
#define IR_DEF_1                0xB6
#define IR_DEF_2                0xAE
#define IR_DEF_3                0xBE
#define IR_DEF_4                0xB2
#define IR_DEF_5                0xAA
#define IR_DEF_6                0xBA
#define IR_DEF_7                0xB4
#define IR_DEF_8                0xAC
#define IR_DEF_9                0xBC

#define IR_DEF_MENU             0xB7
#define IR_DEF_ONOFF            0xB8
#define IR_DEF_BUY              0xAF
#define IR_DEF_FAV              0xB1
#define IR_DEF_AB               0xA9
#define IR_DEF_CHUP             0xAB
#define IR_DEF_CHDOWN           0xBB
#define IR_DEF_VOLUP            0xA3
#define IR_DEF_VOLDOWN          0xB3
#define IR_DEF_MUTE             0xB0
#define IR_DEF_ENTER            0xB9

///////////////////////////// LIGHT SENSOR ////////////////////////////////////
#define LDR_ADC_CHANNEL         0   // PC0 ADC0
#define LIGHT_THRESHOLD         50  // Threshold Before Proportional Dimming

/////////////////////////////// Misc //////////////////////////////////////////
// Piezo Speaker
#define PIEZO_PORT              PORTC
#define PIEZO_DDR               DDRC
#define PIEZO_PIN               1

/////////////////////////////// Misc //////////////////////////////////////////

// Invert the state of a pin low->high or high->low
#define INVERT_PIN(PORT,PIN) (PORT ^= (1 << PIN))
// Returns 1 or 0 if pin is High or Low 
#define PROBE_PIN(PORT,PIN) ((PORT >> PIN) & 0x01) 

/* Assumes number of constants is a multiple of 2, requirement due to use of
 * & not % */
#define ROTATE_ENUM(MODE, ENUMS) do { MODE += 1; MODE &= (ENUMS - 1); } while(0)
#define REWIND_ENUM(MODE, ENUMS) do { MODE -= 1; MODE &= (ENUMS - 1); } while(0)

enum DISPLAY_MODES { M_TIME_DISP, M_DATE_DISP, M_ALARM_DISP, M_WEATHER_DISP };

// Initialize On board time
static struct AVRTime_t AVRTimeNow = {0x00};
static struct AVRTime_t AVRAlarm;

static volatile uint8_t AlarmOn = 0x00;
static volatile uint8_t AlarmSet = 0x00;

static volatile uint8_t MinuteSet[4] = {0x00};
static volatile uint8_t MinuteSetSize = 0x00;
static volatile uint8_t MinuteSetIndex = 0x00;

static volatile uint8_t HourSet[4] = {0x00};
static volatile uint8_t HourSetSize = 0x00;
static volatile uint8_t HourSetIndex = 0x00;

static volatile uint8_t CUR_DISP_MODE = M_TIME_DISP;
static volatile uint8_t DISP_MODES = 0x04;

static void select_LED(uint8_t diode){
    
    if( diode != 0 ){
        MATRIX_COL_PORT = ( 1 << ( (diode & 0x1F) % MATRIX_COLS) );
        MATRIX_ROW_PORT = ( 1 << ( (diode & 0x1F) / MATRIX_ROWS) );
    }

}

static void null_space(uint8_t* buffer, uint16_t size){

    while(size--) *(size + buffer) = 0x00;
}

void process_num_stack(uint8_t* stack, uint8_t size){

    uint8_t hours,mins,secs;
    uint16_t days,years; 

    switch( CUR_DISP_MODE ){

    case M_TIME_DISP:
        /* Set Time */
        hours = stack[0]* 10 + stack[1];
        mins  = stack[2] * 10 + stack[3];
        secs  = stack[4] * 10 + stack[5];

        set_AVRTime_time(&AVRTimeNow, hours, mins, secs);
        break;

    case M_DATE_DISP:
        /* Set Date */
        years = stack[0] * 1000 + stack[1] * 100 + stack[2] * 10 + stack[3];
        days  = stack[4] * 100 + stack[5] * 10 + stack[6];

        set_AVRTime_date(&AVRTimeNow, years, days); 
        break;

    case M_ALARM_DISP:
        /* Set Alarm Time */
        hours = stack[0]* 10 + stack[1];
        mins  = stack[2] * 10 + stack[3];
        secs  = stack[4] * 10 + stack[5];

        set_AVRTime_time(&AVRAlarm, hours, mins, secs);
        break;

    case M_WEATHER_DISP: 
    default: break;

    }
}

static void dispatch_command(uint8_t byte){

    uint8_t num;

    static uint8_t stack[10] = {0x00};
    static uint8_t stackSize = 0x00;

    num = -0x01;

    switch ( byte ) {

        case IR_DEF_0: num = 0x00; break;
        case IR_DEF_1: num = 0x01; break;
        case IR_DEF_2: num = 0x02; break;
        case IR_DEF_3: num = 0x03; break;
        case IR_DEF_4: num = 0x04; break;
        case IR_DEF_5: num = 0x05; break;
        case IR_DEF_6: num = 0x06; break;
        case IR_DEF_7: num = 0x07; break;
        case IR_DEF_8: num = 0x08; break;
        case IR_DEF_9: num = 0x09; break;
    }

    if( num > 0x00 ){
        stack[stackSize++] = num;
        return;
    }

    switch( byte ) {

        /* Rotate Clock View Alarm, Date, Time, Weather */
        case IR_DEF_CHUP:
            REWIND_ENUM(CUR_DISP_MODE, DISP_MODES);
            break; 

        /* Rotate Clock View Alarm, Date, Time, Weather */
        case IR_DEF_CHDOWN:
            ROTATE_ENUM(CUR_DISP_MODE, DISP_MODES);
            break;

            /* Process the numbers collected depending on the current mode */
        case IR_DEF_ENTER:
            process_num_stack(stack,stackSize);
            /* Fallthrough */
            /* Reset number stack */
        case IR_DEF_MENU:
            null_space( stack, 10 );
            stackSize = 0x0;
            break;

        default : break;

    }

}

static void configure_ports(void){

    // Set Multiplexing outputs as low
    MATRIX_COL_PORT &= 0xE0;
    MATRIX_ROW_PORT &= 0xC0;

    // Set Multiplexing pins as outputs
    MATRIX_COL_DDR |= 0x1F;
    MATRIX_ROW_DDR |= 0x3F;

    //Set Speaker pin as an output and put the pin low 
    PIEZO_PORT &= ~(1 << PIEZO_PIN);
    PIEZO_DDR |= (1 << PIEZO_PIN);

    //IR Receiver Pin Change Trigger
    // Set IR Pin as inputs
    IR_DDR &= ~(1 << IR_PIN);

    DDRC = 0x00;

    DDRD = 0xFF;
    PORTD = 0xFF;

}

static void UpdateHourSet(uint8_t hour){

    null_space( (uint8_t*) HourSet, 3);

    HourSetSize = 2;

    if( hour == 0x00 ){ return; }

    HourSet[0] = (hour % 12) + 13;
    HourSet[1] = 25;

    if( hour > 12 ) { // PM
        HourSet[2] = 28;
        HourSetSize = 4;
    }
}

static void UpdateMinuteSet(uint8_t minute){

   null_space((uint8_t*) MinuteSet, 4);

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

    configure_timer0(); /* Multiplexer */
    configure_timer1(); /* Tick Seconds */
    configure_timer2(); /* Misc IR/Optical Interceptor */

    configure_ADC();
    configure_comparator();
    configure_PC_interrupts();

    configure_ports();

    //GO!
    sei();

    for ( ; ; ) asm("nop");

    return 0;

}

// Software Interrupt 0
ISR(SOFT_INT0_vect){ }

// ISR Triggered by IR_RECEIVER (PCINT8) || SOFT_INT1
ISR(IR_INCOMING_INT){

    // Disable PCINT8 Interrupt
    // Don't won't another interrupt until buffer is full
    PCICR = 0x0;

    uint8_t bitBuffer = 0x00;

    /* waste a few cycles here to sync with middle of level change */
    _delay_us(700);

    for( uint8_t bits = 0; bits < 8 ; bits++){

        bitBuffer |= (PROBE_PIN(IR_PORT, IR_PIN) << bits );

        _delay_us(1600);
    }

    // Fire Packet Ready Software Interrupt 
    // TODO Consider Bottom Half Processing in main loop
    dispatch_command( bitBuffer );

    // Restore Pin Change interrupt
    PCICR = (1 << PCIE1);
}

// Software Interrupt 2
ISR(SOFT_INT2_vect){ } 

// Optical Comm Trigger
ISR(ANALOG_COMP_vect){ 

    /*
     * ~~ -----+     +-----+          +---- ~~
     * ~~      |     |     |          |     ~~
     * ~~      |_____|     |__________|     ~~
     *
     *         ^ANALOG_COMP_vect
     *                     ^ANALOG_COMP_vect
     */

    /* Trigger SOFT_INT1 e.g IR_INCOMING */

}

// 8-bit Timer0, Variable Freq
ISR(TIMER0_OVF_vect){

    // Rotate LEDs (Multiplexing)
    // TODO Note: HourSet on avg smaller than minute set may
    // be brighter than minute configurations
    switch ( CUR_DISP_MODE ){
        
    case M_ALARM_DISP:
        /* Update Hour/Minute Set with alarm time */
        UpdateHourSet(AVR_HOUR(&AVRAlarm));
        UpdateMinuteSet(AVR_MIN(&AVRAlarm));
        /* Fallthrough */
    case M_TIME_DISP:

        select_LED( HourSet [ HourSetIndex++ & (HourSetSize - 1) ] );
        _delay_ms(1);
        select_LED( MinuteSet [ MinuteSetIndex++ & (MinuteSetSize - 1) ] );
        break;

    case M_DATE_DISP:
    case M_WEATHER_DISP:
    default : break;
    }

}

// 16-bit Timer1, 1Hz
ISR(TIMER1_COMPA_vect){ 

    /* update the tick count */
    tick_AVRTime(&AVRTimeNow);

    if( AVR_SEC(&AVRTimeNow) == 0 )
        UpdateMinuteSet( AVR_MIN(&AVRTimeNow) );

    if( AVR_MIN(&AVRTimeNow) == 0 )
        UpdateHourSet( AVR_HOUR(&AVRTimeNow) );

    /* compare the current time with the alarm time */ 
    if(AlarmSet && (comp_AVRTime(&AVRTimeNow, &AVRAlarm) == 0) ){
        AlarmOn = 15; // Sound Alarm (Beep 15 Times)
        AlarmSet = 0;
    }

    /* Sound the alarm */
    if(AlarmOn && AlarmOn--){
        INVERT_PIN(PIEZO_PORT,PIEZO_PIN);
    }

    /* Trigger an adc conversion to handle auto dimming */
    trigger_ADC(LDR_ADC_CHANNEL);
}


// 8-bit Timer2
ISR(TIMER2_COMPA_vect){ }

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
