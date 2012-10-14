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

///////////////////////////// MULTIPLEXING ////////////////////////////////////
#define MATRIX_COL_PORT         PORTD // PORTD0..4
#define MATRIX_ROW_PORT         PORTB // PORTB0..5
#define MATRIX_COL_DDR          DDRD // PORTD0..4
#define MATRIX_ROW_DDR          DDRB // PORTD0..5

#define MATRIX_COLS             0x06
#define MATRIX_ROWS             0x06

///////////////////////////// SOFTWARE INTERRUPTS ///////////////////////////// // PCIE0 => PCINT0..7 PORTB
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

#define FIRE_SOFT_INT0()        INVERT_PIN(SOFT_INT0_PORT, SOFT_INT0_PIN)
#define FIRE_SOFT_INT1()        INVERT_PIN(SOFT_INT1_PORT, SOFT_INT1_PIN)
#define FIRE_SOFT_INT2()        INVERT_PIN(SOFT_INT2_PORT, SOFT_INT1_PIN)

///////////////////////////// ANALOG COMPARATOR ///////////////////////////////
#define ANALOG_COMP_PORT        ACSR
#define ANALOG_COMP_PIN         ACO 

///////////////////////////// LIGHT SENSOR ////////////////////////////////////
#define LDR_ADC_CHANNEL         2    // PORTC2
#define LDR_ADC_LIMIT           4000 // mV Voltage threshold before dimming
#define LDR_ADC_SAMPLES         8    // Number of samples to take before average

///////////////////////////// PIEZO SPEAKER ///////////////////////////////////
#define PIEZO_PORT              PORTC
#define PIEZO_DDR               DDRC
#define PIEZO_PIN               1

///////////////////////////// IR COMMUNICATION ////////////////////////////////
#define IR_INCOMING_INT         SOFT_INT1_vect
#define IR_PORT                 PINC 
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

/* Invert the state of a pin low->high or high->low */
#define INVERT_PIN(REG,PIN)    (REG ^= (1 << PIN))
/* Returns 1 or 0 if pin is High or Low */
#define PROBE_PIN(REG,PIN)     (REG & (1 << PIN)) 

#define HIGH 0xFF
#define LOW  0x00

enum DISPLAY_MODES { M_TIME_DISP, M_DATE_DISP, M_ALARM_DISP, M_WEATHER_DISP };

/* Initialize on board time */
static struct AVRTime_t AVRTime = AVR_INIT_TIME(13,40,0);
static struct AVRTime_t AVRAlarm = AVR_INIT_TIME(13,41,0);

static volatile uint8_t AlarmOn = 0x00;
static volatile uint8_t AlarmSet = 0x00;

static volatile uint8_t LEDSet[8] = {0x00};
static volatile uint8_t LEDSetSize = 0x00;
static volatile uint8_t LEDSetPos  = 0x00;

static volatile uint8_t CurrDispMode = M_TIME_DISP;
static volatile uint8_t TotalDispModes = 0x04;

static volatile uint8_t DimmedMode = 0x00;
static volatile uint16_t ADCSampleSum = 0;
static volatile uint16_t ADCSamples = 0;
static volatile uint16_t ADCAvgValue = 0;

static void select_LED(uint8_t diode){

    /* Calculate new row x col combination before changing port values,
     * introduce volatile qualifier to prevent unwanted optimisation
     */
    volatile uint8_t row = ~( 1 << ( ( diode - 1) % MATRIX_ROWS ) );
    volatile uint8_t col = 1 << ( ( diode - 1) / MATRIX_COLS);

    if(diode == 0) col = 0;

    MATRIX_ROW_PORT = row;
    MATRIX_COL_PORT = col; 
}

static void null_space(uint8_t* buffer, uint16_t size){

    while(size--) *(size + buffer) = 0x00;
}

void process_num_stack(uint8_t* stack, uint8_t size){

    uint8_t hours,mins,secs;
    uint16_t days,years; 

    switch( CurrDispMode & (TotalDispModes - 1) ){

    case M_TIME_DISP:
        /* Set Time */
        hours = stack[0] * 10 + stack[1];
        mins  = stack[2] * 10 + stack[3];
        secs  = stack[4] * 10 + stack[5];

        set_AVRTime_time(&AVRTime, hours, mins, secs);
        break;

    case M_DATE_DISP:
        /* Set Date */
        years = stack[0] * 1000 + stack[1] * 100 + stack[2] * 10 + stack[3];
        days  = stack[4] * 100 + stack[5] * 10 + stack[6];

        set_AVRTime_date(&AVRTime, years, days); 
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
            CurrDispMode++;
            break; 

        /* Rotate Clock View Alarm, Date, Time, Weather */
        case IR_DEF_CHDOWN:
            CurrDispMode++;
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

static uint8_t insert_hour_set(uint8_t* set, uint8_t hour){

    uint8_t count = 0;

    set[count++] = (hour % 12) + 1;
    set[count++] = 29;

    if( hour >= 12 ) {
        set[count-1] = 30;
    }

    return count;
}

static uint8_t insert_minute_set(uint8_t* set, uint8_t minute){

    uint8_t count = 0;

    set[count++] = minute / 5 + 13;

    switch( minute % 5){

        case 4:
            set[count++] = 25;
        case 3:
            set[count++] = 26;
        case 2:
            set[count++] = 27; 
        case 1:
            set[count++] = 28;
        case 0:
        default:break;
    }

    return count;
}


static uint8_t insert_time_set(uint8_t hour, uint8_t min){

    uint8_t count = 0;

    count += insert_hour_set((uint8_t*) &LEDSet[count], hour);
    count += insert_minute_set((uint8_t*) &LEDSet[count], min);

    return count;
}

static uint8_t update_day_set(uint8_t* set, uint8_t day){

    uint8_t count = 0;

    set[count++] = 25;
    set[count++] = (day % 10) + 1;
    set[count++] = (day / 10) + 13;

    return count;
}

static uint8_t update_month_set(uint8_t* set, uint8_t month){

    uint8_t count = 0;

    set[count++] = 26;
    set[count++] = (month % 12) + 1;

    return count;
}

static uint8_t insert_year_set(uint8_t* set, uint16_t year){

    uint8_t count = 0;

    uint8_t start, end;    

    start = year / 1000 + 1;
    end   = start + ( ( year / 100 ) % 10 );

    for( uint8_t i = start; i <= end; i++){

        set[count++] = i % 12;
    }

    start = (year % 100)/10;
    end = start + year % 10;

    for( uint8_t i = start; i <= end; i++){

        set[count++] = i % 12 + 12;
    }

    return count;
}

int main(void) {

    init_timer0(); /* Multiplexer */
    init_timer1(); /* Seconds Timer/Counter */
    init_timer2(); /* Unused */

    init_ADC();
    init_comparator();
    init_PC_interrupts();

    /* Set multiplexing outputs as low */
    MATRIX_COL_PORT &= 0xE0;
    MATRIX_ROW_PORT &= 0xC0;

    /* Set multiplexing pins as outputs */
    MATRIX_COL_DDR |= 0x1F;
    MATRIX_ROW_DDR |= 0x3F;

    /* Set speaker pin as an output and put the pin low */
    PIEZO_PORT &= ~(1 << PIEZO_PIN);
    PIEZO_DDR |= (1 << PIEZO_PIN);

    /* Set IR Pin as an input */
    IR_DDR &= ~(1 << IR_PIN);

    /* Fill the led set with the default time */
    LEDSetSize = insert_time_set( AVR_HOUR(&AVRTime), AVR_MIN(&AVRTime) );

    /* Enable interrupts */
    sei();
    
    for ( ; ; ) asm("nop");

    return 0;
}

/* Software Interrupt 0 */
ISR(SOFT_INT0_vect){ }

/* Optical Comm Trigger */
ISR(ANALOG_COMP_vect){ 
    
    //INVERT_PIN(PIEZO_PORT, PIEZO_PIN);

    _delay_ms(25);

    ACSR |= 1 << ACI;

    /*
     * ~~ -----+     +-----+          +---- ~~
     * ~~      |     |     |          |     ~~
     * ~~      |_____|     |__________|     ~~
     *
     *         ^ANALOG_COMP_vect
     *                     ^ANALOG_COMP_vect
     */
}

/* ISR Triggered by IR_RECEIVER (PCINT8) || SOFT_INT1 */
ISR(IR_INCOMING_INT){

    uint8_t buf = 0x00;

    /* waste a few cycles here to sync with middle of level change */
    _delay_us(800);

    for( uint8_t bits = 0; bits < 8 ; bits++){

        buf |= PROBE_PIN(IR_PORT, IR_PIN) << bits;

        _delay_us(1600);
    }

    /* TODO Consider Bottom Half Processing in main loop */
    dispatch_command( buf );

    /* Clear interrupt flag to prevent recuring interrupts */
    PCIFR |= (1 << PCIF1);
}

/* Software Interrupt 2 */
ISR(SOFT_INT2_vect){ } 

/* 8-bit Timer0 */
ISR(TIMER0_OVF_vect){

    /* Wrap LEDSetPos quicker than modulo */
    if(LEDSetPos == LEDSetSize) LEDSetPos = 0x0;

    /* Rotate LEDs (Multiplexing) */
    select_LED( LEDSet [ LEDSetPos++ ] );

    /* Turn the LED off to reduce duty cycle */
    if ( DimmedMode ) select_LED( 0 );
}

/* 16-bit Timer1, 1Hz */
ISR(TIMER1_COMPA_vect){ 

    /* Update the tick count */
    tick_AVRTime(&AVRTime);

    /* Update the LEDSet if a new minute has tick over */
    if( AVR_SEC(&AVRTime) == 0 ) 
        LEDSetSize = insert_time_set( AVR_HOUR(&AVRTime), AVR_MIN(&AVRTime) );

    /* Compare the current time with the alarm time */ 
    if(AlarmSet && !comp_AVRTime(&AVRTime, &AVRAlarm)){
        AlarmOn = 8; /* Sound Alarm (Beep 4 Times) */
        AlarmSet = 0;
    }

    /* Sound the alarm */
    if(AlarmOn && AlarmOn--){
        INVERT_PIN(PIEZO_PORT,PIEZO_PIN);
    }
}

/* 8-bit Timer2 */
ISR(TIMER2_OVF_vect){

    trigger_ADC(LDR_ADC_CHANNEL);
} 

/* 10-bit ADC value left adjusted */
/* 8-bit precision with ADCH */
ISR(ADC_vect) {

    /* volts = 0mV...5000mV */
    uint16_t volts = ADCH * 20;
    
    ADCSampleSum += volts;

    if( ++ADCSamples == LDR_ADC_SAMPLES ){
        
        ADCAvgValue = ADCSampleSum / ADCSamples;

        ADCSamples = 0;
        ADCSampleSum = 0;

        DimmedMode = ( ADCAvgValue > LDR_ADC_LIMIT ) ? HIGH : LOW;
    }

}
