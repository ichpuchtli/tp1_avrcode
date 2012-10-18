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

///////////////////////////// SOFTWARE INTERRUPTS ///////////////////////////// 
//// PCIE0 => PCINT0..7 PORTB
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
#define ACMP_PORT               ACSR
#define ACMP_PIN                ACO 

#define ACMP_WAITING            1
#define ACMP_JIFF_DIFF          1  
#define ACMP_FLASH_FREQ         40

///////////////////////////// LIGHT SENSOR ////////////////////////////////////
#define ADC_CHANNEL             2     // PORTC2
#define ADC_SAMPLES             4     // Number of samples to take before average
#define ADC_VREF                5000  // mV ADC Voltage Reference
#define DIM_LEVEL_MAX         16    // total dimmness levels 
#define DIM_START_LEVEL         2200  // mV Voltage for first  dimmness level
#define DIM_LEVEL_DIFF          ( (ADC_VREF - DIM_START_LEVEL) / DIM_LEVEL_MAX )

#define MAX_DELAY               2000 // useconds
#define DIM_DELAY_US               (MAX_DELAY/DIM_LEVEL_MAX)
///////////////////////////// PIEZO SPEAKER ///////////////////////////////////
#define PIEZO_PORT              PORTC
#define PIEZO_DDR               DDRC
#define PIEZO_PIN               1

///////////////////////////// IR COMMUNICATION ////////////////////////////////
#define IR_INCOMING_INT         SOFT_INT1_vect
#define IR_PORT                 PINC 
#define IR_DDR                  DDRC
#define IR_PIN                  0

#define IR_DEF_0                0x57
#define IR_DEF_1                0x49
#define IR_DEF_2                0x51
#define IR_DEF_3                0x41
#define IR_DEF_4                0x4D
#define IR_DEF_5                0x55
#define IR_DEF_6                0x45
#define IR_DEF_7                0x4B
#define IR_DEF_8                0x53
#define IR_DEF_9                0x43

#define IR_DEF_MENU             0x48
#define IR_DEF_ONOFF            0x47
#define IR_DEF_BUY              0x50
#define IR_DEF_FAV              0x4E
#define IR_DEF_AB               0x56
#define IR_DEF_CHUP             0x54
#define IR_DEF_CHDOWN           0x44
#define IR_DEF_VOLUP            0x5C
#define IR_DEF_VOLDOWN          0x4C
#define IR_DEF_MUTE             0x4F
#define IR_DEF_ENTER            0x46

///////////////////////////// WEATHER INFORMATION /////////////////////////////
#define SUNNY_SET_SIZE          12
#define RAINY_SET_SIZE          11
#define CLOUDY_SET_SIZE         9

/* Invert the state of a pin low->high or high->low */
#define INVERT_PIN(REG,PIN)    (REG ^= (1 << PIN))
/* Returns 1 or 0 if pin is High or Low */
#define PROBE_PIN(REG,PIN)     (REG & (1 << PIN)) 

#define HIGH 0xFF
#define LOW  0x00

enum DISPLAY_MODES { M_TIME_DISP, M_DATE_DISP, M_ALARM_DISP, M_WEATHER_DISP };
enum WEATHER_MODES { M_WEATH_SUNNY, M_WEATH_RAINY, M_WEATH_CLOUDY };

/* Initialize on board time */
static struct AVRTime_t AVRTime = AVR_INIT_TIME(21,39,0);
static struct AVRTime_t AVRAlarm = AVR_INIT_TIME(11,37,0);

static volatile uint8_t AlarmOn = 0x00;
static volatile uint8_t AlarmSet = 0x00;

static volatile uint8_t LEDSet[30] = {0x00};
static volatile uint8_t LEDSetSize = 0x00;
static volatile uint8_t LEDSetPos  = 0x00;

static volatile uint8_t CurrDispMode = M_TIME_DISP;

static volatile int16_t DimLevel = 0x00;
static volatile int16_t ADCSampleSum = 0x00;
static volatile int16_t ADCSamples = 0x00;
static volatile int16_t ADCAvgValue = 0x00;

static volatile uint16_t Jiffies = 0;

static volatile uint8_t WeatherForecast = M_WEATH_SUNNY;

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

static uint16_t mem_copy( uint8_t * dest, uint8_t * src, uint16_t size){

    for(uint16_t i = 0; i < size; i++)
        dest[i] = src[i];

    return size;
}

static void null_space(uint8_t* buffer, uint16_t size){

    while(size--) *(size + buffer) = 0x00;
}

static void insert_weather_set(uint8_t forecast){

    static uint8_t sunnySet[SUNNY_SET_SIZE] = { 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24};
    static uint8_t cloudySet[CLOUDY_SET_SIZE] = { 13, 14, 24, 15, 23, 25, 26, 27, 28};
    static uint8_t rainySet[RAINY_SET_SIZE] = {13, 14, 24, 15, 23, 25, 26, 27, 28, 29, 30};

    switch( forecast ) {

        case M_WEATH_RAINY:
            LEDSetSize = mem_copy( (void*) LEDSet, rainySet, RAINY_SET_SIZE);
            break;

        case M_WEATH_CLOUDY:
            LEDSetSize = mem_copy( (void*) LEDSet, cloudySet, CLOUDY_SET_SIZE);
            break;

        default : case M_WEATH_SUNNY:
            LEDSetSize = mem_copy( (void*) LEDSet, sunnySet, SUNNY_SET_SIZE);
            break;
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


static void insert_time_set(uint8_t hour, uint8_t min){

    uint8_t count = 0;

    count += insert_hour_set((uint8_t*) &LEDSet[count], hour);
    count += insert_minute_set((uint8_t*) &LEDSet[count], min);

    LEDSetSize = count;
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

void insert_date_set( uint8_t day, uint16_t year ){

    LEDSetSize = 0;
}

void process_num_stack(uint8_t* stack, uint8_t size){

    uint8_t hours, mins, secs;
    uint16_t days, years; 

    switch( CurrDispMode ){

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
        hours = stack[0] * 10 + stack[1];
        mins  = stack[2] * 10 + stack[3];
        secs  = stack[4] * 10 + stack[5];

        set_AVRTime_time(&AVRAlarm, hours, mins, secs);
        break;

    case M_WEATHER_DISP: 
        /* Select forecast mode from first number entered */
        WeatherForecast = stack[0];
        insert_weather_set(WeatherForecast);
        break;

    default: break;

    }
}

static void dispatch_command(uint8_t byte){
    
    int8_t num;

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

    if( num >= 0x00 ){
        stack[stackSize++] = num;
        return;
    }

    switch( byte ) {

        /* Change clock to display time */
        case IR_DEF_MENU:
            CurrDispMode = M_TIME_DISP;
            insert_time_set( AVR_HOUR(&AVRTime), AVR_MIN(&AVRTime) );
            break;

        /* Change clock to display date */
        case IR_DEF_BUY:
            CurrDispMode = M_DATE_DISP;
            insert_date_set( AVR_DAY(&AVRTime), AVR_YEAR(&AVRTime) );
            break;

        /* Change clock to display weather */
        case IR_DEF_AB:
            CurrDispMode = M_WEATHER_DISP;
            insert_weather_set(WeatherForecast);
            break;
 
        /* Change clock to display alarm */
        case IR_DEF_MUTE:
            CurrDispMode = M_ALARM_DISP;
            insert_time_set( AVR_HOUR(&AVRAlarm), AVR_MIN(&AVRAlarm) );
            break;

        /* Process the numbers collected depending on the current mode */
        case IR_DEF_ENTER:
            process_num_stack(stack,stackSize);
            /* Fallthrough */
            /* Reset number stack */
        case IR_DEF_ONOFF:
            null_space( stack, 10 );
            stackSize = 0x00;
            break;


        default : break;

    }

}
int main(void) {

    /* Set multiplexing outputs as low */
    MATRIX_COL_PORT &= 0xE0;
    MATRIX_ROW_PORT &= 0xC0;

    /* Set multiplexing pins as outputs */
    MATRIX_COL_DDR |= 0x1F;
    MATRIX_ROW_DDR |= 0x3F;

    init_timer0(); /* Multiplexer */

    /* Enable interrupts */
    sei();

    /* Splash */
    for ( uint8_t i = 1; i < 31; i++){
        LEDSet[LEDSetSize++] = i;
        _delay_ms(15);
    }
    _delay_ms(100); 
    for ( uint8_t i = 1; i < 31; i++) {
        LEDSetSize--;
        _delay_ms(15);
    }

    TCCR0B = (0<<CS02)|(1<<CS01)|(1<<CS00);

    init_ADC();
    init_comparator();
    init_PC_interrupts();

    /* Set speaker pin as an output and put the pin low */
    PIEZO_PORT &= ~(1 << PIEZO_PIN);
    PIEZO_DDR |= (1 << PIEZO_PIN);

    /* Set IR Pin as an input */
    IR_DDR &= ~(1 << IR_PIN);
   
    init_timer2(); /* Unused */

    _delay_ms(300);

    /* Fill the led set with the default time */
    insert_time_set( AVR_HOUR(&AVRTime), AVR_MIN(&AVRTime) );    

    init_timer1(); /* Seconds Timer/Counter */

    for ( ; ; ) asm("nop");

    return 0;
}


/* Software Interrupt 0 */
ISR(SOFT_INT0_vect){ }


/* ISR Triggered by IR_RECEIVER (PCINT8) || SOFT_INT1 */
ISR(IR_INCOMING_INT){

    uint8_t buf = 0x00;
    static uint8_t prevBuf = 0xFF;

    /* waste a few cycles here to sync with middle of level change */
    _delay_us(800);

    for( uint8_t bits = 0; bits < 8 ; bits++){

        buf |= PROBE_PIN(IR_PORT, IR_PIN) << (7 - bits);
        INVERT_PIN(PIEZO_PORT,PIEZO_PIN);
        _delay_us(1600);
    }
    
    if (prevBuf == buf) {
        /* TODO Consider Bottom Half Processing in main loop */
        dispatch_command( buf );
        prevBuf = 0xFF;
    } else {
        prevBuf = buf;
    }
    
    /* Clear interrupt flag to prevent recurring interrupts */
    PCIFR |= (1 << PCIF1);
}

/* Software Interrupt 2 */
ISR(SOFT_INT2_vect){

    //bits = (Jiffies - lastJiffy) / ( 50 / 8 );
    
    //buffer |= ( (bits + 1) * (flipCount & 0x01 ) ) << ( remain - bits );

    /*
     * TWO OPTIONS TO INTERCEPT PACKETS
     *
     * 1. Count the Jiffies between consecutive ACMP interrupts and work
     * out how many bits it has been high/low for, use soft interrupt so it can
     * triggered internally after 12ms timeout.
     *
     * 2. On first ACMP interrupt start counting 10 lots of Jiffies and poll
     * like IR. Screen flash frequency will have to be multiple of 4.
     *
     * 1.      |-40m-|-40m-|----80m----|
     *
     * 2.         |-40m-|-40m-|-40m-|-40m-|
     * ~~ -----+     +-----+           +---- ~~
     * ~~      |     |     |           |     ~~
     * ~~      |_____|     |___________|     ~~
     *
     *         ^ANALOG_COMP_vect
     *               ^ANALOG_COMP_vect
     */

} 

/* Optical Comm Trigger */
ISR(ANALOG_COMP_vect){ 
    
    //INVERT_PIN(PIEZO_PORT, PIEZO_PIN);
    
    FIRE_SOFT_INT2();

    ACSR |= 1 << ACI;
}

/* 8-bit Timer0 */
ISR(TIMER0_OVF_vect){

    /* Wrap LEDSetPos quicker than modulo */
    if(LEDSetPos >= LEDSetSize) LEDSetPos = 0x0;

    /* Rotate LEDs (Multiplexing) */
    select_LED( LEDSet [ LEDSetPos++ ] );

    /* Delay turning off the LED based on dimness */

    if ( DimLevel > 0) {

        int16_t delay = DIM_LEVEL_MAX - DimLevel;

        while(delay--) _delay_us(DIM_DELAY_US);

        select_LED( 0 );
    }
}

/* 16-bit Timer1, 1Hz */
ISR(TIMER1_COMPA_vect){ 

    /* Update the tick count */
    tick_AVRTime(&AVRTime);

    if ( AVR_SEC(&AVRTime) == 0) {

        switch ( CurrDispMode ) {

            case M_TIME_DISP:
                /* Update the LEDSet //if a new minute has ticked over */
                insert_time_set( AVR_HOUR(&AVRTime), AVR_MIN(&AVRTime) );
                break;
            case M_DATE_DISP:
                /* Update the LEDSet //if a new day has ticked over */
                insert_date_set( AVR_DAY(&AVRTime), AVR_YEAR(&AVRTime) );
                break;
            case M_ALARM_DISP:
                /* Update the LEDSet //if a new minute has ticked over */
                insert_time_set( AVR_HOUR(&AVRAlarm), AVR_MIN(&AVRAlarm) );
                break;
        }
    }
 
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
ISR(TIMER2_COMPA_vect){

    Jiffies++; // 250Hz counter only good for ~4 minutes

    // Incoming Packet Timeout
    if ( ACMP_WAITING && (ACMP_JIFF_DIFF >= 3) )
        FIRE_SOFT_INT2();

    trigger_ADC(ADC_CHANNEL);
} 

/* 10-bit ADC value left adjusted */
/* 8-bit precision with ADCH */
ISR(ADC_vect) {

    /* volts = 0mV...5000mV */
    uint16_t volts = ADCH * 20;
    
    ADCSampleSum += volts;

    if( ++ADCSamples >= ADC_SAMPLES ){
        
        ADCAvgValue = ADCSampleSum / ADCSamples;

        ADCSamples   = 0;
        ADCSampleSum = 0;
        DimLevel   = 0;

        if ( ADCAvgValue > DIM_START_LEVEL ) 
            DimLevel = (ADCAvgValue - DIM_START_LEVEL) / DIM_LEVEL_DIFF ;
    }
}
