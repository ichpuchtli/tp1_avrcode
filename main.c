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
#define MATRIX_COL_PORT         PORTD /* PORTD0..4 */
#define MATRIX_ROW_PORT         PORTB /* PORTB0..5 */
#define MATRIX_COL_DDR          DDRD  /* PORTD0..4 */
#define MATRIX_ROW_DDR          DDRB  /* PORTD0..5 */

#define MATRIX_COLS             0x06
#define MATRIX_ROWS             0x06

/*/////////////////////////// SOFTWARE INTERRUPTS ///////////////////////////// */
//// PCIE0 => PCINT0..7 PORTB
#define SOFT_INT0_PORT          PINB
#define SOFT_INT0_PIN           6 /* (PCINT6) */
#define SOFT_INT0_vect          PCINT0_vect

/* Currently Used by IR Receiver to notify of incoming data*/
/* PCIE1 => PCINT8..14 PORTC*/
#define SOFT_INT1_PORT          PINC
#define SOFT_INT1_PIN           0 /* (PCINT8) */
#define SOFT_INT1_vect          PCINT1_vect

/* PCIE2 => PCINT016..23 PORTD*/
/* TODO Unable to use due to overlapping pin registrations */
#if 0
#define SOFT_INT2_PORT          PIND
#define SOFT_INT2_PIN           5  /* (PCINT21) */
#define SOFT_INT2_vect          PCINT2_vect
#endif

///////////////////////////// ANALOG COMPARATOR ///////////////////////////////
#define ACMP_PORT               ACSR
#define ACMP_PIN                ACO 

#define JIFFY_DIFF              8     /* mseconds period = 1000 / (F_CPU/256/OCR) */
#define ACMP_FLASH_DELAY        80    /* mseconds flash period */

#define ACMP_PKT_INBOUND        (!PROBE_PIN(ACSR,ACIE))
#define ACMP_ENABLE()           SET_PIN_HIGH(ACSR,ACIE)
#define ACMP_DISABLE()          SET_PIN_LOW(ACSR,ACIE)

#if 0
#define ACMP_PKT_INBOUND        (!Enabled)
#define ACMP_ENABLE()           (Enabled = 0) 
#define ACMP_DISABLE()          do {            \
    if( Enabled ) { Enabled = 0;}  \
    else {SET_PIN_HIGH(ACSR,ACI); return;} \
}while(0)
#endif

#define ACMP_PULSE_WIDTH        (ACMP_FLASH_DELAY/JIFFY_DIFF) /* jiffies */

///////////////////////////// LIGHT SENSOR ////////////////////////////////////
#define ADC_CHANNEL             2     /* PORTC2*/
#define ADC_SAMPLES             4     /* Number of samples to take before average*/
#define ADC_VREF                5000  /* mV ADC Voltage Reference*/
#define DIM_LEVEL_MAX           8    /* total dimmness levels */
#define DIM_START_LEVEL         3400  /* mV Voltage for first  dimmness level*/
#define DIM_LEVEL_DIFF          200   /* ((ADC_VREF - DIM_START_LEVEL) / DIM_LEVEL_MAX) */

#define MAX_DELAY               800 /* useconds*/
#define DIM_DELAY_US            (MAX_DELAY/DIM_LEVEL_MAX)

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

#define M_WEATH_SUNNY           0
#define M_WEATH_RAINY           1
#define M_WEATH_CLOUDY          2

///////////////////////////// DEBUG PINS //////////////////////////////////////
#define DEBUG0_PORT              PORTB
#define DEBUG0_DDR               DDRB
#define DEBUG0_PIN               5 

#define DEBUG1_PORT              PORTB
#define DEBUG1_DDR               DDRB
#define DEBUG1_PIN               4 

/* Invert the state of a pin low->high or high->low */
#define INVERT_PIN(REG,PIN)    (REG ^= (1 << PIN))
/* Returns 1 or 0 if pin is High or Low */
#define PROBE_PIN(REG,PIN)     (REG & (1 << PIN)) 

#define SET_PIN_HIGH(REG,PIN)  (REG |= (1 << PIN))
#define SET_PIN_LOW(REG,PIN)   (REG &= ~(1 << PIN))

enum DISPLAY_MODES { M_TIME_DISP, M_DATE_DISP, M_ALARM_DISP, M_WEATHER_DISP };

/* Initialize on board time */
static struct AVRTime_t AVRTime = AVR_INIT_TIME(0,0,0);
static struct AVRTime_t AVRAlarm = AVR_INIT_TIME(0,0,0);

static volatile uint8_t AlarmSet = 0x00;

static volatile uint8_t LEDSet[30] = {0x00};
static volatile uint8_t LEDSetSize = 0x00;

static volatile uint8_t CurrDispMode = M_TIME_DISP;
static volatile int16_t DimLevel = 0x00;

static volatile int16_t Jiffies = 0x0000; 
static volatile int16_t NextPollEvent = 0x0000;

static volatile uint8_t WeatherForecast = M_WEATH_SUNNY;

static volatile uint8_t FirstEdge = 0x01;

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

static void insert_day_set(uint8_t day){

    LEDSetSize = 0;

    LEDSet[LEDSetSize++] = 25;
    LEDSet[LEDSetSize++] = (day % 10) + 1;
    LEDSet[LEDSetSize++] = (day / 10) + 13;

}

static void insert_month_set(uint8_t month){

    LEDSetSize = 0;
    
    LEDSet[LEDSetSize++] = 26;
    LEDSet[LEDSetSize++] = (month % 12) + 1;
    LEDSet[LEDSetSize++] = (month / 10) + 13;
}

static void insert_year_set(uint16_t year){

    uint8_t start, end;    

    LEDSetSize = 0;

    LEDSet[LEDSetSize++] = 27;

    start = year / 1000 + 1;
    end   = start + ( ( year / 100 ) % 10 );

    for( uint8_t i = start; i <= end; i++)
        LEDSet[LEDSetSize++] = i % 12;

    start = (year % 100)/10;
    end = start + year % 10;

    for( uint8_t i = start; i <= end; i++) 
        LEDSet[LEDSetSize++] = i % 12 + 12;
}


void process_num_stack(uint8_t* stack, uint8_t size){

    uint8_t hours, mins, secs;
    uint16_t days, months, years;

    /* Switch on the current display mode to update required structures */
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
        days = stack[0] * 10 + stack[1];
        months = stack[2] * 10 + stack[3];
        years = stack[4] * 1000 + stack[5] * 100 + stack[6] * 10 + stack[7];

        //if ( !check_AVRTime_date(days, months, years) ) break;
        
        days = get_AVRTime_day(days, months, years);

        set_AVRTime_date(&AVRTime, days, years); 
        break;

    case M_ALARM_DISP:
        /* Set Alarm Time */
        hours = stack[0] * 10 + stack[1];
        mins  = stack[2] * 10 + stack[3];
        secs  = stack[4] * 10 + stack[5];

        set_AVRTime_time(&AVRAlarm, hours, mins, secs);
        AlarmSet = 0x01;
        break;

    case M_WEATHER_DISP: 
        /* Select forecast mode from first number entered */
        WeatherForecast = stack[0];
        break;

    default: break;

    }

    switch ( CurrDispMode ){

        case M_TIME_DISP:
            insert_time_set( AVR_HOUR(&AVRTime), AVR_MIN(&AVRTime) );
            break;

        case M_ALARM_DISP:
            insert_time_set( AVR_HOUR(&AVRAlarm), AVR_MIN(&AVRAlarm) );
            break;

        case M_DATE_DISP:
            insert_day_set( get_AVRTime_dayofmonth(AVR_DAY(&AVRTime)) );
            break;

        case M_WEATHER_DISP:
            insert_weather_set(WeatherForecast);
            break;

        default:break;
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
            insert_day_set( get_AVRTime_dayofmonth(AVR_DAY(&AVRTime)) );
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

void splash_screen(void){

    /* Splash */

    uint8_t diode;

    for ( diode = 1; diode < 31; diode++) LEDSet[diode-1] = diode;

    for ( diode = 1; diode < 31; diode++) _delay_ms(LEDSetSize++ * 0x00 + 15);

    _delay_ms(100); 

    for ( diode = 1; diode < 31; diode++) _delay_ms(LEDSetSize-- * 0x00 + 15);

    _delay_ms(300);
}

void init_ports(void){

    /* Set multiplexing outputs as low */
    MATRIX_COL_PORT &= 0xE0;
    MATRIX_ROW_PORT &= 0xC0;

    /* Set multiplexing pins as outputs */
    MATRIX_COL_DDR |= 0x1F;
    MATRIX_ROW_DDR |= 0x3F;

    //SET_PIN_HIGH(DEBUG0_DDR,DEBUG0_PIN);
    //SET_PIN_HIGH(DEBUG1_DDR,DEBUG1_PIN);

    /* Set speaker pin as an output and put the pin low */
    SET_PIN_LOW(PIEZO_PORT,PIEZO_PIN);
    SET_PIN_HIGH(PIEZO_DDR,PIEZO_PIN);

    /* Set IR Pin as an input */
    SET_PIN_LOW(IR_DDR,IR_PIN);
}

int main(void) {

    init_ports();

    init_timer0(); /* Multiplexer */
    init_timer1(); /* Seconds Timer/Counter */
    init_timer2(); /* JiffyTicker */

    sei(); /* Enable interrupts */

    splash_screen();

    TCCR0B = (0<<CS02)|(1<<CS01)|(1<<CS00);

    init_ADC();
    init_comparator();
    init_PC_interrupts();
   
    /* Fill the led set with the default time */
    insert_time_set( AVR_HOUR(&AVRTime), AVR_MIN(&AVRTime) );    

    for ( ; ; ) asm("nop");

    return 0;
}

void poll(void){

    /*
     * On first ACMP interrupt setup up jiffy delta to fire an interrupt to poll
     * like IR. Screen flash frequency will have to be multiple of 8.
     *
     *            |-40m-|-40m-|-40m-|-40m-|
     * ~~ -----+     +-----+           +---- ~~
     * ~~      |     |     |           |     ~~
     * ~~      |_____|     |___________|     ~~
     *
     *         ^ANALOG_COMP_vect
     */

    static uint8_t byte = 0x00;
    static int8_t bits = 0x08;

    bits--;

    if( bits >= 0 ){

        byte |= ( ((ACMP_PORT & (1 << ACMP_PIN)) >> ACMP_PIN) << bits );

        return;
    }

    /* Dispatch Command */
    dispatch_command( byte );

    bits = 0x08;
    byte = 0x00;

    /* Enable Analog Compare vector */
    //ACMP_ENABLE();
    FirstEdge = 1;
}

/* Software Interrupt 0 */
ISR(SOFT_INT0_vect){
#if 0
    /*
     * On first ACMP interrupt setup up jiffy delta to fire an interrupt to poll
     * like IR. Screen flash frequency will have to be multiple of 8.
     *
     *            |-40m-|-40m-|-40m-|-40m-|
     * ~~ -----+     +-----+           +---- ~~
     * ~~      |     |     |           |     ~~
     * ~~      |_____|     |___________|     ~~
     *
     *         ^ANALOG_COMP_vect
     */

    static uint8_t byte = 0x00;
    static int8_t bits = 0x08;

    bits--;

    if( bits >= 0 ){

        byte |= PROBE_PIN(ACMP_PORT,ACMP_PIN) << bits;

        return;
    }

    /* Dispatch Command */
    dispatch_command( byte );

    bits = 0x08;
    byte = 0x00;

    /* Enable Analog Compare vector */
    //ACMP_ENABLE();
    FirstEdge = 1;
#endif
}

/* ISR Triggered by IR_RECEIVER (PCINT8) || SOFT_INT1 */
ISR(IR_INCOMING_INT){

    static uint8_t prevbyte = 0xFF;

    uint8_t byte = 0x00;
    uint8_t bits = 8;

    /* waste a few cycles here to sync with middle of level change */
    _delay_us(800);

    while( bits-- ) {

        byte |= PROBE_PIN(IR_PORT, IR_PIN) << bits;

        _delay_us(1600);

        INVERT_PIN(PIEZO_PORT,PIEZO_PIN);
    }
    
    if (prevbyte == byte) { /* Second Pulse*/

        dispatch_command( byte );
        prevbyte = 0xFF;

    } else { /* First Pulse*/

        prevbyte = byte;
    }
    
    /* Clear interrupt flag to prevent recuring interrupts */
    SET_PIN_HIGH(PCIFR, PCIF1);
}

/* Optical Comm Trigger */
ISR(ANALOG_COMP_vect){ 
    
    //INVERT_PIN(DEBUG0_PORT,DEBUG0_PIN);
    
    _delay_ms(2);

    /* Disable Analog Compare vectors */
    //ACMP_DISABLE();

    if( FirstEdge ){
        
        FirstEdge = 0;

        /* Reset Jiffie Count */
        Jiffies = 0x0000;
        
        /* Queue next poll event in half a "pulse width" from now  */
        NextPollEvent = Jiffies + ACMP_PULSE_WIDTH/2;
    }

    /* Prevent Queued Interrupt */
    SET_PIN_HIGH(ACSR,ACI);
}

/* 8-bit Timer0,  */
ISR(TIMER0_COMPA_vect){

    static volatile uint8_t LEDSetPos  = 0x00;

    /* Wrap LEDSetPos quicker than modulo */
    if(LEDSetPos >= LEDSetSize) LEDSetPos = 0x0;

    /* Rotate LEDs (Multiplexing) */
    if(LEDSetSize > 0) select_LED( LEDSet [ LEDSetPos++ ] );

    /* Delay turning off the LED based on dimmness */
    if ( DimLevel > 0) {

        int16_t delay = DIM_LEVEL_MAX - DimLevel;

        while(delay--) _delay_us(DIM_DELAY_US);

        select_LED( 0 );
    }

}

/* 16-bit Timer1, 1Hz */
ISR(TIMER1_COMPA_vect){ 

    static volatile uint8_t AlarmOn = 0x00;

    /* Update the tick count */
    tick_AVRTime(&AVRTime);

    if ( AVR_SEC(&AVRTime) == 0) {

        switch ( CurrDispMode ) {

            case M_TIME_DISP:

                /* Update the LEDSet if a new minute has ticked over */
                insert_time_set( AVR_HOUR(&AVRTime), AVR_MIN(&AVRTime) );
                break;

            case M_ALARM_DISP:

                /* Update the LEDSet if a new minute has ticked over */
                insert_time_set( AVR_HOUR(&AVRAlarm), AVR_MIN(&AVRAlarm) );
                break;

            default:break;
        }
    }

    if (CurrDispMode == M_DATE_DISP) {

        switch( AVR_SEC(&AVRTime) % 12 ){

            case 0:
            case 1:
            case 2:
            case 3:
                insert_day_set( get_AVRTime_dayofmonth(AVR_DAY(&AVRTime)) );
                break;

            case 4:
            case 5:
            case 6:
            case 7:
                insert_month_set( get_AVRTime_month(AVR_DAY(&AVRTime),
                            AVR_YEAR(&AVRTime))  );
                break;

            case 8:
            case 9:
            case 10:
            case 11:
                insert_year_set( AVR_YEAR(&AVRTime) );
                break;
        }    

    }
 
    /* Compare the current time with the alarm time */ 
    if(AlarmSet && !comp_AVRTime(&AVRTime, &AVRAlarm)){
        AlarmOn = 8; /* Sound Alarm (Beep 4 Times) */
        AlarmSet = 0;
    }

    /* Sound the alarm */
    if(AlarmOn && AlarmOn--)
        INVERT_PIN(PIEZO_PORT,PIEZO_PIN);
}

/* 8-bit Timer2, 125Hz */
ISR(TIMER2_COMPA_vect){

    Jiffies++;

    /* Cap at +16383 to prevent overflow during addition and subtraction.*/
    //Jiffies &= ((1 << 14) - 1);

    if( FirstEdge == 0 ) {

        /* New Poll Event, Time to POLL Analog Comparator */
        if( (NextPollEvent - Jiffies) <= 0){

            /* Queue Software Interrupt 0, to handle polling */
            //INVERT_PIN(SOFT_INT0_PORT, SOFT_INT0_PIN);

            poll();

            /* Queue next poll event in one "pulse width" from now */
            NextPollEvent = Jiffies + ACMP_PULSE_WIDTH;
        }
    }

    trigger_ADC(ADC_CHANNEL);
} 

/* 10-bit ADC value left adjusted */
/* 8-bit precision with ADCH */
ISR(ADC_vect) {

    static int16_t ADCSampleSum = 0x00;
    static int16_t ADCSamples = 0x00;
    static int16_t ADCAvgValue = 0x00;

    /* volts = 0mV...5000mV */
    uint16_t volts = ADCH * 20;
    
    ADCSampleSum += volts;

    if( ++ADCSamples >= ADC_SAMPLES ){
        
        ADCAvgValue = ADCSampleSum / ADC_SAMPLES;

        ADCSamples   = 0;
        ADCSampleSum = 0;
        DimLevel     = 0;

        if ( ADCAvgValue > DIM_START_LEVEL ) 
            DimLevel = (ADCAvgValue - DIM_START_LEVEL) / DIM_LEVEL_DIFF;
    }
}
