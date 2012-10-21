#include "AVRTime.h" 

#define __AVR_YEAR(TP)    ((int16_t) (TP)->years & 32767) 
#define __AVR_DAY(TP)     ((int16_t) ((TP)->days & 32767) % 365)
#define __AVR_HOUR(TP)    ((int16_t) ((TP)->hours & 32767) % 24)
#define __AVR_MIN(TP)     ((int16_t) ((TP)->minutes & 32767) % 60)
#define __AVR_SEC(TP)     ((int16_t) ((TP)->seconds & 32767) % 60)


uint8_t MAX_DAYS[12] = {
   DAYS_IN_JAN,
   DAYS_IN_FEB,
   DAYS_IN_MAR,
   DAYS_IN_APR,
   DAYS_IN_MAY,
   DAYS_IN_JUN,
   DAYS_IN_JUL,
   DAYS_IN_AUG,
   DAYS_IN_SEP,
   DAYS_IN_OCT,
   DAYS_IN_NOV,
   DAYS_IN_DEC
};

void init_AVRTime(struct AVRTime_t* stamp){

    stamp->years    = 0;
    stamp->days     = 0;
    stamp->hours    = 0;
    stamp->minutes  = 0;
    stamp->seconds  = 0;

}

int16_t comp_AVRTime(struct AVRTime_t* a, struct AVRTime_t* b){
    
    if( __AVR_YEAR(a) - __AVR_YEAR(b) )   return __AVR_YEAR(a) - __AVR_YEAR(b); 
    if( __AVR_DAY(a)  - __AVR_DAY(b) )    return __AVR_DAY(a)  - __AVR_DAY(b); 
    if( __AVR_HOUR(a) - __AVR_HOUR(b) )   return __AVR_HOUR(a) - __AVR_HOUR(b);
    if( __AVR_MIN(a)  - __AVR_MIN(b) )    return __AVR_MIN(a)  - __AVR_MIN(b);
    if( __AVR_SEC(a)  - __AVR_SEC(b) )    return __AVR_SEC(a)  - __AVR_SEC(a);
    
    return 0;
}


void set_AVRTime_time(struct AVRTime_t* stamp, uint8_t hours, uint8_t mins, uint8_t secs){

    stamp->hours = hours;
    stamp->minutes = mins;
    stamp->seconds = secs;

}

void set_AVRTime_date(struct AVRTime_t* stamp, uint16_t years, uint16_t days){

    stamp->years = years; 
    stamp->days = days; 

}

void tick_AVRTime(struct AVRTime_t* stamp){

    if(AVR_SEC(stamp) == 59){

        if(AVR_MIN(stamp) == 59){

            if(AVR_HOUR(stamp) == 23){

                if(AVR_DAY(stamp) == 364){
                    stamp->years++;
                }

                stamp->days++;
            }

            stamp->hours++;
        }

        stamp->minutes++;
    }

    stamp->seconds++;
}

static void check_leap_year(uint16_t year){

    MAX_DAYS[1] = ( (year & 3) == 0 ) ? 29 : 28;
}

uint16_t get_AVRTime_day(uint8_t day_of_month, uint8_t month, uint16_t year){

    uint16_t days = 0;

    check_leap_year(year);
    
    while(month--) days += MAX_DAYS[month];

    return days + day_of_month;
}

uint8_t get_AVRTime_dayofmonth(int16_t bulk_days){

    uint8_t month = 0; 

    while( bulk_days > MAX_DAYS[month] ) 
        bulk_days -= MAX_DAYS[month++];

    return bulk_days;
}

uint8_t get_AVRTime_month(int16_t bulk_day, uint16_t year){

    uint8_t month = 0;

    check_leap_year(year);

    for ( ; ; ) {
    
        bulk_day -= MAX_DAYS[month++];

        if(bulk_day <= 0) break;
    }

    return month;
}

static int8_t valid_day(uint8_t day_of_month){
    
    return (day_of_month > 0) && (day_of_month <= 31);
}

static int8_t valid_month(uint8_t month){

    return (month > 0) && (month <= 12);
}

static int8_t valid_year(uint16_t year){

    return (year >= 0) && (year <= 9999);

}

int8_t check_AVRTime_date(uint8_t day_of_month, uint8_t month, uint16_t year){

    if( !valid_day(day_of_month) || !valid_month(month) || !valid_year(year) ) {
        return 0;
    }
    
    check_leap_year(year);

    return day_of_month <= MAX_DAYS[month];
}

