#include "AVRTime.h" 

#define __AVR_YEAR(TP)    ((int16_t) (TP)->years & 32767) 
#define __AVR_DAY(TP)     ((int16_t) ((TP)->days & 32767) % 365)
#define __AVR_HOUR(TP)    ((int16_t) ((TP)->hours & 32767) % 24)
#define __AVR_MIN(TP)     ((int16_t) ((TP)->minutes & 32767) % 60)
#define __AVR_SEC(TP)     ((int16_t) ((TP)->seconds & 32767) % 60)

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
