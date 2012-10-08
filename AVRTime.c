#include "AVRTime.h" 

void init_AVRTime(struct AVRTime_t* stamp){

    stamp->years    = 0;
    stamp->days     = 0;
    stamp->hours    = 0;
    stamp->minutes  = 0;
    stamp->seconds  = 0;

}

int16_t comp_AVRTime(struct AVRTime_t* a, struct AVRTime_t* b){
    
    if( AVR_YEAR(a) < AVR_YEAR(b) )      return -1;
    if( AVR_YEAR(a) > AVR_YEAR(b) )      return 1;
   
    if( AVR_DAY(a) - AVR_DAY(b) )        return AVR_DAY(a) - AVR_DAY(b); 
    if( AVR_HOUR(a) - AVR_HOUR(b) )      return AVR_HOUR(a) - AVR_HOUR(b);
    if( AVR_MIN(a) - AVR_MIN(b) )        return AVR_MIN(a) - AVR_MIN(b);
    if( AVR_SEC(a) - AVR_SEC(b) )        return AVR_SEC(a) - AVR_SEC(a);
    
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

