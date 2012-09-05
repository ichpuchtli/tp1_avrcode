#include "AVRTime.h" 

void init_AVRTime(struct AVRTime_t* stamp){


    stamp->years    = 0;
    stamp->months   = 0;
    stamp->days     = 0;
    stamp->hours    = 0;
    stamp->minutes  = 0;
    stamp->seconds  = 0;

}

void comp_AVRTime(struct AVRTime_t* a, struct AVRTime_t* b){
    
    struct AVRTime_t diff;


}

