#include <avr/io.h>

#define DAYS_IN_JAN       31
#define DAYS_IN_FEB       28  /* 29 on leap year 28 non leap year */
#define DAYS_IN_MAR       31
#define DAYS_IN_APR       30
#define DAYS_IN_MAY       31
#define DAYS_IN_JUN       30
#define DAYS_IN_JUL       31
#define DAYS_IN_AUG       31
#define DAYS_IN_SEP       30
#define DAYS_IN_OCT       31
#define DAYS_IN_NOV       30
#define DAYS_IN_DEC       31

#define AVR_YEAR(TP)    ((TP)->years)
#define AVR_DAY(TP)     ((TP)->days % 365)
#define AVR_HOUR(TP)    ((TP)->hours % 24)
#define AVR_MIN(TP)     ((TP)->minutes % 60)
#define AVR_SEC(TP)     ((TP)->seconds % 60)

/* Initial starting time 9:30am,1st January, 2012 */
#define AVR_DFL_YEAR 1991
#define AVR_DFL_DAY  169 
#define AVR_DFL_HOUR 0
#define AVR_DFL_MIN  0 
#define AVR_DFL_SEC  0

#define AVR_DEFAULT_TIME { \
    AVR_DFL_YEAR,          \
    AVR_DFL_DAY,           \
    AVR_DFL_HOUR,          \
    AVR_DFL_MIN,           \
    AVR_DFL_SEC            \
}

#define AVR_INIT_TIME(HOUR,MINUTE,SECOND) {    \
    AVR_DFL_YEAR,                              \
    AVR_DFL_DAY,                               \
    HOUR,                                      \
    MINUTE,                                    \
    SECOND                                     \
}

struct AVRTime_t {
    volatile uint16_t years;        // 0 <=> 65535
    volatile uint16_t days;         // -32768 <=> 32768 % 365
    volatile uint16_t hours;        // -32768 <=> 32768 % 24
    volatile uint16_t minutes;      // -32768 <=> 32768 % 60
    volatile uint16_t seconds;      // -32768 <=> 32768 % 60
};

/* Compare two AVRTime_t's 
 * Returns the difference e.g. a value less than zero is return if 
 * a - b < 0, 0 if a - b == 0 and a value greater than zero is returned
 * if a - b > 0
 */
int16_t comp_AVRTime(struct AVRTime_t* a, struct AVRTime_t* b);

/* Increment the seconds counter and overflow into minutes, hours, etc. */
void tick_AVRTime(struct AVRTime_t* stamp);

void set_AVRTime_time(struct AVRTime_t* stamp, uint8_t hours, uint8_t mins, uint8_t secs);
void set_AVRTime_date(struct AVRTime_t* stamp, uint16_t years, uint16_t days);

uint16_t get_AVRTime_day(uint8_t day_of_month, uint8_t month, uint16_t year);
uint8_t get_AVRTime_dayofmonth(int16_t bulk_days);
uint8_t get_AVRTime_month(int16_t bulk_day, uint16_t year); 

int8_t check_AVRTime_date(uint8_t day_of_month, uint8_t month, uint16_t year);
