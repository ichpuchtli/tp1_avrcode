#include <avr/io.h>

#define AVR_YEAR(TP)    ((TP)->years)
#define AVR_DAY(TP)     ((TP)->days % 365)
#define AVR_HOUR(TP)    ((TP)->hours % 24)
#define AVR_MIN(TP)     ((TP)->minutes % 60)
#define AVR_SEC(TP)     ((TP)->seconds % 60)

/* Initial starting time 9:30am,1st January, 2012 */
#define AVR_DFL_YEAR (2012)
#define AVR_DFL_DAY  (1)
#define AVR_DFL_HOUR (0)
#define AVR_DFL_MIN  (0)
#define AVR_DFL_SEC  (0)

#define AVR_DEFAULT_TIME {AVR_DFL_YEAR,AVR_DFL_DAY,AVR_DFL_HOUR,AVR_DFL_MIN,AVR_DFL_SEC}

struct AVRTime_t {
    volatile uint16_t years;        // 0 <=> 65535
    volatile int16_t days;         // -32768 <=> 32768 % 365
    volatile int16_t hours;        // -32768 <=> 32768 % 24
    volatile int16_t minutes;      // -32768 <=> 32768 % 60
    volatile int16_t seconds;      // -32768 <=> 32768 % 60
};

/* Initialize an AVRTime_t struct */
void init_AVRTime(struct AVRTime_t* stamp);

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
