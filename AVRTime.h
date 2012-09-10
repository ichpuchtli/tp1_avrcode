#include <avr/io.h>

#define AVR_YEAR(TP)    (TP->years)
#define AVR_DAY(TP)     (TP->days % 365)
#define AVR_HOUR(TP)    (TP->hours % 24)
#define AVR_MIN(TP)     (TP->minutes % 60)
#define AVR_SEC(TP)     (TP->seconds % 60)

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