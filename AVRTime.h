#include <avr/io.h>


struct AVRTime_t {
    volatile uint16_t years;        // 0 <=> 65535
    volatile int16_t months;       // -32768 <=> 65535 % 12
    volatile int16_t days;         // 0 <=> 65535 % 365
    volatile int16_t hours;        // 0 <=> 65535 % 24
    volatile int16_t minutes;      // 0 <=> 65535 % 60
    volatile int16_t seconds;      // 0 <=> 65535 % 60
};


struct AVRTime_t _ZERO_TIME_ = {0};

