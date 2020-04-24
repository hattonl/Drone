#include "my_delay.h"

// only use for 100Mhz STM32F411XE

void my_delay_ms(unsigned int t) {
    int i;
    for (i = 0; i < t; i++) {
        int a = 24538;  // at 100MHz 24538 is ok
        while (a--)
            ;
    }
}

void my_delay_us(unsigned int t) {
    int i;
    for (i = 0; i < t; i++) {
        int a = 20;  // at 168MHz 20 is ok,the higher the number the more timing
                     // precise
        while (a--)
            ;
    }
}
