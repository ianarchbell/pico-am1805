/**
 * 
 * Put am1805 through its paces
 * 
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/sleep.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include "am1805.h"
#include "pico/util/datetime.h"
#include "time.h"

static char event_str[128];

static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

bool intFlag = false;

void gpio_callback(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    gpio_event_string(event_str, events);
    printf("GPIO %d %s\n", gpio, event_str);
    intFlag = true;
}

void dumpRegs(){
    uint8_t reg;
    for (int i=0;i<0x1D; i++){
         int bytes = am1805_register_read(i , &reg, 1);
         printf("Reg %02x,value: %02x\n", i , reg);
    }
}
  
int main() {
    // Enable UART so we can print status output
    stdio_init_all();
    sleep_ms(2000);

    printf("Testing am1805\n");
    am1805_init();
   
    bool isVerified = am1805_verify_product_id();
    if ( isVerified )
        printf("Device is AM1805\n");
    else
        printf("Device is not AM1805\n"); 

    gpio_init(22);
    gpio_set_dir(22, GPIO_IN);
    gpio_pull_up(22);

    gpio_set_irq_enabled_with_callback(22, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    uint8_t reg;

    // }
    int bytes;

    setWDT(0);
    setWDT(30);

    char datetime_buf[256];
    char *datetime_str = &datetime_buf[0];
 
    // Start on Friday 5th of June 2020 15:45:00
    datetime_t t = {
            .year  = 2023,
            .month = 12,
            .day   = 18,
            .dotw  = 1, // 0 is Sunday, so 5 is Friday
            .hour  = 11,
            .min   = 50,
            .sec   = 30
    };

    struct tm mydateTime;
 
    // Start the RTC
    rtc_init();
    rtc_set_datetime(&t);
    sleep_us(64);

    datetime_t dt;
    bool result = rtc_get_datetime(&dt);	
    datetime_to_str(datetime_str, sizeof(datetime_buf), &dt);
    printf("\r%s      \n", datetime_str);

    printf("Setting am1805 clock\n");
    mydateTime.tm_year = dt.year - 1900;
    mydateTime.tm_mon = dt.month - 1;
    mydateTime.tm_mday = dt.day;
    mydateTime.tm_hour = dt.hour;
    mydateTime.tm_min = dt.min;
    mydateTime.tm_sec = dt.sec;
    mydateTime.tm_wday = dt.dotw;


    setRtcFromTm(&mydateTime); 


    sleep_ms(5000);
    time_reg_struct_t rtcTime;
    am1805_get_time(&rtcTime);
    printf("Got am1805 clock\n");

    
    am1805_config_countdown_timer( PERIOD_SEC, 20, SINGLE_PLUSE_1_64_SEC,  PIN_FOUT_nIRQ);

    // setup regs
    struct tm alarm = mydateTime;
    printf("Month: %d\n", alarm.tm_mon);
    printf("Wday: %d\n", alarm.tm_wday);
    alarm.tm_min++;


    am1805_config_alarm_from_tm(&alarm, ONCE_PER_MINUTE, SINGLE_PLUSE_1_64_SEC, PIN_FOUT_nIRQ);

    dumpRegs();


    static uint32_t ms_from_boot;
    static uint32_t last_time;
    absolute_time_t timestamp = get_absolute_time();
    last_time =  to_ms_since_boot(timestamp);

    int i = 1;

    for(;;){

        absolute_time_t timestamp = get_absolute_time();
        ms_from_boot =  to_ms_since_boot(timestamp);
        if ((ms_from_boot - last_time)	== 1000){
            printf("Seconds: %d\n", i);
            i++;
            last_time = ms_from_boot;
        }
        if(intFlag){
            intFlag = false;
            am1805_get_time(&rtcTime);
        }
    }

    test_am1805();  
}

