#include <stdio.h>
#include "pico/stdlib.h"
#include "AM1805.h"

#define DEMO_INPUT_INTERRUPT 0
#define DEMO_COUNTDOWN_TIMER 1
#define DEMO_ALARM_FUNCTION 1

typedef enum
{
    DEFAULT_FLAG = 0xFF,  /**< Default State */
    INIT_FLAG = 0x01,     /**< Initialize Done */
    HOST_RUN_ONE = 0x02,  /**< Run to mode 1 */
    HOST_RUN_TWO = 0x04   /**< Run to mode 2 */
} demo_mode_t;

int test_am1805() {
    
    demo_mode_t demo_status = DEFAULT_FLAG;
    
    /* attempt to read the current status */
    sleep_ms(5);
    demo_status = (demo_mode_t)am1805_read_ram(0);        
    
    if ( demo_status == DEFAULT_FLAG )
    {
        /* Not config AM1805 yet */
        sleep_ms(1);

        if ( am1805_init() == false )
        {              
            panic("Failed to initialize AM1805\n");
        }

#if DEMO_INPUT_INTERRUPT
        am1805_config_input_interrupt(XT1_INTERRUPT);
#elif DEMO_COUNTDOWN_TIMER        
        am1805_config_countdown_timer( PERIOD_SEC, 10, REPEAT_PLUSE_1_64_SEC, PIN_nTIRQ);
#elif DEMO_ALARM_FUNCTION
     
        time_reg_struct_t time_regs;
    
        time_regs.hundredth = 0x00;
        time_regs.second = 0x08;
        time_regs.minute = 0x00;
        time_regs.hour = 0x00;
        time_regs.date = 0x00;
        time_regs.month = 0x00;
        time_regs.year = 0x00;
        time_regs.weekday = 0x00;
        time_regs.century = 0x00;
        time_regs.mode = 0x02;        
        /* Set specific alarm time */
        am1805_config_alarm(time_regs, ONCE_PER_MINUTE, PULSE_1_4_SEC, PIN_FOUT_nIRQ);
#endif
        sleep_us(200);
        am1805_write_ram(0,HOST_RUN_ONE);
        sleep_us(200);
        am1805_set_sleep(7,1);                         
    } else {
        switch(demo_status){
            case HOST_RUN_ONE:
                for(uint8_t i = 0;i < 5;i++)
                {
                    sleep_ms(1);
                }
                am1805_write_ram(0,HOST_RUN_TWO);
                sleep_ms(0.2);
                am1805_set_sleep(7,1); 
                break;
            case HOST_RUN_TWO:
                for(uint8_t i = 0;i < 5;i++)
                {
                   sleep_ms(1);   
                }
                am1805_write_ram(0,HOST_RUN_ONE);
                sleep_us(200);
                am1805_set_sleep(7,1);                              
                break;
            default:
                panic("Invalid test selected for AM1805\n");
                break;
        }
    }
        
    while (1) {
    }
}