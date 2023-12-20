/* AM1805 Sample code: external RTC module is used by host MCU */

#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <hardware/i2c.h>
#include <time.h>

#include "am1805.h"

#define I2C i2c0
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define AM1805_I2C_ADDRESS 0x69

/* Register Value */
#define AM1805_VALUE_ID0 0x18

#define AM1805_DEBUG 1

bool maskRegister(uint8_t regAddr, uint8_t andValue, uint8_t orValue) {
    bool bResult = false;

    uint8_t value;

    int bytes = am1805_register_read(regAddr, &value, 1);
    if (bytes == 1) {
        uint8_t newValue = (value & andValue) | orValue;
        
        if (newValue != value) {
            bytes = am1805_register_write(regAddr, newValue);
        }
    }

    return bResult;
}

bool isBitClear(uint8_t regAddr, uint8_t bitMask) {
    bool bResult;
    uint8_t value;

    bResult = am1805_register_read(regAddr, &value, 1);
    
    return bResult && ((value & bitMask) == 0);
}

bool isBitSet(uint8_t regAddr, uint8_t bitMask) {
    bool bResult;
    uint8_t value;

    bResult = am1805_register_read(regAddr, &value, 1);
    
    return bResult && ((value & bitMask) != 0);
}

bool clearRegisterBit(uint8_t regAddr, uint8_t bitMask) {
    return maskRegister(regAddr, ~bitMask, 0x00);
}

bool setRegisterBit(uint8_t regAddr, uint8_t bitMask) {
    return maskRegister(regAddr, 0xff, bitMask);
}

static uint8_t get_extension_address(uint8_t address)
{
    uint8_t xadd;
    uint8_t temp;

    am1805_register_read(REG_EXT_ADDR, &temp, 1);
    temp = temp & 0xC0;

    if (address < 64) { xadd = 0x8; }
    else if (address < 128) { xadd = 0x9; }
    else if (address < 192) { xadd = 0xA; }
    else { xadd = 0xB; }
    return (xadd | temp);
}

/* Set one or more bits in the selected register, selected by 1's in the mask */
static void setreg(uint8_t address, uint8_t mask)
{
    uint8_t temp;

    am1805_register_read(address, &temp, 1);
    temp |= mask;
    am1805_register_write(address, temp);
}

/* Clear one or more bits in the selected register, selected by 1's in the mask */
static void clrreg(uint8_t address, uint8_t mask)
{
    uint8_t temp;
    
    am1805_register_read(address, &temp, 1);
    temp &= ~mask;
    am1805_register_write(address, temp);
}

bool am1805_verify_product_id(void)
{
    uint8_t who_am_i[1];
    am1805_register_read(REG_ID0, &who_am_i[0], 1);    
#if AM1805_DEBUG    
    printf("ID:%x\r\n",who_am_i[0]);
#endif    
    if (who_am_i[0] != AM1805_VALUE_ID0) 
        return false;
    else 
        return true;
}

bool am1805_init(void)
{   
    bool transfer_succeeded = false;

    i2c_init(I2C, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));
       
    /* Read and verify product ID */
    transfer_succeeded = am1805_verify_product_id();

    return transfer_succeeded;
}

bool am1805_register_read(uint8_t register_address, uint8_t* destination, uint8_t number_of_bytes)
{
  int bytes = i2c_write_blocking(I2C, AM1805_I2C_ADDRESS, &register_address, 1, false);
  if (bytes){
    bytes = i2c_read_blocking(I2C, AM1805_I2C_ADDRESS, destination, number_of_bytes, false);
    if(bytes == number_of_bytes)
        return true;    
  }
  printf("read failed");
  return false;
}

bool am1805_register_write(uint8_t register_address, uint8_t value)
{   
    uint8_t tx_data[2];   
    tx_data[0] = register_address;
    tx_data[1] = value;
    int bytes = i2c_write_blocking(I2C, AM1805_I2C_ADDRESS, (uint8_t*) &tx_data, 2, false); 
    if(bytes == 2)
        return true; 
    else
        return false;         
}

bool am1805_register_writes(uint8_t regAddr, const uint8_t *array, size_t num ){
    int bytes = i2c_write_blocking(I2C, AM1805_I2C_ADDRESS, &regAddr, 1, false);
    if (bytes){
        bytes = i2c_write_blocking(I2C, AM1805_I2C_ADDRESS, array, num, false);   
        if (bytes == num)
            return true;
    }
    return false;
}

int am1805_burst_write(uint8_t *tx_data, uint8_t number_of_bytes)
{
    return i2c_write_blocking(I2C, AM1805_I2C_ADDRESS, tx_data, number_of_bytes, false);    
}

void am1805_config_input_interrupt(input_interrupt_t index_Interrupt)
{
    switch(index_Interrupt) {
        case XT1_INTERRUPT:
            /* Set input interrupt pin EX1T */
            clrreg(REG_STATUS,0x01);             // Clear EX1
            setreg(REG_INT_MASK,0x01);            // Set EX1E    
            break;
        case XT2_INTERRUPT:
            /* Set input interrupt pin WDI */
            clrreg(REG_STATUS,0x02);             // Clear EX2
            setreg(REG_INT_MASK,0x02);            // Set EX2E       
            break;
        default:
#if AM1805_DEBUG    
            printf("Wrong Input Interrupt Index\r\n");
#endif         
            break;        
    }
}


// Read a byte from local RAM
uint8_t am1805_read_ram(uint8_t address)
{
    uint8_t xadd;
    uint8_t temp;
    uint8_t reg_ram = 0;

    xadd = get_extension_address(address);                  // Calc XADDR value from address
    am1805_register_write(REG_EXT_ADDR, xadd);    // Load the XADDR register
    reg_ram = (address & 0x3F) | 0x40;                      // Read the data
    am1805_register_read(reg_ram, &temp, 1);
#if AM1805_DEBUG  
    printf("Read from addr:%x Data:%x\r\n",address,temp);
#endif    
    return (uint8_t)temp;                
}

// Write a byte to local RAM
void am1805_write_ram(uint8_t address, uint8_t data)
{
    uint8_t xadd;
    uint8_t reg_ram = 0;

    xadd = get_extension_address(address);                  // Calc XADDR value from address
    am1805_register_write(REG_EXT_ADDR, xadd);    // Load the XADDR register
    reg_ram = (address & 0x3F) | 0x40;  
    am1805_register_write(reg_ram, data);                   // Write the data
}

uint8_t valueToBcd(int value) {
    int tens = (value / 10) % 10;
    int ones = value % 10;

    return (uint8_t) ((tens << 4) | ones);
}

void tmToRegisters(const struct tm *timeptr, uint8_t *array, bool includeYear) {
    uint8_t *p = array;
    *p++ = valueToBcd(timeptr->tm_sec);
    *p++ = valueToBcd(timeptr->tm_min);
    *p++ = valueToBcd(timeptr->tm_hour);
    *p++ = valueToBcd(timeptr->tm_mday);
    *p++ = valueToBcd(timeptr->tm_mon + 1); // struct tm is 0-11, not 1-12!
    if (includeYear) {
        *p++ = valueToBcd(timeptr->tm_year % 100);
    }
    *p++ = valueToBcd(timeptr->tm_wday);
}

int watchdogSecs = 0;
unsigned long watchdogUpdatePeriod = 0;

bool resetConfig(uint32_t flags) {

    // Reset configuration registers to default values
    am1805_register_write(REG_STATUS, REG_STATUS_DEFAULT);
    am1805_register_write(REG_CTRL_1, REG_CTRL_1_DEFAULT);
    am1805_register_write(REG_CTRL_2, REG_CTRL_2_DEFAULT);
    am1805_register_write(REG_INT_MASK, REG_INT_MASK_DEFAULT);
    am1805_register_write(REG_SQW, REG_SQW_DEFAULT);
    am1805_register_write(REG_SLEEP_CTRL, REG_SLEEP_CTRL_DEFAULT);

    if ((flags & RESET_PRESERVE_REPEATING_TIMER) != 0) {
        maskRegister(REG_TIMER_CTRL, ~REG_TIMER_CTRL_RPT_MASK, REG_TIMER_CTRL_DEFAULT & ~REG_TIMER_CTRL_RPT_MASK);
    }  
    else {
        am1805_register_write(REG_TIMER_CTRL, REG_TIMER_CTRL_DEFAULT);
    }

    am1805_register_write(REG_TIMER, REG_TIMER_DEFAULT);
    am1805_register_write(REG_TIMER_INITIAL, REG_TIMER_INITIAL_DEFAULT);
    am1805_register_write(REG_WDT, REG_WDT_DEFAULT);

    uint8_t oscCtrl = REG_OSC_CTRL_DEFAULT;
    if ((flags & RESET_DISABLE_XT) != 0) {
        // If disabling XT oscillator, set OSEL to 1 (RC oscillator)
        // Also enable FOS so if the XT oscillator fails, it will switch to RC (just in case)
        // and ACAL to 0 (however REG_OSC_CTRL_DEFAULT already sets ACAL to 0)
        oscCtrl |= REG_OSC_CTRL_OSEL | REG_OSC_CTRL_FOS;
    }
    am1805_register_write(REG_OSC_CTRL, oscCtrl);
    am1805_register_write(REG_OSC_STATUS, 0x00);
    am1805_register_write(REG_TRICKLE, REG_TRICKLE_DEFAULT);
    am1805_register_write(REG_BREF_CTRL, REG_BREF_CTRL_DEFAULT);
    am1805_register_write(REG_AFCTRL, REG_AFCTRL_DEFAULT);
    am1805_register_write(REG_BATMODE_IO, REG_BATMODE_IO_DEFAULT);
    am1805_register_write(REG_OCTRL, REG_OCTRL_DEFAULT);

    return true;
}

#define REG_WDT_RESET_TYPE REG_WDT_RESET
//#define REG_WDT_RESET_TYPE REG_WDT_DEFAULT

bool setWDT(int seconds) {
    bool bResult = false;

    if (seconds < 0) {
        seconds = watchdogSecs;
    }

    if (seconds == 0) {
        // Disable WDT
        bResult = am1805_register_write(REG_WDT, 0x00);

        watchdogSecs = 0;
        watchdogUpdatePeriod = 0;
    } 
    else {
        printf("Writing WDT Register %d, %d, %d\n", REG_WDT_RESET_TYPE, (seconds << 2), REG_WDT_WRB_1_HZ );
        printf("Writing WDT Register ored %02x\n", REG_WDT_RESET_TYPE | (seconds << 2) | REG_WDT_WRB_1_HZ );

        bResult = am1805_register_write(REG_WDT, REG_WDT_RESET_TYPE | (seconds << 2) | REG_WDT_WRB_1_HZ ); 

        watchdogSecs = seconds;

        watchdogUpdatePeriod = (seconds * 2000);
    }

    return bResult;      
}

bool setRtcFromTm(const struct tm *timeptr) {
    uint8_t array[9];

    array[0] = 0x00; // REG_HUNDREDTH
    array[1] = 0x00; // hundredths
    tmToRegisters(timeptr, &array[2], true);

    // Can only write RTC registers when WRTC is 1
    setRegisterBit(REG_CTRL_1, REG_CTRL_1_WRTC);
    bool bResult = am1805_burst_write(array, sizeof(array));
    if (bResult) {
        // Clear the REG_CTRL_1_WRTC after setting the RTC.
        // Aside from being a good thing to do, that's how we know we've set it.
        clearRegisterBit(REG_CTRL_1, REG_CTRL_1_WRTC);
    }
    else {
        panic("Failed to set time %d\n", __LINE__);
    }
    return bResult;
}

/*
 * hundredth : 0 ~ 99
 * second : 0 ~ 59
 * minute : 0 ~ 59
 * weekday : 0 ~ 6
 * month : 1 ~ 12
 * year : 0 ~ 99
 * mode : 0 ~ 2
 */
void am1805_set_time(time_reg_struct_t time_regs)
{
    uint8_t temp;
    uint8_t temp_buff[9];

    /* 
     * Determine whether 12 or 24-hour timekeeping mode is being used
     * and set the 1224 bit appropriately
     */
    if (time_regs.mode == 2)        // 24-hour day
    {
        clrreg(REG_CTRL_1, 0x40);
    }
    else if (time_regs.mode == 1)   // 12-hour day PM
    {
        time_regs.hour |= 0x20;     // Set AM/PM
        setreg(REG_CTRL_1, 0x40);
    }
    else                            // 12-hour day AM
    {
        setreg(REG_CTRL_1, 0x40);
    }

    /* Set the WRTC bit to enable counter writes. */
    setreg(REG_CTRL_1, 0x01);

    /* Set the correct century */
    if (time_regs.century == 0)
    {
        clrreg(REG_STATUS, 0x80);
    }
    else
    {
        setreg(REG_STATUS, 0x80);
    }

    /* Write all of the time counters */
    temp_buff[0] = REG_HUNDREDTH;
    temp_buff[1] = time_regs.hundredth;
    temp_buff[2] = time_regs.second;
    temp_buff[3] = time_regs.minute;
    temp_buff[4] = time_regs.hour;
    temp_buff[5] = time_regs.date;
    temp_buff[6] = time_regs.month;
    temp_buff[7] = time_regs.year;
    temp_buff[8] = time_regs.weekday;

    /* Write the values to the AM18XX */
    am1805_burst_write(temp_buff, sizeof(temp_buff));

    /* Load the final value of the WRTC bit based on the value of protect */
    am1805_register_read(REG_CTRL_1, &temp, 1);
    temp &= 0x7E;                   // Clear the WRTC bit and the STOP bit
    //temp_buff[1] |= (0x01 & (~protect));    // Invert the protect bit and update WRTC
    am1805_register_write(REG_CTRL_1, temp);
       
    return;    
}

bool setRtcFromTime(time_t time) {
    struct tm *tm = gmtime(&time);
    return setRtcFromTm(tm);
}

void am1805_get_time(time_reg_struct_t *time_regs)
{
    uint8_t temp_buff[8];
    uint8_t time_mode;

    /* Read the counters. */
    am1805_register_read(REG_HUNDREDTH, temp_buff, 8);

    time_regs->hundredth = temp_buff[0];
    time_regs->second = temp_buff[1];
    time_regs->minute = temp_buff[2];
    time_regs->hour = temp_buff[3];
    time_regs->date = temp_buff[4];
    time_regs->month = temp_buff[5];
    time_regs->year = temp_buff[6];
    time_regs->weekday = temp_buff[7];

    /* Get the current hours format mode 12:24 */
    am1805_register_read(REG_CTRL_1, &time_mode, 1);
    if ((time_mode & 0x40) == 0)
    {
        /* 24-hour mode. */
        time_regs->mode = 2;
        time_regs->hour = time_regs->hour & 0x3F;           // Get tens:ones
    }
    else
    {
        /* 12-hour mode.  Get PM:AM. */
        time_regs->mode = (time_regs->hour & 0x20) ? 1 : 0;  // PM : AM
        time_regs->hour &= 0x1F;                            // Get tens:ones
    }

    time_regs->hundredth = temp_buff[0];
    time_regs->second = temp_buff[1];
    time_regs->minute = temp_buff[2];
    time_regs->hour = temp_buff[3];
    time_regs->date = temp_buff[4];
    time_regs->month = temp_buff[5];
    time_regs->year = temp_buff[6];
    time_regs->weekday = temp_buff[7];

#if AM1805_DEBUG 
    printf("hundredth:%x\r\n",time_regs->hundredth);
    printf("second:%x\r\n",time_regs->second);
    printf("minute:%x\r\n",time_regs->minute);
    printf("hour:%x\r\n",time_regs->hour);
    printf("date:%x\r\n",time_regs->date);
    printf("month:%x\r\n",time_regs->month);
    printf("year:%x\r\n",time_regs->year);
    printf("weekday:%x\r\n",time_regs->weekday);
#endif     
}

void am1805_config_alarm_from_tm(struct tm *t, alarm_repeat_t repeat, interrupt_mode_t intmode, interrupt_pin_t pin){

    time_reg_struct_t regs;

    regs.hundredth = 0x00;  // REG_HUNDREDTH
    regs.mode = 0x00;       // Default to 24 hours
    tmToRegisters(t, &regs.second, false);
    am1805_config_alarm(regs, repeat, intmode, pin);

}

void am1805_config_alarm(time_reg_struct_t time_regs, alarm_repeat_t repeat, interrupt_mode_t intmode, interrupt_pin_t pin)
{ 
    uint8_t temp;
    uint8_t temp_buff[8];

    /* Determine whether a 12-hour or a 24-hour time keeping mode is being used */
    if (time_regs.mode == 1)
    {
        /* A 12-hour day PM */
        time_regs.hour = time_regs.hour | 0x20;   // Set AM/PM
    }

    /* Write all of the time counters */
    temp_buff[0] = REG_HUNDREDTH_ALARM ;
    temp_buff[1] = time_regs.hundredth;
    temp_buff[2] = time_regs.second;
    temp_buff[3] = time_regs.minute;
    temp_buff[4] = time_regs.hour;
    temp_buff[5] = time_regs.date;
    temp_buff[6] = time_regs.month;
    temp_buff[7] = time_regs.weekday;

    clrreg(REG_TIMER_CTRL, 0x1C);      // Clear the RPT field
    clrreg(REG_INT_MASK, 0x64);       // Clear the AIE bit and IM field
    clrreg(REG_STATUS, 0x04);        // Clear the ALM flag

    if (pin == PIN_FOUT_nIRQ)
    {
        /* Interrupt on FOUT/nIRQ */
        am1805_register_read(REG_CTRL_2, &temp, 1);   // Get the Control2 Register       
        temp = (temp & 0x03);               // Extract the OUT1S field        
        if (temp != 0)                      // Not already selecting nIRQ
        {
            setreg(REG_CTRL_2, 0x03);    // Set OUT1S to 3           
        }        
    }
    if (pin == PIN_PSW_nIRQ2)
    {
        /* Interrupt on PSW/nIRQ2 */
        am1805_register_read(REG_CTRL_2, &temp, 1);   // Get the Control2 Register
        temp &= 0x1C;                       // Extract the OUT2S field
        if (temp != 0)                      // Not already selecting nIRQ
        {
            clrreg(REG_CTRL_2, 0x1C);    // Clear OUT2S
            setreg(REG_CTRL_2, 0x0C);    // Set OUT2S to 3
        }
    }

    if (repeat == ONCE_PER_10TH_SEC)
    {
        /* 10ths interrupt */
        temp_buff[1] |= 0xF0;
        repeat = ONCE_PER_SECOND;                   // Select correct RPT value
    }
    if (repeat == ONCE_PER_100TH_SEC)
    {
        /* 100ths interrupt */
        temp_buff[1] = 0xFF;
        repeat = ONCE_PER_SECOND;                   // Select correct RPT value
    }
    if (repeat != 0)                                // Don't initiate if repeat = 0
    {
        temp = (repeat << 2);                       // Set the RPT field to the value of repeat
        setreg(REG_TIMER_CTRL, temp);          // Was previously cleared
        setreg(REG_INT_MASK, (intmode << 5)); // Set the alarm interrupt mode
        setreg(REG_INT_MASK, 0x60); // Set the alarm interrupt mode
        am1805_burst_write(temp_buff, 8);           // Execute the burst write
        setreg(REG_INT_MASK, 0x04);           // Set the AIE bit        
    }
    else
        setreg(REG_INT_MASK, 0x60);           // Set IM field to 0x3 (reset value) to minimize current draw
    
    return;
}

void am1805_config_countdown_timer(count_down_range_t range, int32_t period, count_down_repeat_t repeat, interrupt_pin_t pin)
{
    uint8_t tm;
    uint8_t trpt;
    uint8_t tfs;
    uint8_t te;
    uint8_t temp;
    uint8_t tctrl;
    int32_t timer;
    uint8_t oscmode; 

    /* 0 = XT, 1 = RC */
    am1805_register_read(REG_OSC_STATUS, &oscmode, 1);   
    oscmode = (oscmode & 0x10) ? 1 : 0;
    
    /* disable count down timer */
    if (pin == INTERNAL_FLAG)
    {
        te = 0;
    }
    else
    {
        te = 1;
        if (repeat == SINGLE_LEVEL_INTERRUPT)
        {
            /* Level interrupt */
            tm = 1;                                     // Level
            trpt = 0;                                   // No repeat
            if (range == PERIOD_US)
            {
                /* Microseconds */
                if (oscmode == 0)
                {
                    /* XT Mode */
                    if (period <= 62500)                // Use 4K Hz
                    {
                        tfs = 0;
                        timer = (period * 4096);
                        timer = timer / 1000000;
                        timer = timer - 1;
                    }
                    else if (period <= 16384000)        // Use 64 Hz
                    {
                        tfs = 1;
                        timer = (period * 64);
                        timer /= 1000000;
                        timer = timer - 1;
                    }
                    else                                // Use 1 Hz
                    {
                        tfs = 2;
                        timer = period / 1000000;
                        timer = timer - 1;
                    }
                }
                else
                {
                    /* RC Mode */
                    if (period <= 2000000) {            // Use 128 Hz
                        tfs = 0;
                        timer = (period * 128);
                        timer /= 1000000;
                        timer = timer - 1;
                    }
                    else if (period <= 4000000) {       // Use 64 Hz
                        tfs = 1;
                        timer = (period * 64);
                        timer /= 1000000;
                        timer = timer - 1;
                    }
                    else {                              // Use 1 Hz
                        tfs = 2;
                        timer = period / 1000000;
                        timer = timer - 1;
                    }
                }
            }
            else
            {
                /* Seconds */
                if (period <= 256)
                {
                    /* Use 1 Hz */
                    tfs = 2;
                    timer = period - 1;
                }
                else
                {
                    /* Use 1/60 Hz */
                    tfs = 3;
                    timer = period / 60;
                    timer = timer - 1;
                }
            }
        }
        else
        {
            /* Pulse interrupts */
            tm = 0;                 // Pulse
            trpt = repeat & 0x01;   // Set up repeat
            if (repeat < REPEAT_PLUSE_1_128_SEC)
            {
                tfs = 0;
                if (oscmode == 0)
                {
                        timer = (period * 4096);
                        timer /= 1000000;
                        timer = timer - 1;
                }
                else
                {
                        timer = (period * 128);
                        timer /= 1000000;
                        timer = timer - 1;
                }
            }
            else if (repeat < REPEAT_PLUSE_1_64_SEC)
            {
                tfs = 1;
                timer = (period * 128);
                timer /= 1000000;
                timer = timer - 1;
            }
            else if (period <= 256)
            {
                /* Use 1 Hz */
                tfs = 2;
                timer = period - 1;
            }
            else
            {
                /* Use 1/60 Hz */
                tfs = 3;
                timer = period / 60;
                timer = timer - 1;
            }
        }
    }
    
    am1805_register_read(REG_TIMER_CTRL, &tctrl, 1);               // Get TCTRL, keep RPT, clear TE
    tctrl = tctrl & 0x1C;
    am1805_register_write(REG_TIMER_CTRL, tctrl);

    tctrl = tctrl | (te * 0x80) | (tm * 0x40) | (trpt * 0x20) | tfs;    // Merge the fields
    
    if (pin == PIN_FOUT_nIRQ)                                           // generate nTIRQ interrupt on FOUT/nIRQ (asserted low)
    {
         clrreg(REG_CTRL_2, 0x3);                                 // Clear OUT1S
    }
    if (pin == PIN_PSW_nIRQ2)                                           // generate nTIRQ interrupt on PSW/nIRQ2 (asserted low)
    {
         am1805_register_read(REG_CTRL_2, &temp, 1);              // Get OUT2S
         if ((temp & 0x1C) != 0)
         {
             temp = (temp & 0xE3) | 0x14;                               // If OUT2S != 0, set OUT2S to 5
         }
         am1805_register_write(REG_CTRL_2, temp);                 // Write back
    }
    if (pin != 0)
    {
        clrreg(REG_STATUS,0x08);                     // Clear TIM
        setreg(REG_INT_MASK,0x08);                    // Set TIE
        am1805_register_write(REG_TIMER, timer);     // Initialize the timer
        am1805_register_write(REG_TIMER_INITIAL, timer);   // Initialize the timer repeat
        am1805_register_write(REG_TIMER_CTRL, tctrl);  // Start the timer        
    }
    
    return ;    
}

/** Parameter:
 *  timeout - minimum timeout period in 7.8 ms periods (0 to 7)
 *  mode - sleep mode (nRST modes not available in AM08xx)
 *      0 => nRST is pulled low in sleep mode
 *      1 => PSW/nIRQ2 is pulled high on a sleep
 *      2 => nRST pulled low and PSW/nIRQ2 pulled high on sleep
 *  error ?returned value of the attempted sleep command
 *      0 => sleep request accepted, sleep mode will be initiated in timeout seconds
 *      1 => illegal input values
 *      2 => sleep request declined, interrupt is currently pending
 *      3 => sleep request declined, no sleep trigger interrupt enabled
**/
void am1805_set_sleep(uint8_t timeout, uint8_t mode)
{
    uint8_t slres = 0;
    uint8_t temp = 0;

#if AM1805_DEBUG    
    am1805_register_read(REG_CTRL_2, &temp, 1);       // Get SLST bit (temp & 0x08)
    
    if ( ( temp & 0x08 ) == 0)
    {
        printf("Previous Sleep Failed\r\n");
    } else {
        printf("Previous Sleep Successful\r\n");    
    }
    clrreg(REG_CTRL_2,0x08);                     // Clear SLST
    
    am1805_register_read(REG_CTRL_2, &temp, 1);       // Get SLST bit (temp & 0x08)
    
    if ( ( temp & 0x08 ) == 0)
    {
        printf("Clear Succ\r\n");
    } else {
        printf("Clear Fail\r\n");    
    }
    clrreg(REG_CTRL_2,0x08);                     // Clear SLST   
#endif
 
    if (mode > 0)
    {
        /* Sleep to PSW/nIRQ2 */
        am1805_register_read(REG_CTRL_2, &temp, 1);   // Read OUT2S
        temp = (temp & 0xE3) | 0x18;                        // MUST NOT WRITE OUT2S WITH 000
        am1805_register_write(REG_CTRL_2, temp);      // Write value to OUT2S
        slres = 0;
    } 
    
    temp = timeout | (slres << 6) | 0x80;                   // Assemble SLEEP register value
    am1805_register_write(REG_SLEEP_CTRL, temp);      // Write to the register    

#if AM1805_DEBUG
    /* Determine if SLEEP was accepted */
    am1805_register_read(REG_CTRL_2, &temp, 1);       // Get SLP bit (temp & 0x80)
    
    if ( ( temp & 0x80 ) == 0)
    {
        uint8_t reg_wdi_value = 0;
        /* SLEEP did not happen - determine why and return reason. */
        am1805_register_read(REG_INT_MASK, &temp, 1);         // Get status register interrupt enables
        am1805_register_read(REG_WDT, &reg_wdi_value, 1);    // Get WDT register
        if ((( temp & 0x0F ) == 0) & (((reg_wdi_value & 0x7C) == 0) || ((reg_wdi_value & 0x80) == 0x80)))
        {
            printf("No trigger interrupts enabled\r\n");
        }
        else
        {
            printf("Interrupt pending\r\n");
        }
    }
    else
    {
        printf("SLEEP request successful\r\n");
    }
#endif
}