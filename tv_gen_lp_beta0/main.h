/*
 * main.h
 *
 *  Created on: 20 џэт. 2023 у.
 *      Author: AS
 */

#ifndef MAIN_H_
#define MAIN_H_
#include "main.h"
#include "stdint.h"
#define PWM_LED_RED GPIO3
#define CPU_FREQ 60000000

#define ADC_IHB_LS      1
#define ADC_VHB         2
#define ADC_TINT        3
#define ADC_IHB         4
#define ADC_VRES        6
#define ADC_VDC         8
#define ADC_IDC         9
#define ADC_PFC_TEMP    10
#define ADC_PFC_VIN     11
#define ADC_PFC_IIN     12

void  reprom_ID_read(void);
typedef struct
{
    uint64_t ID;
    uint32_t init_delay, init_cnt, init_cnt_1, ack_delay, ack_cnt, read_delay, read_cnt, start_read_delay, start_read_cnt, start_write_delay, start_write_cnt,start_write_cnt_1,start_write_cnt_2, write_delay, write_cnt, start_release_cnt, start_release_delay, init_release, read_window;
    uint16_t ID_read_flag;
    uint16_t init_ack;
    uint16_t family;
    uint16_t CRC;
    uint16_t time_scale;
    uint16_t state;
    uint16_t read_rom_counter;
    char id[7];
}DS2502R;


#endif /* MAIN_H_ */
