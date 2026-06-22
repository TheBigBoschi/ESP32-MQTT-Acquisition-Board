#ifndef STATIC_DATA_H_
#define STATIC_DATA_H_

#include <stdio.h>
#include <string.h>
#include <syncronization.h>
#include <time.h>

struct {
time_t time;          
float PM2p5;          
float PM10p0;         
float temperature;    
float pressure;       
uint32_t payload_group;
uint32_t errors;       
uint16_t ambient_light;
uint16_t uvi;          
uint8_t humidity;     
float board_temperature;   
} typedef rtc_data;

void check_data();

rtc_data* get_data();

void store_data(sps30_task_param_t *sps30_task_param, ltr390_task_param_t *ltr390_task_param, sht40_task_param_t *sht40_task_param, bmp280_task_param_t *bmp280_task_param, EventBits_t error_mask, time_t time);


#endif