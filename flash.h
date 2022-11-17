#ifndef __FLASH_H__
#define __FLASH_H__

#include "nrf_fstorage.h"
#include "timer.h"
#include "nrf_log.h"
#include "app_error.h"

#define FLASH_UPDATE_FLAG_NONE				0
#define FLASH_UPDATE_FLAG_VIBRATION_POWER		1
#define FLASH_UPDATE_FLAG_EXEC_TIME_ERASE		2
#define FLASH_UPDATE_FLAG_EXEC_ALON_WRITE		3
#define FLASH_UPDATE_FLAG_EXEC_BLE_TX                   4

union _UNION_TIME
{
	bmt_time_t union_time;
	uint32_t union_data[2];
};


void flash_init(void);
uint8_t get_vibration_power(void);
void test_vibration_power(uint32_t data);
bmt_time_t exec_time_print(uint16_t num);
void flash_operation(void);
uint16_t exec_time_count(void);
bmt_time_t exec_time_read(uint16_t num);
void flash_write(uint32_t start_page, uint32_t data);
void flash_read(uint32_t start_page, uint32_t* data);
bmt_time_t exec_time_packet_TX(uint16_t num);
void flash_clear();
void send_packet();


void exec_time_write_test();

extern uint8_t flash_flag;
extern uint32_t flash_update_flag;


#endif	// __FLASH_H__