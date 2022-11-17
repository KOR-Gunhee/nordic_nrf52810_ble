

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

#include "time.h"

#include "main.h"
#include "sensor.h"
#include "pwm.h"
#include "flash.h"
#include "timer.h"
#include "saadc.h"

APP_TIMER_DEF(m_timer_sensor_update_id);
#define TIMER_INTERVAL_SENSOR_UPDATE	APP_TIMER_TICKS(100) // 100 ms intervals

APP_TIMER_DEF(m_timer_haptic_id);
#define TIMER_INTERVAL_HAPTIC			APP_TIMER_TICKS(70)

APP_TIMER_DEF(m_timer_quad_id);
#define TIMER_INTERVAL_QUAD				APP_TIMER_TICKS(30000)	// 30 sec

APP_TIMER_DEF(m_timer_3min_id);
#define TIMER_INTERVAL_3MIN				APP_TIMER_TICKS(180000)	// 180 sec (3 min)

APP_TIMER_DEF(m_timer_1sec_id);
#define TIMER_INTERVAL_1SEC				APP_TIMER_TICKS(1000)	// 1 sec

APP_TIMER_DEF(m_timer_pwm_id);
#define TIMER_INTERVAL_PWM				APP_TIMER_TICKS(10)	// 10msec

APP_TIMER_DEF(m_timer_30sec_id);
#define TIMER_INTERVAL_30SEC				APP_TIMER_TICKS(30000)	// 30 sec

APP_TIMER_DEF(m_timer_exec_time_id);
#define TIMER_INTERVAL_EXEC_TIME		APP_TIMER_TICKS(1000)	// 1 sec

APP_TIMER_DEF(m_timer_i2c_id);
#define TIMER_INTERVAL_I2C		APP_TIMER_TICKS(2)	// 2 msec

//extern ble_nus_t m_nus;
extern void ble_send_data(uint8_t *buf, uint16_t len);
extern uint16_t m_conn_handle;
uint32_t duty_val=0;
uint32_t bmt_timer_sensor_cnt;
volatile uint32_t bmt_quad_cnt;

struct tm time_struct; 
struct tm m_tm_return_time; 
time_t init_time;
volatile uint32_t rtc_counter;

bmt_time_t local_time; 
bmt_time_t exec_time;
bmt_time_t save_time;
uint32_t exec_time_cnt;
uint16_t test_time_cnt=1;
uint32_t last_motor_power=0;
uint8_t PWM_Count=1;
uint8_t debug_flag=0;

void bmt_set_time(bmt_time_t m_time)
{
    local_time = m_time;
	
    rtc_counter = 0;
	
    time_struct.tm_year = m_time.year - 1900;
    time_struct.tm_mon = m_time.mon - 1;
    time_struct.tm_mday = m_time.day;
    time_struct.tm_hour = m_time.hour;
    time_struct.tm_min = m_time.min;
    time_struct.tm_sec = m_time.sec;   
    
    //NRF_LOG_INFO("TIME STRUCT : %d-%d-%d %d:%02d:%02d \r\n",
    //time_struct.tm_year, time_struct.tm_mon, time_struct.tm_mday, time_struct.tm_hour, time_struct.tm_min, time_struct.tm_sec);

    init_time = mktime(&time_struct);
}    

void print_time(bmt_time_t m_time)
{
        NRF_LOG_INFO("TIME : %d-%d-%d %d:%02d:%02d", m_time.year, 
        m_time.mon, m_time.day, m_time.hour, m_time.min, m_time.sec);
}

void timer_handler_1sec(void * p_context)
{
    time_t update_time;

    rtc_counter++;
	
    update_time = init_time + rtc_counter;
    m_tm_return_time = *localtime(&update_time);
    
    local_time.year = m_tm_return_time.tm_year + 1900;
    local_time.mon = m_tm_return_time.tm_mon + 1;
    local_time.day = m_tm_return_time.tm_mday;
    local_time.hour = m_tm_return_time.tm_hour;
    local_time.min = m_tm_return_time.tm_min;
    local_time.sec = m_tm_return_time.tm_sec;
}

void timer_init_1sec(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&m_timer_1sec_id, APP_TIMER_MODE_REPEATED, timer_handler_1sec);
	APP_ERROR_CHECK(err_code);
}

void timer_start_1sec(void)
{
	uint32_t err_code;
	
	
	err_code = app_timer_start(m_timer_1sec_id, TIMER_INTERVAL_1SEC, NULL);
	APP_ERROR_CHECK(err_code);
}

void timer_handler_sensor(void * p_context)
{
	uint8_t raw_acc_sen_data[6];
	uint8_t tx_packet[20] = {0,};
	uint16_t tx_len = 0;
	uint8_t idx = 0;
	
        int16_t ICM42605_DATA[7] = {0, 0, 0, 0, 0, 0, 0};

	static uint16_t num_of_exec_time;
	static uint16_t count_of_tx_exec_time;

	if(tx_enable_device_info == true) {
		tx_enable_device_info = false;
                
                sensor_val_update(ICM42605_DATA);

                bmt_sensor_accx = ICM42605_DATA[1];
                bmt_sensor_accy = ICM42605_DATA[2];
                bmt_sensor_accz = ICM42605_DATA[3];

                bmt_sensor_gyrox = ICM42605_DATA[4];
                bmt_sensor_gyroy = ICM42605_DATA[5];
                bmt_sensor_gyroz = ICM42605_DATA[6];

                NRF_LOG_INFO("Ax : %d", bmt_sensor_accx);
                NRF_LOG_INFO("Ay : %d", bmt_sensor_accy);
                NRF_LOG_INFO("Az : %d", bmt_sensor_accz);
                NRF_LOG_INFO("Gx : %d", bmt_sensor_gyrox);
                NRF_LOG_INFO("Gy : %d", bmt_sensor_gyroy);
                NRF_LOG_INFO("Gx : %d", bmt_sensor_gyroz);

                idx = 0;

		tx_packet[idx++] = 'd';						// '0x64' Header
		tx_packet[idx++] = BMT_FW_VER;					// FW Version
                tx_packet[idx++] = (uint8_t)(get_batt_volt());	// Battery Voltage
                tx_packet[idx++] = get_vibration_power();		// Vibration Power
		tx_packet[idx++] = get_sensor_id();			// Sensor ID
		tx_packet[idx++] = (uint8_t)((bmt_sensor_accx >> 8) & 0xFF);
		tx_packet[idx++] = (uint8_t)(bmt_sensor_accx & 0xFF);
                tx_packet[idx++] = (uint8_t)((bmt_sensor_accy >> 8) & 0xFF);
		tx_packet[idx++] = (uint8_t)(bmt_sensor_accy & 0xFF);
		tx_packet[idx++] = (uint8_t)((bmt_sensor_accz >> 8) & 0xFF);
		tx_packet[idx++] = (uint8_t)(bmt_sensor_accz & 0xFF);
                
                ///////////////bmt100 sensor disable////////////////
                tx_packet[idx++] = (uint8_t)((bmt_sensor_gyrox >> 8) & 0xFF);
		tx_packet[idx++] = (uint8_t)(bmt_sensor_gyrox & 0xFF);
                tx_packet[idx++] = (uint8_t)((bmt_sensor_gyroy >> 8) & 0xFF);
		tx_packet[idx++] = (uint8_t)(bmt_sensor_gyroy & 0xFF);
		tx_packet[idx++] = (uint8_t)((bmt_sensor_gyroz >> 8) & 0xFF);
		tx_packet[idx++] = (uint8_t)(bmt_sensor_gyroz & 0xFF);
                /////////////////////////////////////////////
               
		tx_packet[idx++] = nrf_gpio_pin_read(0) ? true : false;
		tx_packet[idx] = BMT_calc_checksum(tx_packet, idx);
		
		tx_len = ++idx;
                
                ble_send_data((uint8_t *)tx_packet, tx_len);
		return;
	}
	
	if(tx_enable_local_time == true) {
		tx_enable_local_time = false;
		
                idx = 0;

		tx_packet[idx++] = 'l';	// '0x6C' Header
		tx_packet[idx++] = (uint8_t)((local_time.year & 0xFF00) >> 8);
		tx_packet[idx++] = (uint8_t)(local_time.year & 0x00FF);
		tx_packet[idx++] = local_time.mon;
		tx_packet[idx++] = local_time.day;
		tx_packet[idx++] = local_time.hour;
		tx_packet[idx++] = local_time.min;
		tx_packet[idx++] = local_time.sec;
		tx_packet[idx] = BMT_calc_checksum(tx_packet, idx);
		
		tx_len = ++idx;

                NRF_LOG_INFO("TX LOCAL_TIME : %d-%d-%d %d:%02d:%02d \r\n",
                local_time.year, local_time.mon, local_time.day, local_time.hour, local_time.min, local_time.sec);
                
                ble_send_data((uint8_t *)tx_packet, tx_len);
                
		return;
	}
	
	if(tx_enable_ack_current_time == true) {
		tx_enable_ack_current_time = false;

		num_of_exec_time = exec_time_count();
		count_of_tx_exec_time = 0;

                idx = 0;

		tx_packet[idx++] = 'o';	// '0x6F' Header
		tx_packet[idx++] = (uint8_t)((num_of_exec_time & 0xFF00) >> 8);
		tx_packet[idx++] = (uint8_t)(num_of_exec_time & 0x00FF);
                
                ble_send_data((uint8_t *)tx_packet, tx_len);
		return;
	}
	
	if(tx_enable_exec_time == true) {
		tx_enable_exec_time = false;
		
                idx = 0;
		if(count_of_tx_exec_time >= num_of_exec_time) {
                        
                        tx_packet[idx++] = 'e';	// '0x65' Header
                        tx_packet[idx++] = 0;
                        tx_packet[idx] = BMT_calc_checksum(tx_packet, idx);
                        tx_len = ++idx;
                        ble_send_data((uint8_t *)tx_packet, tx_len);
                        
                        flash_update_flag = FLASH_UPDATE_FLAG_EXEC_TIME_ERASE;
                        NRF_LOG_INFO("Flash Memory empty");
			return;
		}
		
		bmt_time_t temp_time = exec_time_read(count_of_tx_exec_time++);
                                
		tx_packet[idx++] = 'e';	// '0x65' Header
		tx_packet[idx++] = (uint8_t)((save_time.year & 0xFF00) >> 8);
		tx_packet[idx++] = (uint8_t)(save_time.year & 0x00FF);
		tx_packet[idx++] = save_time.mon;
		tx_packet[idx++] = save_time.day;
		tx_packet[idx++] = save_time.hour;
		tx_packet[idx++] = save_time.min;
		tx_packet[idx++] = save_time.sec;
		tx_packet[idx++] = save_time.dur;
		tx_packet[idx] = BMT_calc_checksum(tx_packet, idx);
		
                
                NRF_LOG_INFO("TX LOCAL_TIME : %d-%d-%d %d:%02d:%02d",
                save_time.year, save_time.mon, save_time.day, save_time.hour, save_time.min, save_time.sec);
                NRF_LOG_INFO("%d",save_time.dur);
		
		tx_len = ++idx;
                 
                ble_send_data((uint8_t *)tx_packet, tx_len);
                NRF_LOG_INFO("Flash Memory TX success");
		return;
	}
	
	if(tx_enable_exec_time_again == true) {
		tx_enable_exec_time_again = false;
		
		bmt_time_t temp_time = exec_time_read(--count_of_tx_exec_time);
		count_of_tx_exec_time++;

                idx = 0;

		tx_packet[idx++] = 'e';	// '0x65' Header
		tx_packet[idx++] = (uint8_t)((save_time.year & 0xFF00) >> 8);
		tx_packet[idx++] = (uint8_t)(save_time.year & 0x00FF);
		tx_packet[idx++] = save_time.mon;
		tx_packet[idx++] = save_time.day;
		tx_packet[idx++] = save_time.hour;
		tx_packet[idx++] = save_time.min;
		tx_packet[idx++] = save_time.sec;
		tx_packet[idx++] = save_time.dur;
		tx_packet[idx] = BMT_calc_checksum(tx_packet, idx);
		
		tx_len = ++idx;
		
                ble_send_data((uint8_t *)tx_packet, tx_len);
		return;
	}
        
        if(tx_enable_exec_time_call == true) {
                tx_enable_exec_time_call = false;
                
                idx = 0;

		tx_packet[idx++] = 'e';	// '0x65' Header
		tx_packet[idx++] = (uint8_t)((local_time.year & 0xFF00) >> 8);
		tx_packet[idx++] = (uint8_t)(local_time.year & 0x00FF);
		tx_packet[idx++] = local_time.mon;
		tx_packet[idx++] = local_time.day;
		tx_packet[idx++] = local_time.hour;
		tx_packet[idx++] = local_time.min;
		tx_packet[idx++] = local_time.sec;
		tx_packet[idx++] = get_exec_time_cnt();
		tx_packet[idx] = BMT_calc_checksum(tx_packet, idx);
		
		tx_len = ++idx;
		
                ble_send_data((uint8_t *)tx_packet, tx_len);

		return;
        }
        
        if(tx_enable_sensor_info == true){

            sensor_val_update(ICM42605_DATA);

            bmt_sensor_accx = ICM42605_DATA[1];
            bmt_sensor_accy = ICM42605_DATA[2];
            bmt_sensor_accz = ICM42605_DATA[3];

            bmt_sensor_gyrox = ICM42605_DATA[4];
            bmt_sensor_gyroy = ICM42605_DATA[5];
            bmt_sensor_gyroz = ICM42605_DATA[6];
            
            if (debug_flag==1)
            {
                NRF_LOG_INFO("Ax : %d || Ay : %d || Az : %d || Gx : %d || Gy : %d || Gz : %d", bmt_sensor_accx, bmt_sensor_accy, bmt_sensor_accz, bmt_sensor_gyrox, bmt_sensor_gyroy, bmt_sensor_gyroz);
            }
            
            idx = 0;

            tx_packet[idx++] = 's';	// 0x73
            tx_packet[idx++] = (uint8_t)((bmt_sensor_accx >> 8) & 0xFF);
            tx_packet[idx++] = (uint8_t)(bmt_sensor_accx & 0xFF);
            tx_packet[idx++] = (uint8_t)((bmt_sensor_accy >> 8) & 0xFF);
            tx_packet[idx++] = (uint8_t)(bmt_sensor_accy & 0xFF);
            tx_packet[idx++] = (uint8_t)((bmt_sensor_accz >> 8) & 0xFF);
            tx_packet[idx++] = (uint8_t)(bmt_sensor_accz & 0xFF);
            tx_packet[idx++] = (uint8_t)((bmt_sensor_gyrox >> 8) & 0xFF);
            tx_packet[idx++] = (uint8_t)(bmt_sensor_gyrox & 0xFF);
            tx_packet[idx++] = (uint8_t)((bmt_sensor_gyroy >> 8) & 0xFF);
            tx_packet[idx++] = (uint8_t)(bmt_sensor_gyroy & 0xFF);
            tx_packet[idx++] = (uint8_t)((bmt_sensor_gyroz >> 8) & 0xFF);
            tx_packet[idx++] = (uint8_t)(bmt_sensor_gyroz & 0xFF);
            tx_packet[idx++] = key_state;
            tx_packet[idx++] = BMT_calc_checksum(tx_packet, idx);
            
            tx_len = ++idx;
          
            if(++bmt_timer_sensor_cnt > 20) {
                    bmt_timer_sensor_cnt = 0;
            }
	
            if(key_state) {
                key_state = false;
            }

            ble_send_data((uint8_t *)tx_packet, tx_len);
        }
}
        
void timer_init_sensor(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&m_timer_sensor_update_id, APP_TIMER_MODE_REPEATED, timer_handler_sensor);
	APP_ERROR_CHECK(err_code);
}

void timer_start_sensor(void)
{
	uint32_t err_code;
	
	err_code = app_timer_start(m_timer_sensor_update_id, TIMER_INTERVAL_SENSOR_UPDATE, NULL);
	APP_ERROR_CHECK(err_code);
}

void timer_stop_sensor(void)
{
	app_timer_stop(m_timer_sensor_update_id);
}

void timer_handler_3min(void * p_context)
{
	uint32_t err_code;
	
	motor_stop();
	
	if(m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        sd_nvic_SystemReset();
	}
}
      
void timer_init_3min(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&m_timer_3min_id, APP_TIMER_MODE_SINGLE_SHOT, timer_handler_3min);
	APP_ERROR_CHECK(err_code);
}

void timer_start_3min(void)
{
	app_timer_start(m_timer_3min_id, TIMER_INTERVAL_3MIN, NULL);
}

void timer_stop_3min(void)
{
	app_timer_stop(m_timer_3min_id);
}

void timer_handler_haptic(void * p_context)
{
	motor_stop();
}
      
void timer_init_haptic(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&m_timer_haptic_id, APP_TIMER_MODE_SINGLE_SHOT, timer_handler_haptic);
	APP_ERROR_CHECK(err_code);
}

void timer_start_haptic(void)
{
	uint32_t err_code;
	
	err_code = app_timer_start(m_timer_haptic_id, TIMER_INTERVAL_HAPTIC, NULL);
	APP_ERROR_CHECK(err_code);
}

void timer_handler_quad(void * p_context)
{        
    if(++bmt_quad_cnt < 4) {
            for(uint8_t count=0;count<3;count++)
            {
              new_motor_action(HAPTIC_VIBRATION_POWER1);
              nrf_delay_ms(HAPTIC_DELAY);
              new_motor_action(HAPTIC_VIBRATION_POWER2);
              nrf_delay_ms(HAPTIC_DELAY);
              
            }
            new_motor_action(vibration_power[alone_power_index]);
    }
    else{
            exec_time = local_time;
            exec_time.dur = 120;
            
            timer_stop_exec_time();
            motor_stop();

            flash_update_flag = FLASH_UPDATE_FLAG_EXEC_ALON_WRITE;  
    }
}

void timer_init_quad(void)
{
	uint32_t err_code;

	err_code = app_timer_create(&m_timer_quad_id, APP_TIMER_MODE_REPEATED, timer_handler_quad);
	APP_ERROR_CHECK(err_code);
}

void timer_start_quad(void)
{
	uint32_t err_code;
	
        bmt_quad_cnt = 0;
	
	err_code = app_timer_start(m_timer_quad_id, TIMER_INTERVAL_QUAD, NULL);
	APP_ERROR_CHECK(err_code);
}

void timer_stop_quad(uint8_t flag)
{
        NRF_LOG_INFO("flag : %d", flag);
        if(flag==1)
        {
          exec_time = local_time;
          exec_time.dur = get_exec_time_cnt();
          
          NRF_LOG_INFO("exec_time.dur : %d", exec_time.dur);
          if (exec_time.dur>=30){
            flash_update_flag = FLASH_UPDATE_FLAG_EXEC_ALON_WRITE;}
          
          else{
             sd_nvic_SystemReset(); 
            }
        }
         
        bmt_quad_cnt = 0;
        
        app_timer_stop(m_timer_quad_id);
}

void timer_handler_exec_time(void * p_context)
{
	exec_time_cnt++;
        
        //if(motor_state == true)
        //{
        //   timer_stop_pwm_time();
        //}
}

void timer_init_exec_time(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&m_timer_exec_time_id, APP_TIMER_MODE_REPEATED, timer_handler_exec_time);
	APP_ERROR_CHECK(err_code);
}

void timer_start_exec_time(void)
{
	uint32_t err_code;
	
	exec_time_cnt = 0;
	err_code = app_timer_start(m_timer_exec_time_id, TIMER_INTERVAL_EXEC_TIME, NULL);
	APP_ERROR_CHECK(err_code);
}

void timer_stop_exec_time(void)
{
	app_timer_stop(m_timer_exec_time_id);
}

uint32_t get_exec_time_cnt(void)
{
	return exec_time_cnt;
}

void timer_handler_pwm_time(void * p_context)
{
    uint32_t val00;
   
    if(motor_state)
    {
        if (duty_val>BASE_VIBRATION_POWER)
        {
            if (PWM_Count<2)
            {
               val00=ceil(1000-((1000-duty_val)*PWM_Count*(0.5)));
               new_motor_action(val00);
               PWM_Count++;
            }
            else 
            {
               PWM_Count=1;
               new_motor_action(duty_val);
               timer_stop_pwm_time();
            }
        }
        else
        {
           if (PWM_Count<10)
            {
               val00=ceil(1000-((1000-duty_val)*PWM_Count*(0.1)));
               new_motor_action(val00);
               PWM_Count++;
               
               NRF_LOG_INFO("-----------------------");   
               NRF_LOG_INFO("PWM count : %d", PWM_Count);
               NRF_LOG_INFO("PWM(2) : %d", val00);
               NRF_LOG_INFO("-----------------------"); 
            }
            else 
            {
               PWM_Count=1;
               new_motor_action(duty_val);
               timer_stop_pwm_time();
               
               NRF_LOG_INFO("-----------------------");   
               NRF_LOG_INFO("PWM(3) : %d", duty_val);
               NRF_LOG_INFO("-----------------------"); 
            }
        }
    }
    else
    {
        timer_stop_pwm_time();
    }
}

void timer_init_pwm_time(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&m_timer_pwm_id, APP_TIMER_MODE_REPEATED, timer_handler_pwm_time);
	APP_ERROR_CHECK(err_code);
}    

void timer_start_pwm_time(void)
{
	uint32_t err_code;

	err_code = app_timer_start(m_timer_pwm_id, TIMER_INTERVAL_PWM, NULL);
	APP_ERROR_CHECK(err_code);
}

void timer_stop_pwm_time(void)
{
        if(motor_state==0)
        {
            new_motor_disable();
            new_motor_action(1000);
        }
        
        app_timer_stop(m_timer_pwm_id);
}

void timer_handler_30sec(void * p_context)
{
  	uint32_t err_code;
        
	if(m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        sd_nvic_SystemReset();
	}
}

void timer_init_30sec(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&m_timer_30sec_id, APP_TIMER_MODE_REPEATED, timer_handler_30sec);
	APP_ERROR_CHECK(err_code);
}
      
void timer_start_30sec(void)
{
	uint32_t err_code;
	 
	err_code = app_timer_start(m_timer_30sec_id, TIMER_INTERVAL_30SEC, NULL);
	APP_ERROR_CHECK(err_code);
}
      
void timer_stop_30sec(void)
{
        app_timer_stop(m_timer_30sec_id);
}
      
void timer_handler_i2c_time(void * p_context)
{
        int16_t ICM42605_DATA[7] = {0, 0, 0, 0, 0, 0, 0};
    
        sensor_val_update(ICM42605_DATA);
        
        bmt_sensor_accx = (((bmt_sensor_accx<<3) + (bmt_sensor_accx<<2) + (bmt_sensor_accx<<1) + bmt_sensor_accx + ICM42605_DATA[1]) >> 4);
        bmt_sensor_accy = (((bmt_sensor_accy<<3) + (bmt_sensor_accy<<2) + (bmt_sensor_accy<<1) + bmt_sensor_accy + ICM42605_DATA[2]) >> 4);
        bmt_sensor_accz = (((bmt_sensor_accz<<3) + (bmt_sensor_accz<<2) + (bmt_sensor_accz<<1) + bmt_sensor_accz + ICM42605_DATA[3]) >> 4);
        bmt_sensor_gyrox = (((bmt_sensor_gyrox<<3) + (bmt_sensor_gyrox<<2) + (bmt_sensor_gyrox<<1) + bmt_sensor_gyrox + ICM42605_DATA[4]) >> 4);
        bmt_sensor_gyroy = (((bmt_sensor_gyroy<<3) + (bmt_sensor_gyroy<<2) + (bmt_sensor_gyroy<<1) + bmt_sensor_gyroy + ICM42605_DATA[5]) >> 4);
        bmt_sensor_gyroz = (((bmt_sensor_gyroz<<3) + (bmt_sensor_gyroz<<2) + (bmt_sensor_gyroz<<1) + bmt_sensor_gyroz + ICM42605_DATA[6]) >> 4);
}

void timer_init_i2c_time(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&m_timer_i2c_id, APP_TIMER_MODE_REPEATED, timer_handler_i2c_time);
	APP_ERROR_CHECK(err_code);
}
      
void timer_start_i2c_time(void)
{
	uint32_t err_code;
	 
	err_code = app_timer_start(m_timer_i2c_id, TIMER_INTERVAL_I2C, NULL);
	APP_ERROR_CHECK(err_code);
}
      
void timer_stop_i2c_time(void)
{
        app_timer_stop(m_timer_i2c_id);
}

void bmt_app_timer_init(void)
{
	timer_init_1sec();
	timer_init_sensor();
	timer_init_3min();
	timer_init_quad();
	timer_init_exec_time();
        timer_init_pwm_time();
        timer_init_i2c_time();
        timer_init_30sec();
}

void  bmt_app_timer_stop(void)
{
	timer_stop_sensor();
	timer_stop_3min();
	timer_stop_quad(0);
	timer_stop_exec_time();
        timer_stop_pwm_time();
        timer_stop_30sec();
        timer_stop_i2c_time();
}

void update_key_state(void)
{
	key_state = true;	
}
      



