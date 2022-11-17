#ifndef __TIMER_H__
#define __TIMER_H__

typedef struct
{
	uint16_t year;
	uint8_t  mon;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  min;
	uint8_t  sec;
	uint8_t  dur;
} bmt_time_t;

void bmt_app_timer_init(void);
void bmt_app_timer_stop(void);

void bmt_set_time(bmt_time_t);

void timer_start_sensor(void);
void timer_stop_sensor(void);
void timer_start_3min(void);
void timer_stop_3min(void);
void timer_start_quad(void);
void timer_stop_quad(uint8_t flag);
void timer_start_haptic(void);
void timer_stop_haptic(void);
void timer_start_1sec(void);
void timer_stop_1sec(void);
void timer_start_exec_time(void);
void timer_stop_exec_time();
void timer_start_pwm_time(void);
void timer_stop_pwm_time(void);
void timer_start_30sec(void);
void timer_stop_30sec(void);
void timer_start_i2c_time(void);
void timer_stop_i2c_time(void);

uint32_t get_exec_time_cnt(void);

void update_key_state(void);
void print_time(bmt_time_t m_time);
void test();


extern bmt_time_t local_time;
extern bmt_time_t exec_time;
extern bmt_time_t save_time;
extern uint32_t duty_val;
extern uint32_t last_motor_power;
extern uint16_t test_time_cnt;
extern uint8_t debug_flag;
extern uint8_t PWM_Count;
extern uint8_t key_state;

#endif	// __TIMER_H__
