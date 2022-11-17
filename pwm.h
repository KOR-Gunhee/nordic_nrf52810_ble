#ifndef __PWM_H__
#define __PWM_H__


#define MAX_VIBRATION_POWER_INDEX	7
#define BASE_VIBRATION_POWER_INDEX      3
#define MAX_VIBRATION_POWER		1000
#define BASE_VIBRATION_POWER            700
#define HAPTIC_VIBRATION_POWER1		1000
#define HAPTIC_VIBRATION_POWER2		500
#define HAPTIC_DELAY                    70


#define RED                     0
#define GREEN                   1
#define BLUE                    2
#define YELLOW                  3
#define CYAN                    4
#define MAGENTA                 5
#define WHITE                   6
#define EXIT                    7



void new_motor_action(uint16_t duty_cycle);
void new_led_control(uint16_t r_lux, uint16_t g_lux, uint16_t b_lux);
void new_motor_disable();
void new_pwm_uninit();
void new_pwm_init();

void pwm_init(void);
void pwm_onoff(uint8_t onoff);
void motor_start(uint32_t pwm_duty);
void haptic_mode(uint32_t pwm_duty);
void motor_disable();
void motor_stop(void);
void motor_action(uint32_t pwm_duty);
void pwm_disable();
void LEDClear();
void LEDBlink(uint8_t idx);
void led_test(uint8_t lux);
void led_control(uint8_t r_lux, uint8_t g_lux, uint8_t b_lux);
void led_color(uint8_t color, uint8_t lux);
void pmw_start();


extern uint16_t vibration_power[MAX_VIBRATION_POWER_INDEX];
extern uint8_t vibration_power_index;
extern uint8_t alone_power_index;
extern uint8_t motor_state;
extern uint8_t pwm_state;
extern uint8_t step;

extern uint8_t MOTOR;
extern uint8_t LED_R;
extern uint8_t LED_G;
extern uint8_t LED_B;

#endif	// __PWM_H__