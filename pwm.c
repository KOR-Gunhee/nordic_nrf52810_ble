#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "app_pwm.h"
#include "ble_nus.h"
#include "timer.h"
#include "nrfx_log.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_clock.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "pwm.h"


#define GPIO_MOTOR		10	// BMT200
#define GPIO_RLED		18	// BMT200
#define GPIO_GLED		19	// BMT200
#define GPIO_BLED		20	// BMT200

#define ch_motor                0
#define ch_led_r                1
#define ch_led_g                0
#define ch_led_b                1

extern uint16_t m_conn_handle;

uint8_t MOTOR =	NRF_GPIO_PIN_MAP(0,10); // Motor LED 
uint8_t LED_R =	NRF_GPIO_PIN_MAP(0,18); // Red LED 
uint8_t LED_G =	NRF_GPIO_PIN_MAP(0,19); // Green LED 
uint8_t LED_B =	NRF_GPIO_PIN_MAP(0,20); // Blue LED

uint16_t vibration_power[MAX_VIBRATION_POWER_INDEX] = {1000, 850, 700, 550, 400, 250, 100};
uint8_t vibration_power_index;
uint8_t alone_power_index;
uint8_t motor_state;
uint8_t pwm_state=false;
uint8_t step=1;
uint32_t ledpo;

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

static nrf_pwm_values_individual_t seq_values[] = 
{
    {1000, 0, 0, 0}
};
nrf_pwm_sequence_t const seq =
{
    .values.p_individual = seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats         = 0,
    .end_delay       = 0
};

  
void new_motor_action(uint16_t duty_cycle)
{
     seq_values->channel_0 = duty_cycle;
     
     nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void new_led_control(uint16_t r_lux, uint16_t g_lux, uint16_t b_lux)
{
    LEDClear();
    
    if(r_lux>=0){
      nrf_drv_gpiote_out_clear(LED_R);
      seq_values->channel_1 = r_lux;
    }
    if(g_lux>=0){
      nrf_drv_gpiote_out_clear(LED_G);
      seq_values->channel_2 = g_lux;
    }
    if(b_lux>=0){
      nrf_drv_gpiote_out_clear(LED_B);
      seq_values->channel_3 = b_lux;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void new_motor_disable()
{
   NRF_LOG_INFO("----motor disable------");      
   new_motor_action(1000);
}

void new_pwm_uninit()
{
    nrf_drv_gpiote_out_clear(MOTOR);
    nrf_drv_pwm_uninit(&m_pwm0);
}

void new_pwm_init()
{
    ret_code_t err_code;
       
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
    {
        GPIO_MOTOR | NRF_DRV_PWM_PIN_INVERTED,            // channel motor
        GPIO_RLED | NRF_DRV_PWM_PIN_INVERTED,             // channel led_r
        GPIO_GLED | NRF_DRV_PWM_PIN_INVERTED,             // channel led_g
        GPIO_BLED | NRF_DRV_PWM_PIN_INVERTED,             // channel led_b
    },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 1000,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
      
    // Init PWM without error handler
    err_code = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
    APP_ERROR_CHECK(err_code);    
}
  
void motor_start(uint32_t pwm_duty)
{
    if(motor_state == false) {
      duty_val=pwm_duty;
      last_motor_power=pwm_duty;
      motor_state = true;
      PWM_Count=1;
      timer_start_pwm_time();
    }
    else{
      NRF_LOG_INFO("don't start motor");
    }
    NRF_LOG_INFO("PWM : %d", pwm_duty);
}

void motor_stop(void)
{
    if(motor_state == true) {
        NRF_LOG_INFO("motor stop");        
        duty_val=0;
	motor_state = false;
                       
        timer_start_pwm_time();
    }
    if(m_conn_handle != BLE_CONN_HANDLE_INVALID) {
       LEDBlink(LED_G);  
    }
}

void LEDClear()
{
   nrf_drv_gpiote_out_set(LED_R);
   nrf_drv_gpiote_out_set(LED_G);
   nrf_drv_gpiote_out_set(LED_B);
}
    
void LEDBlink(uint8_t idx)
{
   LEDClear();
   
   if(idx == LED_R)
      nrf_drv_gpiote_out_clear(LED_R);
   if(idx == LED_G)
      nrf_drv_gpiote_out_clear(LED_G);
   if(idx == LED_B)
      nrf_drv_gpiote_out_clear(LED_B);
}