#include "saadc.h"

#define SAMPLES_IN_BUFFER 5
#define MAX_ADC_RESOLUTION 1023
#define ADC_GAIN 0.1667f
#define ADC_REF_VOLTAGE 0.6f


volatile uint8_t state = 1;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;
static nrf_saadc_value_t get_adc;
static uint32_t bmt_batt_volt;
//static nrf_saadc_value_t sample;

/**
* SAADC
*/

//uint32_t get_batt_volt(void)
//{
//    return bmt_batt_volt;
//}

void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 2000ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 2000);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    //uint32_t temp_adc_val;

    //if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    //{
    //    ret_code_t err_code;

    //    err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    //    APP_ERROR_CHECK(err_code);

    //    int i;
    //    //NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

    //    for (i = 0; i < SAMPLES_IN_BUFFER; i++)
    //    {
    //        temp_adc_val = p_event->data.done.p_buffer[i];
    //        bmt_batt_volt = 360 * temp_adc_val >> 10;
    //    }

       //static uint8_t data_array[1];
       //data_array[0] = bmt_batt_volt;
       //do
       //{
       //    //NRF_LOG_INFO("BATT: %d", bmt_batt_volt);
       //    err_code = ble_nus_data_send(&m_nus, data_array, 1, m_conn_handle);
       //    if ((err_code != NRF_ERROR_INVALID_STATE) &&
       //        (err_code != NRF_ERROR_RESOURCES) &&
       //        (err_code != NRF_ERROR_NOT_FOUND))
       //     {
       //         APP_ERROR_CHECK(err_code);
       //     }
       //} while (err_code == NRF_ERROR_RESOURCES);

        //m_adc_evt_counter++;
    //}
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(NRF_SAADC_INPUT_AIN5, &channel_config);
    APP_ERROR_CHECK(err_code);
}
  
int get_batt_volt()
{  
    nrf_drv_saadc_sample_convert(NRF_SAADC_INPUT_AIN5, &get_adc);
    
    bmt_batt_volt = 360 * get_adc >> 10;
    //bmt_batt_volt = ((float)get_adc / ADC_GAIN * ADC_REF_VOLTAGE / MAX_ADC_RESOLUTION);
    NRF_LOG_INFO("Get_adc=%x", get_adc);
    NRF_LOG_INFO("Get_vol=%x", bmt_batt_volt);
    
    //nrf_delay_ms(500);
    
    return bmt_batt_volt;
}