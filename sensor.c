/* TWI instance ID. */
#include "sensor.h"
#include "nrf_drv_twi.h"
#define TWI_INSTANCE_ID     0
#define BMA250E_TWI_TIMEOUT 10000 

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

uint8_t bmt_sensor_id;

int16_t bmt_sensor_accx;
int16_t bmt_sensor_accy;
int16_t bmt_sensor_accz;

int16_t bmt_sensor_gyrox;
int16_t bmt_sensor_gyroy;
int16_t bmt_sensor_gyroz;

uint8_t sampling_flag=0;

uint8_t get_sensor_id(void)
{
    return bmt_sensor_id;
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    twi_rx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                    twi_rx_done = true;
                    break;
                default:
                    break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            //NRF_LOG_INFO("NRF_DRV_TWI_EVT_ADDRESS_NACK");
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            //NRF_LOG_INFO("NRF_DRV_TWI_EVT_DATA_NACK");
            break;
        default:
            break;
    }
}


/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

uint32_t BMT_ICM42670_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = BMA250E_TWI_TIMEOUT;

    err_code = nrf_drv_twi_tx(&m_twi, ICM42670_ADDRESS, &reg, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    err_code = nrf_drv_twi_rx(&m_twi, ICM42670_ADDRESS, p_data, length);
    if(err_code != NRF_SUCCESS) return err_code;

    timeout = BMA250E_TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_rx_done = false;

    return err_code;
}

uint32_t BMT_ICM42670_write_single_register(uint8_t reg, uint8_t data)
{
    uint32_t err_code;
    uint32_t timeout = BMA250E_TWI_TIMEOUT;

    uint8_t packet[2] = {reg, data};

    err_code = nrf_drv_twi_tx(&m_twi, ICM42670_ADDRESS, packet, 2, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    twi_tx_done = false;

    return err_code;
}

void sensor_init(void)
{
    BMT_ICM42670_read_registers(ICM42670_WHO_AM_I, &bmt_sensor_id, 1);
    //NRF_LOG_INFO("WHO AM I : %X",bmt_sensor_id);

    uint8_t pwr_mgmt0=0x00;
    BMT_ICM42670_write_single_register(ICM42670_PWR_MGMT0, pwr_mgmt0 | 0x0F); // enable gyro and accel in low noise mode
    BMT_ICM42670_read_registers(ICM42670_PWR_MGMT0, &pwr_mgmt0, 1);    // make sure not to disturb reserved bit values
    //NRF_LOG_INFO("pwr_mgmt0 : %X",pwr_mgmt0);

    uint8_t gyro_config0=0x00;
    BMT_ICM42670_write_single_register(ICM42670_GYRO_CONFIG0, gyro_config0 | GODR_1600Hz | (GFS_1000DPS<<5) ); // gyro full scale and data rate
    BMT_ICM42670_read_registers(ICM42670_GYRO_CONFIG0, &gyro_config0, 1);
    //NRF_LOG_INFO("gyro_config0 : %X",gyro_config0);
    
    uint8_t accel_config0=0x00;
    BMT_ICM42670_write_single_register(ICM42670_ACCEL_CONFIG0, accel_config0 | AODR_1600Hz | (AFS_4G<<5) ); // set accel full scale and data rate
    BMT_ICM42670_read_registers(ICM42670_ACCEL_CONFIG0, &accel_config0, 1); 
    //NRF_LOG_INFO("accel_config0 : %X",accel_config0);
   
    uint8_t gyro_config1=0x00;
    BMT_ICM42670_write_single_register(ICM42670_GYRO_CONFIG1, gyro_config1 | LOWPASSFILTER_34Hz); //  Gyro Low Pass Filter 25 Hz 
    BMT_ICM42670_read_registers(ICM42670_GYRO_CONFIG1, &gyro_config1, 1);
    //NRF_LOG_INFO("gyro_config1 : %X",gyro_config1);
    
    uint8_t accel_config1=0x00;
    BMT_ICM42670_write_single_register(ICM42670_ACCEL_CONFIG1, accel_config1 | LOWPASSFILTER_34Hz ); //  Accel Low Pass Filter 25 Hz 
    BMT_ICM42670_read_registers(ICM42670_ACCEL_CONFIG1, &accel_config1, 1);
    //NRF_LOG_INFO("accel_config1 : %X",accel_config1);
    
    uint8_t int_config=0x00;
    BMT_ICM42670_write_single_register(ICM42670_INT_CONFIG, int_config | 0x18 | 0x03); // set both interrupts active high, push-pull, pulsed
    BMT_ICM42670_read_registers(ICM42670_INT_CONFIG, &int_config, 1);
    //NRF_LOG_INFO("int_config : %X",int_config);
    
    uint8_t int_source0=0x00;
    BMT_ICM42670_write_single_register(ICM42670_INT_SOURCE0, int_source0 | 0x08); // route data ready interrupt to INT1
    BMT_ICM42670_read_registers(ICM42670_INT_SOURCE0, &int_source0, 1);
    //NRF_LOG_INFO("int_source0 : %X",int_source0);
    
    uint8_t int_source3=0x00;
    BMT_ICM42670_write_single_register(ICM42670_INT_SOURCE3, int_source3 | 0x01); // route AGC interrupt interrupt to INT2
    BMT_ICM42670_read_registers(ICM42670_INT_SOURCE3, &int_source3, 1);
    //NRF_LOG_INFO("int_source3 : %X",int_source3);
}

void sensor_val_update(int16_t * destination)
{
    uint8_t bit_shift=5;
    uint8_t rawData[14];  // x/y/z accel register data stored here
    BMT_ICM42670_read_registers(ICM42670_TEMP_DATA1, rawData, 14);

    //destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    //destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    //destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
    //destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
    //destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
    //destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
    //destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
    
    destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) >> bit_shift ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = -(int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) >> bit_shift ;
    destination[2] = -(int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) >> bit_shift ;
    destination[3] = (int16_t)(((int16_t)rawData[6] << 8) | rawData[7]) >> bit_shift ;
    destination[4] = (int16_t)(((int16_t)rawData[8] << 8) | rawData[9]) >> bit_shift ;
    destination[5] = (int16_t)(((int16_t)rawData[10] << 8) | rawData[11]) >> bit_shift ;
    destination[6] = (int16_t)(((int16_t)rawData[12] << 8) | rawData[13]) >> bit_shift ;
        
}

void sensor_sleep(void)
{
     BMT_ICM42670_write_single_register(ICM42670_PWR_MGMT0, 0x00);
}
   
