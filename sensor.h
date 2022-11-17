#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Bank 0
#define ICM42670_DEVICE_CONFIG             0x11
#define ICM42670_DRIVE_CONFIG              0x13
//#define ICM42670_INT_CONFIG                0x14
#define ICM42670_INT_CONFIG                0x06
#define ICM42670_FIFO_CONFIG               0x16
#define ICM42670_TEMP_DATA1                0x09
#define ICM42670_TEMP_DATA0                0x0A
#define ICM42670_ACCEL_DATA_X1             0x0B
#define ICM42670_ACCEL_DATA_X0             0x0C
#define ICM42670_ACCEL_DATA_Y1             0x0D
#define ICM42670_ACCEL_DATA_Y0             0x0E
#define ICM42670_ACCEL_DATA_Z1             0x0F
#define ICM42670_ACCEL_DATA_Z0             0x10
#define ICM42670_GYRO_DATA_X1              0x11
#define ICM42670_GYRO_DATA_X0              0x12
#define ICM42670_GYRO_DATA_Y1              0x13
#define ICM42670_GYRO_DATA_Y0              0x14
#define ICM42670_GYRO_DATA_Z1              0x15
#define ICM42670_GYRO_DATA_Z0              0x16
#define ICM42670_TMST_FSYNCH               0x2B
#define ICM42670_TMST_FSYNCL               0x2C
#define ICM42670_INT_STATUS                0x2D
#define ICM42670_FIFO_COUNTH               0x2E
#define ICM42670_FIFO_COUNTL               0x2F
#define ICM42670_FIFO_DATA                 0x30
#define ICM42670_APEX_DATA0                0x31
#define ICM42670_APEX_DATA1                0x32
#define ICM42670_APEX_DATA2                0x33
#define ICM42670_APEX_DATA3                0x34
#define ICM42670_APEX_DATA4                0x35
#define ICM42670_APEX_DATA5                0x36
#define ICM42670_INT_STATUS2               0x37
#define ICM42670_INT_STATUS3               0x38
#define ICM42670_SIGNAL_PATH_RESET         0x4B
#define ICM42670_PWR_MGMT0                 0x1F
//#define ICM42670_GYRO_CONFIG0              0x4F
#define ICM42670_GYRO_CONFIG0              0x20
//#define ICM42670_ACCEL_CONFIG0             0x50
#define ICM42670_ACCEL_CONFIG0             0x21
//#define ICM42670_GYRO_CONFIG1              0x51
#define ICM42670_TEMP_CONFIG               0x22
#define ICM42670_GYRO_CONFIG1              0x23
//#define ICM42670_ACCEL_CONFIG1             0x53
#define ICM42670_ACCEL_CONFIG1             0x24
#define ICM42670_GYRO_ACCEL_CONFIG0        0x52
#define ICM42670_GYRO_ACCEL_CONFIG0        0x52
#define ICM42670_TMST_CONFIG               0x54
#define ICM42670_APEX_CONFIG0              0x56
#define ICM42670_SMD_CONFIG                0x57
#define ICM42670_FIFO_CONFIG1              0x5F
#define ICM42670_FIFO_CONFIG2              0x60
#define ICM42670_FIFO_CONFIG3              0x61
#define ICM42670_FSYNC_CONFIG              0x62
#define ICM42670_INTF_CONFIG0              0x35
#define ICM42670_INTF_CONFIG1              0x36
#define ICM42670_INT_SOURCE0               0x2B
#define ICM42670_INT_SOURCE1               0x2C
#define ICM42670_INT_SOURCE3               0x2D
#define ICM42670_INT_SOURCE4               0x2E
#define ICM42670_FIFO_LOST_PKT0            0x6C
#define ICM42670_FIFO_LOST_PKT1            0x6D
#define ICM42670_SELF_TEST_CONFIG          0x70
#define ICM42670_WHO_AM_I                  0x75
#define ICM42670_BLK_SEL_W                 0x79
#define ICM42670_BLK_SEL_R                 0x7C


// Bank 1
#define ICM42670_SENSOR_CONFIG0            0x03
#define ICM42670_GYRO_CONFIG_STATIC2       0x0B
#define ICM42670_GYRO_CONFIG_STATIC3       0x0C
#define ICM42670_GYRO_CONFIG_STATIC4       0x0D
#define ICM42670_GYRO_CONFIG_STATIC5       0x0E
#define ICM42670_GYRO_CONFIG_STATIC6       0x0F
#define ICM42670_GYRO_CONFIG_STATIC7       0x10
#define ICM42670_GYRO_CONFIG_STATIC8       0x11
#define ICM42670_GYRO_CONFIG_STATIC9       0x12
#define ICM42670_GYRO_CONFIG_STATIC10      0x13
#define ICM42670_XG_ST_DATA                0x5F
#define ICM42670_YG_ST_DATA                0x60
#define ICM42670_ZG_ST_DATA                0x61
#define ICM42670_TMSTVAL0                  0x62
#define ICM42670_TMSTVAL1                  0x63
#define ICM42670_TMSTVAL2                  0x64
#define ICM42670_INTF_CONFIG4              0x7A
#define ICM42670_INTF_CONFIG5              0x7B
#define ICM42670_INTF_CONFIG6              0x7C

// Bank 2
#define ICM42670_ACCEL_CONFIG_STATIC2      0x03
#define ICM42670_ACCEL_CONFIG_STATIC3      0x04
#define ICM42670_ACCEL_CONFIG_STATIC4      0x05
#define ICM42670_XA_ST_DATA                0x3B
#define ICM42670_YA_ST_DATA                0x3C
#define ICM42670_ZA_ST_DATA                0x3D

// Bank 4
#define ICM42670_GYRO_ON_OFF_CONFIG        0x0E
#define ICM42670_APEX_CONFIG1              0x40
#define ICM42670_APEX_CONFIG2              0x41
#define ICM42670_APEX_CONFIG3              0x42
#define ICM42670_APEX_CONFIG4              0x43
#define ICM42670_APEX_CONFIG5              0x47
#define ICM42670_APEX_CONFIG6              0x45
#define ICM42670_APEX_CONFIG7              0x46
#define ICM42670_APEX_CONFIG8              0x47
#define ICM42670_APEX_CONFIG9              0x48
#define ICM42670_ACCEL_WOM_X_THR           0x4A
#define ICM42670_ACCEL_WOM_Y_THR           0x4B
#define ICM42670_ACCEL_WOM_Z_THR           0x4C
#define ICM42670_INT_SOURCE6               0x4D
#define ICM42670_INT_SOURCE7               0x4E
#define ICM42670_INT_SOURCE8               0x4F
#define ICM42670_INT_SOURCE9               0x50
#define ICM42670_INT_SOURCE10              0x51
#define ICM42670_OFFSET_USER0              0x77
#define ICM42670_OFFSET_USER1              0x78
#define ICM42670_OFFSET_USER2              0x79
#define ICM42670_OFFSET_USER3              0x7A
#define ICM42670_OFFSET_USER4              0x7B
#define ICM42670_OFFSET_USER5              0x7C
#define ICM42670_OFFSET_USER6              0x7D
#define ICM42670_OFFSET_USER7              0x7E
#define ICM42670_OFFSET_USER8              0x7F

#define ICM42670_ADDRESS                   0x68   // Address of ICM42670 accel/gyro when ADO = HIGH


#define AFS_16G 0x00  // default
#define AFS_8G  0x01
#define AFS_4G  0x02
#define AFS_2G  0x03


#define GFS_2000DPS   0x00 // default
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
//#define GFS_125DPS    0x04
//#define GFS_62_5DPS   0x05
//#define GFS_31_25DPS  0x06
//#define GFS_15_125DPS 0x07


#define AODR_1600Hz   0x05
#define AODR_800Hz    0x06 // default
#define AODR_400Hz    0x07
#define AODR_200Hz    0x08
#define AODR_100Hz    0x09
#define AODR_50Hz     0x0A
#define AODR_25Hz     0x0B
#define AODR_12_5Hz   0x0C
#define AODR_6_25Hz   0x0D
#define AODR_3_125Hz  0x0E
#define AODR_1_5625Hz 0x0F

#define GODR_1600Hz  0x05
#define GODR_800Hz   0x06 // default
#define GODR_400Hz   0x07
#define GODR_200Hz   0x08
#define GODR_100Hz   0x09
#define GODR_50Hz    0x0A
#define GODR_25Hz    0x0B
#define GODR_12_5Hz  0x0C

#define LOWPASSFILTER_Bypass   0x01
#define LOWPASSFILTER_180Hz    0x01
#define LOWPASSFILTER_121Hz    0x02
#define LOWPASSFILTER_73Hz     0x03
#define LOWPASSFILTER_53Hz     0x04
#define LOWPASSFILTER_34Hz     0x05
#define LOWPASSFILTER_25Hz     0x06
#define LOWPASSFILTER_16Hz     0x07


extern int16_t bmt_sensor_accx;
extern int16_t bmt_sensor_accy;
extern int16_t bmt_sensor_accz;

extern int16_t bmt_sensor_gyrox;
extern int16_t bmt_sensor_gyroy;
extern int16_t bmt_sensor_gyroz;

uint8_t get_sensor_id(void);
void twi_init (void);
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
void sensor_init(void);
uint32_t BMT_ICM42670_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length);
uint32_t BMT_ICM42670_write_single_register(uint8_t reg, uint8_t data);
void sensor_val_update(int16_t * destination);
void sensor_sleep(void);

#endif	// __SENSOR_H__