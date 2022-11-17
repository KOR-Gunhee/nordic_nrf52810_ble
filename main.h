#ifndef __MAIN_H__
#define __MAIN_H__

//#define BMT_FW_VER				10	// ȭ ÷  FW  10  ϱ...
//#define BMT_FW_VER				11	// iOS APP  MAC Address Ȯ  Manufacturer Data ߰
#define BMT_FW_VER				5       // NACK ó ߰

#define BMT_STATE_LOW_POWER		0
#define BMT_STATE_ACTIVE                1
#define BMT_STATE_CONNECT		2
#define BMT_STATE_STAND_ALONE           3


/****************************************************
*	UART  Ϸ Ʒ ּ  Ѵ.		*
*****************************************************/
//#define UART_DEBUG

#ifdef UART_DEBUG
	#define UART_PRINTF(msg...) {printf(msg);}
#else
	#define UART_PRINTF(msg...)
#endif	// UART_DEBUG

#define NRF_SZ_UART_BUFF 32

static volatile bool ready_flag;            // A flag indicating PWM status.

extern uint32_t tx_enable_device_info;
extern uint32_t tx_enable_local_time;
extern uint32_t tx_enable_ack_current_time;
extern uint32_t tx_enable_exec_time;
extern uint32_t tx_enable_exec_time_again;
extern uint32_t tx_enable_exec_time_call;
extern uint32_t tx_enable_sensor_info;

uint8_t BMT_calc_checksum(uint8_t *data, uint32_t len);
void BMT_execute_cmd(uint8_t p_data);
void BMT_packet_cmd();
void BMT_system_low_power_enter(void);
//void changeLEDStatus(uint8_t idx);


typedef struct{
    uint32_t Head;
    uint32_t Tail;
    uint8_t Buff[NRF_SZ_UART_BUFF];
}NRF_UART_FIFO;
  
extern NRF_UART_FIFO volatile nrf_uart_buff;

#endif	// __MAIN_H__