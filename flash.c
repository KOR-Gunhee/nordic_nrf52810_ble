#include <stdint.h>
#include <string.h>

#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"
#include "nrf_soc.h"
#include "nrf_nvic.h"
#include "nrf_delay.h"
#else
#include "nrf_drv_clock.h"
#include "nrf_fstorage_nvmc.h"
#endif

#include "pwm.h"
#include "flash.h"
#include "main.h"

#define PAGE_SIZE_WORDS 256


uint8_t flash_flag=0;
uint32_t page_1_datas[PAGE_SIZE_WORDS];
volatile uint8_t erase_flag = 0;
volatile uint8_t store_flag = 0;

uint32_t flash_update_flag = 0;
uint32_t flash_update_time_flag = false;

/**@brief   Sleep until an event is received. */
static void power_manage(void)
{
#ifdef SOFTDEVICE_PRESENT
    (void) sd_app_evt_wait();
#else
    __WFE();
#endif
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x26000,
    .end_addr   = 0x27fff,
};

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_READ_RESULT:
        {
            store_flag = 0;
            flash_flag = 0;
            NRF_LOG_INFO("--> Event received: read %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);

        } break;
        
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            store_flag = 0;
            flash_flag = 0;
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);

        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            erase_flag = 0;
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

void flash_init(void)
{
    ret_code_t rc;
    nrf_fstorage_api_t * p_fs_api;

#ifdef SOFTDEVICE_PRESENT
    NRF_LOG_INFO("SoftDevice is present.");
    NRF_LOG_INFO("Initializing nrf_fstorage_sd implementation...");
    /* Initialize an fstorage instance using the nrf_fstorage_sd backend.
     * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
     * used whenever there is a SoftDevice, regardless of its status (enabled/disabled). */
    p_fs_api = &nrf_fstorage_sd;
#else
    NRF_LOG_INFO("SoftDevice not present.");
    NRF_LOG_INFO("Initializing nrf_fstorage_nvmc implementation...");
    /* Initialize an fstorage instance using the nrf_fstorage_nvmc backend.
     * nrf_fstorage_nvmc uses the NVMC peripheral. This implementation can be used when the
     * SoftDevice is disabled or not present.
     *
     * Using this implementation when the SoftDevice is enabled results in a hardfault. */
    p_fs_api = &nrf_fstorage_nvmc;
#endif

    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);

    //print_flash_info(&fstorage);
}

void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}

// Retrieve the address of a page.
static uint32_t const * address_of_page(uint16_t page_num)
{
    return fstorage.start_addr + (page_num * PAGE_SIZE_WORDS);
}

uint8_t get_vibration_power(void)
{
    uint32_t flash_data;
    uint8_t ret;
    ret_code_t rc;

    rc = nrf_fstorage_read(&fstorage, address_of_page(0), &flash_data, sizeof(flash_data));
            
    if(flash_data > MAX_VIBRATION_POWER_INDEX - 1) {
        ret = BASE_VIBRATION_POWER_INDEX;
    }
    else {
	ret = flash_data;
    }
      
    NRF_LOG_INFO("Flah Data (Vibration Power IDX) : %d \r\n", ret);
    
    return ret;
}

void flash_exec_time_send(uint32_t data_h, uint32_t data_l)
{
	uint8_t tx_packet[20] = {0,};
	uint16_t tx_len = 0;
	uint8_t idx = 0;
	bmt_time_t save_time;
        
        
        tx_packet[idx++] = 'e';	// '0x6C' Header
        tx_packet[idx++] = (uint8_t)((data_h & 0xFF000000) >> 24);
        tx_packet[idx++] = (uint8_t)((data_h & 0x00FF0000) >> 16);
        tx_packet[idx++] = (uint8_t)((data_h & 0x0000FF00) >> 8);
        tx_packet[idx++] = (uint8_t)(data_h & 0x000000FF);
        tx_packet[idx++] = (uint8_t)((data_l & 0x00FF0000) >> 24);
        tx_packet[idx++] = (uint8_t)((data_l & 0x00FF0000) >> 16);
        tx_packet[idx++] = (uint8_t)((data_l & 0x00FF0000) >> 8);
        tx_packet[idx++] = (uint8_t)(data_l & 0x00FF0000);
        tx_packet[idx] = BMT_calc_checksum(tx_packet, idx);
        
        tx_len = ++idx;
        
        NRF_LOG_INFO("TX LOCAL_TIME  : ");
        NRF_LOG_INFO("%d-", (data_h & 0xFFFF0000) >> 16);
        NRF_LOG_INFO("%d-", (data_h & 0x0000FF00) >> 8);
        NRF_LOG_INFO("%d ", data_h & 0x000000FF);
        NRF_LOG_INFO("%d:", (data_l & 0xFF000000) >> 24);
        NRF_LOG_INFO("%02d:", (data_l & 0x00FF0000) >> 16);
        NRF_LOG_INFO("%02d:", (data_l & 0x0000FF00) >> 8);
        NRF_LOG_INFO("%02d \r\n", data_l & 0x000000FF);
        
//        print_time(exec_time);
  
        ble_send_data((uint8_t *)tx_packet, tx_len);
        //ble_nus_data_send(&m_nus, (uint8_t *)tx_packet, tx_len, m_conn_handle);
        return;
}
 
void flash_write(uint32_t start_page, uint32_t data)
{
      ret_code_t rc;
      
      store_flag = 1;
      flash_flag = 1;
      
      rc = nrf_fstorage_write(&fstorage, start_page, &data, sizeof(data), NULL);

      if (rc != NRF_SUCCESS)
      {
          NRF_LOG_INFO("fs_write error \r\n");
      }else{
          NRF_LOG_INFO("fs_write success \r\n");
      }

      while(store_flag == 1) {
          power_manage();
      }

}

void flash_read(uint32_t start_page, uint32_t* data)
{
    ret_code_t rc;
    
    rc = nrf_fstorage_read(&fstorage, start_page, data, sizeof(data));
        if (rc != NRF_SUCCESS) {NRF_LOG_INFO("fs_read error \r\n");}
        else {NRF_LOG_INFO("fs_read ok \r\n");}
}
      
void flash_erase(uint32_t start_page, uint32_t page_num)
{
    //Erase one page
    erase_flag = 1;

    NRF_LOG_INFO("ERASE PAGE (%d PAGE ~ %d PAGE)\t", start_page, (start_page + page_num - 1));
    NRF_LOG_INFO("Start Address 0x%X\r\n", (uint32_t)address_of_page(start_page));
    NRF_LOG_INFO("End Address 0x%X\r\n", (uint32_t)address_of_page(start_page + page_num - 1));

    ret_code_t rc = nrf_fstorage_erase(&fstorage, address_of_page(start_page), page_num, NULL);
    if (rc != NRF_SUCCESS)
    {
         NRF_LOG_INFO("fs_erase error \r\n");
    }
    else {
        NRF_LOG_INFO("fs_erase ok \r\n");
    }

    while(erase_flag == 1) {
      power_manage();
    }
}
   
void update_vibration_power(uint32_t data)
{
      ret_code_t rc;
      volatile uint32_t flash_data;

      flash_erase(0, 1);	// void flash_erase(uint32_t start_page, page_num)
	
      //Read data
      NRF_LOG_INFO("Data read from flash address 0x%X: ", (uint32_t)address_of_page(0));

      //Write one word
      NRF_LOG_INFO("Writing data 0x%X to address 0x%X\r\n", data, address_of_page(0));

      NRF_LOG_INFO("data : %X \r\n", data);
      
      store_flag = 1;
      flash_flag = 1;
      
      rc = nrf_fstorage_write(&fstorage, address_of_page(0), &data, sizeof(data), NULL);

      if (rc != NRF_SUCCESS)
      {
          NRF_LOG_INFO("fs_write error \r\n");
      }else{
          NRF_LOG_INFO("fs_write success \r\n");
      }

      while(store_flag == 1) {
          power_manage();
      }
      
      //Read data
     ///NRF_LOG_INFO("Data read from flash address 0x%X: ", address_of_page(0));
}
  
/********************************************************************************
*																				*
*	∞Ïù¥¨Í∏∞ : 8 bytes														*
*	1 page ¨Í∏∞ : 0x400 = 1024 bytes											*
*	1 page 128 Í∞∞Ïù¥ÄÍ∞Ä										*
*	∞Ïù¥Ä•Ïóê 8Í∞page ¨Ïö©													*
*	8Í∞page êÎäî 1024 Í∞∞Ïù¥ÄÍ∞Ä									*
*																				*
********************************************************************************/
#define MAX_DATA_CNT				1024
#define TIME_DATA_SIZE				8
#define DATA_NUM_PER_PAGE			128
#define EXEC_TIME_PAGE_NUM_START	0
#define EXEC_TIME_PAGE_NUM_USE		8
#define EXEC_TIME_PAGE_NUM_END		(EXEC_TIME_PAGE_NUM_START + EXEC_TIME_PAGE_NUM_USE)

uint16_t exec_time_count(void)
{
	uint32_t i;
	uint16_t ret_cnt = 0;
	uint32_t *p_addr = (uint32_t *)(address_of_page(1));
	uint32_t read_data_0;
	uint32_t read_data_1;

	for(i = 0; i < MAX_DATA_CNT; i++){
		read_data_0 = *(p_addr++);
		read_data_1 = *(p_addr++);
                
                //NRF_LOG_INFO("read_data_0 : 0x%08X\r\n", read_data_0);
                //NRF_LOG_INFO("read_data_1 : 0x%08X\r\n", read_data_1);
		
		if((read_data_0 == 0xFFFFFFFF) && (read_data_1 == 0xFFFFFFFF)) {
			ret_cnt = i;
			break;
		}
	}
	return ret_cnt;
}
      
bmt_time_t exec_time_read(uint16_t num)
{
	uint32_t *p_addr_h;
        uint32_t *p_addr_l;
	bmt_time_t ret_time;
	union _UNION_TIME _t;
	
	//p_addr = (uint32_t *)(address_of_page(1) + (num * 2));
	p_addr_h = (uint32_t *)address_of_page(1)+(num*2);
        p_addr_l = (uint32_t*) address_of_page(1)+1+(num*2);
        
        NRF_LOG_INFO("memory_count : %d", num);
        NRF_LOG_INFO("0x%X ~ 0x%X : 0x%08X ", p_addr_h, p_addr_l, *(p_addr_h));
        NRF_LOG_INFO("0x%X ~ 0x%X : 0x%08X ", p_addr_l, p_addr_l + 1, *(p_addr_l));        
        
        
        save_time.year = (*(p_addr_h)&0xffff0000) >> 16;
        save_time.mon = (*(p_addr_h)&0x0000ff00) >> 8;
        save_time.day = (*(p_addr_h)&0x000000ff);
        save_time.hour = (*(p_addr_l)&0xff000000) >> 24;
        save_time.min = (*(p_addr_l)&0x00ff0000) >> 16;
        save_time.sec = (*(p_addr_l)&0x0000ff00) >> 8;
        save_time.dur = (*(p_addr_l)&0x000000ff);
        
        NRF_LOG_INFO("TX LOCAL_TIME : %d-%d-%d %d:%02d:%02d \r\n",
        save_time.year, save_time.mon, save_time.day, save_time.hour, save_time.min, save_time.sec);
        NRF_LOG_INFO("%d",save_time.dur);
        
	_t.union_data[0] = *(p_addr_h);
	_t.union_data[1] = *(p_addr_l);
	
        //NRF_LOG_INFO("ret_time : %d", ret_time);
        
	ret_time = _t.union_time;

	return ret_time;
}

bmt_time_t exec_time_print(uint16_t num)
{
	uint16_t cnt;
	uint16_t i;
	bmt_time_t temp_time;

	cnt = exec_time_count();
	for(i = 0; i < cnt; i++) {
                temp_time = exec_time_read(i);
                
  //              NRF_LOG_INFO("---------exec_time----------------");
  //              NRF_LOG_INFO("%d ", exec_time_read(i).year);
		//NRF_LOG_INFO("%d ", exec_time_read(i).mon);
		//NRF_LOG_INFO("%d ", exec_time_read(i).day);
		//NRF_LOG_INFO("%d ", exec_time_read(i).hour);
		//NRF_LOG_INFO("%d ", exec_time_read(i).min);
		//NRF_LOG_INFO("%d ", exec_time_read(i).sec);
		//NRF_LOG_INFO("%d \r\n", exec_time_read(i).dur);
  //              NRF_LOG_INFO("---------temp_time----------------");
  //                              NRF_LOG_INFO("%d ", exec_time_read(i).year);
		//NRF_LOG_INFO("%d ", temp_time.mon);
		//NRF_LOG_INFO("%d ", temp_time.day);
		//NRF_LOG_INFO("%d ", temp_time.hour);
		//NRF_LOG_INFO("%d ", temp_time.min);
		//NRF_LOG_INFO("%d ", temp_time.sec);
		//NRF_LOG_INFO("%d \r\n", temp_time.dur);
                
                
	}
        NRF_LOG_INFO("EXEC TIME COUNT (%d)\r\n", cnt);
}

void exec_time_erase(void)
{
	uint16_t data_num;
	uint8_t erase_page_num;

	data_num = exec_time_count();
        NRF_LOG_INFO("data_num : %d", data_num);
        
	erase_page_num = (data_num / DATA_NUM_PER_PAGE) + 1;
	NRF_LOG_INFO("erase_page_num : %d", erase_page_num);
        
	if(erase_page_num > EXEC_TIME_PAGE_NUM_USE) {
		erase_page_num = EXEC_TIME_PAGE_NUM_USE;
	}
	
	flash_erase(EXEC_TIME_PAGE_NUM_START, erase_page_num);
        //flash_erase(0,2);
        update_vibration_power(vibration_power_index);
}

      
bmt_time_t exec_time_packet_TX(uint16_t num)
{
    ret_code_t rc;
    uint32_t flash_data_h, flash_data_l;
    uint32_t memory_count=0;
        
    memory_count=exec_time_count();
    //memory_count=20;
          
    for(uint32_t cnt=0;cnt<memory_count;cnt++)
    {
        flash_read(((uint32_t)address_of_page(1)+(cnt*0x08)), &flash_data_h);
        nrf_delay_ms(1);
        flash_read(((uint32_t)address_of_page(1)+0x04+(cnt*0x08)), &flash_data_l);
        
        NRF_LOG_INFO("memory_count : %d", memory_count);
        NRF_LOG_INFO("read_address/exec_time_h : 0x%X 0x%08X \r\n", ((uint32_t)address_of_page(1)+(cnt*0x08)), flash_data_h);
        NRF_LOG_INFO("read_address/exec_time_l : 0x%X 0x%08X \r\n", ((uint32_t)address_of_page(1)+0x02+(cnt*0x08)), flash_data_l);        
                       
                 
        //flash_exec_time_send(flash_data_h, flash_data_l);
    }    
}

void exec_time_write()
{
	uint32_t exec_time_h, exec_time_l = 0;
        uint32_t memory_count=0;
        
        memory_count=exec_time_count();
        
        if (memory_count>=990) {
          sd_nvic_SystemReset();
        }
        
        exec_time_h=((exec_time.year<<16)+(exec_time.mon<<8)+(exec_time.day));
        exec_time_l=((exec_time.hour<<24)+(exec_time.min<<16)+(exec_time.sec<<8)+(exec_time.dur));

        NRF_LOG_INFO("exec_time_dur : %x", exec_time.dur);    
        NRF_LOG_INFO("0x%X ~ 0x%X : 0x%08X ", ((uint32_t)address_of_page(1)+(memory_count*0x08)), ((uint32_t)address_of_page(1)+0x04+(memory_count*0x08)), exec_time_h);
        NRF_LOG_INFO("0x%X ~ 0x%X : 0x%08X ", ((uint32_t)address_of_page(1)+0x04+(memory_count*0x08)), ((uint32_t)address_of_page(1)+0x08+(memory_count*0x08)), exec_time_l);        

        flash_write(((uint32_t)address_of_page(1)+(memory_count*0x08)), exec_time_h);
        nrf_delay_ms(1);
        flash_write(((uint32_t)address_of_page(1)+0x04+(memory_count*0x08)),exec_time_l);
        
        sd_nvic_SystemReset();
}
      
void exec_time_write_test()
{
	uint32_t exec_time_h, exec_time_l = 0;
        uint32_t memory_count=0;
        
        memory_count=exec_time_count();
        
        exec_time_h=((exec_time.year<<16)+(exec_time.mon<<8)+(exec_time.day));
        exec_time_l=((exec_time.hour<<24)+(exec_time.min<<16)+(exec_time.sec<<8)+(exec_time.dur));
        
        NRF_LOG_INFO("TEST exec_time_dur : %x", exec_time.dur);
        NRF_LOG_INFO("0x%X ~ 0x%X : 0x%08X ", ((uint32_t)address_of_page(1)+(memory_count*0x08)), ((uint32_t)address_of_page(1)+0x04+(memory_count*0x08)), exec_time_h);
        NRF_LOG_INFO("0x%X ~ 0x%X : 0x%08X ", ((uint32_t)address_of_page(1)+0x04+(memory_count*0x08)), ((uint32_t)address_of_page(1)+0x08+(memory_count*0x08)), exec_time_l);
        
        flash_write(((uint32_t)address_of_page(1)+(memory_count*0x08)), exec_time_h);
        nrf_delay_ms(1);
        flash_write(((uint32_t)address_of_page(1)+0x04+(memory_count*0x08)),exec_time_l);
}

void flash_operation(void)
{
	if(flash_update_flag == FLASH_UPDATE_FLAG_VIBRATION_POWER) {
		update_vibration_power(vibration_power_index);
                //flash_flag=1;               
	}
	else if(flash_update_flag == FLASH_UPDATE_FLAG_EXEC_TIME_ERASE) {
		exec_time_erase();
        }
        else if(flash_update_flag == FLASH_UPDATE_FLAG_EXEC_ALON_WRITE) {
                sd_nvic_SystemReset();                
                //exec_time_write( );
	}
	else {
		// Do nothing...	
	}

	flash_update_flag = FLASH_UPDATE_FLAG_NONE;
}

void flash_clear()
{
      flash_erase(0,2);
      update_vibration_power(vibration_power_index);
}
    
