#include <stdint.h>
#include <string.h>
#include "packet.h"
#include "nrfx_log.h"

uint8_t volatile Packet_RxStep=0;
uint8_t volatile PacketData[10];
uint8_t volatile PacketDataLen=0;

uint8_t PacketChecksum=0;

uint32_t Packet_OnByte(uint8_t byte)
{
    uint8_t ret=0;
    uint8_t len=0;
    
    //NRF_LOG_INFO("What : 0x%02X", byte);               
    switch(Packet_RxStep)
    {
        case PACKET_STEP_CMDIDLE:
                if(byte==STX) {
                    Packet_RxStep = PACKET_STEP_CMDLEN;
                    PacketDataLen=0;
                    PacketChecksum=0xFF;
                }
                else {
                    BMT_execute_cmd(byte);
                    Packet_RxStep=PACKET_STEP_CMDIDLE;
                }
                break;
        case PACKET_STEP_CMDLEN:
                len=byte;
                Packet_RxStep=PACKET_STEP_CMDDATA;
                break;
        case PACKET_STEP_CMDDATA:
                if(byte==ETX){
                    if(len==PacketDataLen)
                    {
                      Packet_RxStep=PACKET_STEP_CMDIDLE;
                      for(uint8_t ucount=0;ucount<len-1;ucount++)
                      {
                          NRF_LOG_INFO("PacketData : 0x%X", PacketData[ucount]);
                          PacketChecksum+=PacketData[ucount];
                          
                      }
                      NRF_LOG_INFO("Checksum : 0x%02X", ~PacketChecksum);  
                      if(~PacketChecksum!=PacketData[len])
                      {
                          ret=0;
                          break;
                      }
                        
                      ret=1;			
                      break;
                    } 
                    else{
                      PacketData[PacketDataLen++]=byte;
                    }
                    Packet_RxStep=PACKET_STEP_CMDIDLE;
                    ret=1;
                    break;
                } 
                if(PacketDataLen < MAX_PACKET_SIZE) {
                    PacketData[PacketDataLen++]=byte;
                }
                else {
                    PacketDataLen=0;
                    Packet_RxStep=PACKET_STEP_CMDIDLE;
                }        
                break;
                
        default:
                Packet_RxStep = PACKET_STEP_CMDIDLE;
                break;
    }
    return ret;
}