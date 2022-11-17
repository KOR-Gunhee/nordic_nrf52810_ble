#ifndef __PACKET_H__
#define __PACKET_H__


#define PACKET_STEP_CMDIDLE 0
#define PACKET_STEP_CMDLEN  1
#define PACKET_STEP_CMDDATA 2

#define STX                 0x7f
#define ETX                 0x03

#define MAX_PACKET_SIZE     10


extern uint8_t volatile PacketID;
extern uint8_t volatile PacketData[10];
extern uint8_t volatile PacketDataLen;


uint32_t Packet_OnByte(uint8_t byte);







#endif	// __PACKET_H__