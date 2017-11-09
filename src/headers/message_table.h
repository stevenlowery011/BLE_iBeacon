#include <stdint.h>
#pragma once

#define FW_GROUP_ID 0x10
#define CTL_GROUP_ID 0x20
#define REQ_GROUP_ID 0x30
#define RES_GROUP_ID 0x40
#define SYNC_GROUP_ID 0x50
#define DATA_GROUP_ID 0x60

#define FW_GROUP_ID_START (FW_GROUP_ID * 0x100)
#define CTL_GROUP_ID_START (CTL_GROUP_ID * 0x100)
#define REQ_GROUP_ID_START (REQ_GROUP_ID * 0x100)
#define RES_GROUP_ID_START (RES_GROUP_ID * 0x100)
#define SYNC_GROUP_ID_START (SYNC_GROUP_ID * 0x100)
#define DATA_GROUP_ID_START (DATA_GROUP_ID * 0x100)

//BEGIN and NUM_UNIQUE_MSGS use line numbers, so
//adding comments in between will return incorrect
//values for NUM_UNIQUE_MSGS. Only add MSG names

//uint8_t BEGIN = __LINE__; 
enum MSGS { 
	FW_RCV_FIRM = FW_GROUP_ID_START,     
	FW_DATA_LINE,           
	FW_TX_BGN,              
	FW_TX_OVR,
	FW_EXEC,
	FW_NUM_MISSED,
	FW_REQ_DATA_LINE,
	CTL_SHUT_UP_4PRD = CTL_GROUP_ID_START,
	CTL_SHUT_UP_4EVR,
	CTL_RESUME_COMS,
	CTL_REBOOT,
	CTL_ERASE_FW,
	CTL_SET_TYPE,
	CTL_SET_GROUP,
	REQ_WHAT_FW = REQ_GROUP_ID_START,
	RES_STATUS = RES_GROUP_ID_START,
	RES_HEALTH,
	RES_HAVE_FW_VERS,
	RES_NEED_FW_VERS,
	SYNC = SYNC_GROUP_ID_START,
	FF_ADJ_PER,
	RR_ADJ_PER,
	DATA_POSE_LOCAL = DATA_GROUP_ID_START,
	DATA_DISTANCE_REMOTE,
	GENERIC=0xFFF0,
};
//uint8_t	NUM_UNIQUE_MSGS = __LINE__ - BEGIN - 3; 


struct __attribute__((packed)) MSG_HDR {

    uint8_t Adv;
	uint8_t Length;
	uint8_t Rfu1;
	uint8_t Rfu2;
	uint16_t NID; // Swarm Group ID & Swarm Individual ID (Node)
	uint16_t SID; // Sender Group ID & Sender Individual ID
	uint16_t RID; // Receiver Group ID & Receiver Individual ID
	uint16_t MID; // Message Group ID  & Message Individual ID    
	
};

struct __attribute__((packed)) MSG_FW_RCV_FIRM {
    
  struct MSG_HDR Msg_hdr;
	uint32_t Firmware_size;
	uint16_t Version;
	uint32_t Start_Address;
	uint32_t Num_data_lines;
	uint32_t Checksum;
	uint8_t  RFU[10];
};

struct __attribute__((packed)) MSG_FW_DATA_LINE {
  struct MSG_HDR Msg_hdr;
	uint32_t Firmware_size;
	uint16_t Version;
	uint32_t Address;
	uint16_t Data_size;
	uint32_t Data[4];
};

struct __attribute__((packed)) MSG_FW_WHAT_FW {
	struct MSG_HDR Msg_hdr;
	uint32_t Firmware_size;
	uint16_t Version;
	uint8_t  RFU[22];
};

struct __attribute__((packed)) MSG_FW_TX_BGN {
  struct MSG_HDR Msg_hdr;
	uint32_t Firmware_size;
	uint16_t Version;
	uint32_t Start_Address;
	uint8_t  RFU[18];
};

struct __attribute__((packed)) MSG_FW_TX_OVR {
  struct MSG_HDR Msg_hdr;
	uint32_t Firmware_size;
	uint16_t Version;
	uint32_t Start_Address;
	uint8_t  RFU[18];
};

struct __attribute__((packed)) MSG_FW_EXEC {
  struct MSG_HDR Msg_hdr;
	uint32_t RFU[7];
};

struct __attribute__((packed)) MSG_FW_NUM_MISSED {
  struct MSG_HDR Msg_hdr;
	uint32_t Firmware_size;
	uint16_t Version;
	uint32_t Start_Address;
	uint32_t Num_missed_packets;
	uint8_t  RFU[14];
};

struct __attribute__((packed)) MSG_FW_REQ_DATA_LINE  {

  struct MSG_HDR Msg_hdr;
	uint32_t Firmware_size;
	uint16_t Version;
	uint32_t Address1;
	uint32_t Address2;
	uint32_t Address3;
	uint32_t Address4;
	uint32_t Address5;
	uint16_t RFU;
};

struct __attribute__((packed)) MSG_FW_HAVE_FW_VERS {
  struct MSG_HDR Msg_hdr;
	uint32_t Firmware_size;
	uint16_t Version;
	uint32_t Start_Address;
	uint32_t Num_data_lines;
	uint32_t Checksum;
	uint8_t  RFU[10];
};

struct __attribute__((packed)) MSG_FW_NEED_FW_VERS {
  struct MSG_HDR Msg_hdr;
	uint32_t Firmware_size;
	uint16_t Version;
	uint8_t  RFU[22];
};

struct __attribute__((packed)) MSG_ADJ_FF_PER {
  struct MSG_HDR Msg_hdr;
	uint16_t Msg_type;
	uint32_t RFU[7];
};

struct __attribute__((packed)) MSG_REBOOT {
  struct MSG_HDR Msg_hdr;
	uint32_t RFU[7];
};

struct __attribute__((packed)) MSG_ERASE_FW {
  struct MSG_HDR Msg_hdr;
	uint32_t RFU[7];
};

struct __attribute__((packed)) MSG_SYNC {
  struct MSG_HDR Msg_hdr;
	uint32_t RFU[7];
};

//POSE information of local robot
struct __attribute__((packed)) MSG_DATA_POSE_LOCAL {
  struct MSG_HDR Msg_hdr;
	uint32_t X_POS;
	uint32_t Y_POS;
	uint32_t Z_POS;
	uint32_t X_ROT;
	uint32_t Y_ROT;
	uint32_t Z_ROT;
	uint32_t RFU;

};

//contains information of a Remote Nodes' RSSI 
struct __attribute__((packed)) MSG_DATA_DISTANCE_REMOTE {
  struct  MSG_HDR Msg_hdr;
	uint32_t RSSI1;
	uint32_t RID1;
	uint32_t RSSI2;
	uint32_t RID2;
	uint32_t RSSI3;
	uint32_t RID3;
	uint32_t RFU;

};

struct __attribute__((packed)) MSG_GENERIC {
  struct MSG_HDR Msg_hdr;
	uint8_t data[28];
};

//message that is printed out of BLE Enemy
struct __attribute__((packed)) BLE_ENEMY_MSG
{
	struct MSG_GENERIC msg;  //BLE MESSAGE  (40bytes)
	uint32_t CRC;
	uint8_t  RSSI;
	uint8_t  channel;
	uint8_t  buff_pointer;
	uint8_t  last_packet_printed;
	uint8_t  Rfu;
};

