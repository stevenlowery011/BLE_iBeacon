#include "compiler_abstraction.h"
#include "nrf.h"
#include <stdint.h>


#define TX_DATA_SIZE 31											  //total transmit data size (bytes)
#define TX_BUF_SIZE	 10												//total number of packets to transmit
#define SCAN_RSP	0x04												//PDU type
#define DEV_ADDR	(uint64_t)(0x3D8E657A148A)	//devices adv_address

extern void disable_radio(void);
extern void hal_radio_start_rx(void);
extern uint8_t tx_enable(uint32_t*,uint32_t);


void BLE_transmit_data(uint8_t*, uint8_t);
