#include "compiler_abstraction.h"
#include "nrf.h"
#include <stdint.h>
#include "../headers/message_table.h"
#include "../headers/network_characteristics.h"

void hal_radio_config(void);
void hal_radio_set_rcv_mode(void);
void hal_radio_start_rx(void);	
void hal_radio_channel_index_set(uint8_t);
void disable_radio(void);
uint8_t tx_enable(uint32_t*,uint32_t);
extern void heartbeat(void);

