#include "nrf.h"
#include "./src/hal/gpio_setup.h"
#include "./src/hal/hal_radio.h"
#include "./src/hal/tx_data_handler.h"


#define DD_FW_VERSION NRF_UICR->CUSTOMER[0]		//whatever firmware version we are on.  0 is test
#define DD_ROBO_ID    NRF_UICR->CUSTOMER[1]		//32 bit robot ID generated by RNG

#define TEST_VERS		0

void heartbeat(void);


