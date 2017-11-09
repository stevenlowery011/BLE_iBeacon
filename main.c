#include "compiler_abstraction.h"
#include "main.h"
#include "nrf.h"
#include "./src/headers/message_table.h"
#include "./src/headers/network_characteristics.h"
#include <stdint.h>


#define MS_DELAY_100	3125


uint8_t *tx_data_pointer;
uint8_t tx_ready_flag = 1;


struct __attribute__((packed)) MSG_BEACON {
	
	uint8_t S0;
	uint8_t Length;
	uint8_t S1;
	uint8_t MAC_ADDR[6];
	uint8_t iBeacon_prefix[9];
	uint8_t UUID[16];
	uint8_t Major[2];
	uint8_t Minor[2];
	uint8_t TXpower;	
	
};

struct MSG_BEACON iBeacon_message;

void setup_beacon_message()
{
	iBeacon_message.S0 = 0x40;
	iBeacon_message.Length = 0x24;
	iBeacon_message.S1 = 0;
	iBeacon_message.MAC_ADDR[0] = 0x05;
	iBeacon_message.MAC_ADDR[1] = 0xa2; 
	iBeacon_message.MAC_ADDR[2] = 0x17; 
	iBeacon_message.MAC_ADDR[3] = 0x6e; 
	iBeacon_message.MAC_ADDR[4] = 0x3d; 
	iBeacon_message.MAC_ADDR[5] = 0x71;
	iBeacon_message.iBeacon_prefix[0] = 0x02;
	iBeacon_message.iBeacon_prefix[1] = 0x01;
	iBeacon_message.iBeacon_prefix[2] = 0x1a;
	iBeacon_message.iBeacon_prefix[3] = 0x1a;
	iBeacon_message.iBeacon_prefix[4] = 0xff;
	iBeacon_message.iBeacon_prefix[5] = 0x4c;
	iBeacon_message.iBeacon_prefix[6] = 0x00;
	iBeacon_message.iBeacon_prefix[7] = 0x02;
	iBeacon_message.iBeacon_prefix[8] = 0x15;
	iBeacon_message.UUID[0] = 0xe2;
	iBeacon_message.UUID[1] = 0xc5;
	iBeacon_message.UUID[2] = 0x6d;
	iBeacon_message.UUID[3] = 0xb5;
	iBeacon_message.UUID[4] = 0xdf;
	iBeacon_message.UUID[5] = 0xfb;
	iBeacon_message.UUID[6] = 0x48;
	iBeacon_message.UUID[7] = 0xd2;
	iBeacon_message.UUID[8] = 0xb0;
	iBeacon_message.UUID[9] = 0x60;
	iBeacon_message.UUID[10] = 0xd0;
	iBeacon_message.UUID[11] = 0xf5;
	iBeacon_message.UUID[12] = 0xa7;
	iBeacon_message.UUID[13] = 0x10;
	iBeacon_message.UUID[14] = 0x96;
	iBeacon_message.UUID[15] = 0xe0;
	iBeacon_message.Major[0] = 0x00;
	iBeacon_message.Major[1] = 0x00;
	iBeacon_message.Minor[0] = 0x00;
	iBeacon_message.Minor[1] = 0x00;
	iBeacon_message.TXpower = 0xc5;
}




void clock_initialization()
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    /* Start low frequency crystal oscillator for app_timer(used by bsp)*/
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
		//NRF_CLOCK->LFCLKSRC =0;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}


void setup_timer()
{
	NRF_TIMER0->TASKS_STOP = 1;													//stop the timer to configure it
	NRF_TIMER0->PRESCALER = 9;													//set timer prescaler to 2^9 (16MHz/512 = 31250) for 32us ticks
	NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;						//set module to timer mode
	NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;	//make it a 24-bit timer so max = 536.87s
	NRF_TIMER0->CC[0] = MS_DELAY_100;											//The period for desync algorithm
	NRF_TIMER0->CC[1] = 0;															//clear the RX capture time register
	NRF_TIMER0->INTENSET = 0x00010000;									//enable interrupt for compare0
	NVIC_EnableIRQ(TIMER0_IRQn);												//enable interrupt in NVIC
	NRF_TIMER0->TASKS_CLEAR = 1;												//clear the timer to 0 counts
	NRF_TIMER0->TASKS_START = 1;												//start the timer
}


void TIMER0_IRQHandler(void)
{
	NRF_TIMER0->TASKS_CLEAR = 1;												//clear the timer to 0 counts
	
  NRF_TIMER0->CC[0] = MS_DELAY_100;											//set the next TX period
	
	tx_ready_flag = 1;																	//ready to TX next time in main
			
	NRF_TIMER0->EVENTS_COMPARE[0] = 0;									//clear the compare flag
	
	NVIC_ClearPendingIRQ(TIMER0_IRQn);									//clear the interrupt in NVIC
}


int main (void)
{
		clock_initialization();  	//setup clocks
		setup_timer();						//sets up timer0 for transmit/receive intervals
		setup_gpio();							//sets up all gpio pins
		hal_radio_config();				//configure the radio parameters
		setup_beacon_message();			//
		while(1)
		{			
			if(tx_ready_flag)
			{
				BLE_transmit_data((uint8_t *)&iBeacon_message, (uint8_t)TX_DATA_SIZE);	//transmit data after delay
				tx_ready_flag = 0;
				toggle_led(ORNG_LED);
			}
		}
}

