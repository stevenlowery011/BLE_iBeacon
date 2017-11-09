#include "nrf.h"
#include "tx_data_handler.h"


/*function
* handles the transmitting of raw data, called from main
*/
void BLE_transmit_data(uint8_t *p_data, uint8_t data_length)
{
	//uint32_t temp;
	//temp = *p_data;
	
	//split data into 31 byte packets and add address, S0, len, S1 fields
	//tx_data_packets_pointer = tx_split_data(p_data, data_length);	
	
	disable_radio();			//Stop the Radio
	//tx_enable(tx_data_packets_pointer,(uint32_t)((data_length-1)/31+1));	//start the tx and wait for finish	
	tx_enable((uint32_t *)p_data,1);	//start the tx and wait for finish	
	disable_radio();			//tx disable
	//hal_radio_start_rx();//rx enable
	return;
}

