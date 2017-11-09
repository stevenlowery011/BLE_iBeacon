#include "hal_radio.h"

/*globals*/
uint32_t tx_packets_remaining = 0;	//global variable: how many packets left to transmit
uint8_t tx_finished = 0;						//flag for when transmit has finished
uint8_t sync_tx_flag = 0;						//flag sets after sync packet TX, clears after sync packet RX
enum{DISABLED,TX_MODE,RX_MODE};
uint8_t g_radio_state = DISABLED;
int32_t g_phi1 = 0;
int32_t g_phi2 = 0;
int32_t g_phiMid = 0;
int32_t g_deltaPhi = 0;

#define ALPHA 0.5

//the radio should operate in rcv mode all the time, 
//switch to tx mode and tx when ready, then go pack to rcv mode
//interrupt on packet rcv and store in ring buffer

#define RADIO_BUFSIZE	20

uint32_t packet[RADIO_BUFSIZE][11];              /**< Packet to transmit. */ //setup buffer of 128
uint32_t rxcrc[RADIO_BUFSIZE];              /**< Packet to transmit. */ //setup buffer of 128
uint8_t rssi[RADIO_BUFSIZE];              /**< Packet to transmit. */ //setup buffer of 128
uint32_t packet_pointer = 0;  //not to exceed RADIO_BUFSIZE
uint32_t radio_buf_flip=0;  // is on when packet goes over 128, turns off when last_printed goes 128.

uint8_t access_address[4] = {0xD6, 0xBE, 0x89, 0x8E};
uint8_t seed[3] = {0x55, 0x55, 0x55};


/**@brief The maximum possible length in device discovery mode. */
#define DD_MAX_PAYLOAD_LENGTH         (31 + 6)


/**@brief The default CRC init polynominal. 

   @note Written in little endian but stored in big endian, because the BLE spec. prints
         is in little endian but the HW stores it in big endian. */
#define CRC_POLYNOMIAL_INIT_SETTINGS  ((0x5B << 0) | (0x06 << 8) | (0x00 << 16))


/**@brief This macro converts the given channel index to a freuency
          offset (i.e. offset from 2400 MHz).

  @param index - the channel index to be converted into frequency offset.
  
  @return The frequency offset for the given index. */
#define CHANNEL_IDX_TO_FREQ_OFFS(index) \
    (((index) == 37)?\
        (2)\
        :\
            (((index) == 38)?\
                (26)\
            :\
                (((index) == 39)?\
                    (80)\
                :\
                    ((/*((index) >= 0) &&*/ ((index) <= 10))?\
                        ((index)*2 + 4)\
                    :\
                        ((index)*2 + 6)))))
                        

void hal_radio_channel_index_set(uint8_t channel_index)
{
    NRF_RADIO->FREQUENCY = CHANNEL_IDX_TO_FREQ_OFFS(channel_index);
    NRF_RADIO->DATAWHITEIV = channel_index;
}


//base configuration for  DING DING radio
void hal_radio_config(void)
{

    NVIC_DisableIRQ(RADIO_IRQn);

		// cycle radio power
    NRF_RADIO->POWER = RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos;
    NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos;

	  NRF_RADIO->TXPOWER=0x00; //set lowest tx power
	
		//RADIO has ramped up and is ready to be started, then start radio
		//packet sent or recieved, then disable radio
  //  NRF_RADIO->SHORTS = DEFAULT_RADIO_SHORTS;
	
		NRF_RADIO->SHORTS = 0x000000000;
	
    //length on air of S0, S1, and LFLEN
    NRF_RADIO->PCNF0 =   (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk)
                       | (((2UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk)
                       | (((6UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk);

    //little endian, base address length (3 + 1 = 4), static length = 0, max length, enable payload	
    NRF_RADIO->PCNF1 =   (((RADIO_PCNF1_ENDIAN_Little)        << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk)
                       | (((3UL)                              << RADIO_PCNF1_BALEN_Pos)  & RADIO_PCNF1_BALEN_Msk)
                       | (((0UL)                              << RADIO_PCNF1_STATLEN_Pos)& RADIO_PCNF1_STATLEN_Msk)
                       | ((((uint32_t)DD_MAX_PAYLOAD_LENGTH)  << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk)
                       | ((RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk);
    
    /* The CRC polynomial is fixed, and is set here. */
    /* The CRC initial value may change, and is set by */
    /* higher level modules as needed. */
    NRF_RADIO->CRCPOLY = (uint32_t)CRC_POLYNOMIAL_INIT_SETTINGS;
    NRF_RADIO->CRCCNF = (((RADIO_CRCCNF_SKIPADDR_Skip) << RADIO_CRCCNF_SKIPADDR_Pos) & RADIO_CRCCNF_SKIPADDR_Msk)
                      | (((RADIO_CRCCNF_LEN_Three)      << RADIO_CRCCNF_LEN_Pos)       & RADIO_CRCCNF_LEN_Msk);

		//enable reception on logical address, what does this mean?
    NRF_RADIO->RXADDRESSES  = ( (RADIO_RXADDRESSES_ADDR0_Enabled) << RADIO_RXADDRESSES_ADDR0_Pos);

		//set BLE mode
    NRF_RADIO->MODE    = ((RADIO_MODE_MODE_Ble_1Mbit) << RADIO_MODE_MODE_Pos) & RADIO_MODE_MODE_Msk;

		//interframe space in us between end tx and begin tx
    NRF_RADIO->TIFS = 150;

		//something with access address?
    NRF_RADIO->PREFIX0 = access_address[3];
    NRF_RADIO->BASE0   = ( (((uint32_t)access_address[2]) << 24) 
                         | (((uint32_t)access_address[1]) << 16)
                         | (((uint32_t)access_address[0]) << 8) );

		//Initial value for CRC calculation.
    NRF_RADIO->CRCINIT = ((uint32_t)seed[0]) | ((uint32_t)seed[1])<<8 | ((uint32_t)seed[2])<<16;
		
		NRF_RADIO->DATAWHITEIV=37;
		
		NVIC_EnableIRQ(RADIO_IRQn);
}

//setup interrupts and such per user application
void hal_radio_start_rx(void)
{	
		g_radio_state = RX_MODE;
		hal_radio_set_rcv_mode();
	
		NRF_RADIO->INTENSET=0x00000008; //on event end
		NRF_RADIO->PACKETPTR = (uint32_t)&(packet[0][0]);
    NRF_RADIO->TASKS_START = 1;
}
 
uint8_t tx_enable(uint32_t *p_tx_frame_start, uint32_t numPackets)
{
	tx_finished = 0;
	g_radio_state = TX_MODE;
	//radio should be configured at this point.  Power it up. 
	NRF_RADIO->EVENTS_READY = 0U; 			//clear the ready flad
  // Enable radio and wait for ready
	NRF_RADIO->EVENTS_END = 0U; 				//clear flag
	NRF_RADIO->INTENSET=0x00000008; 		//on event end
  NRF_RADIO->PACKETPTR = (uint32_t)p_tx_frame_start;
  NRF_RADIO->TASKS_TXEN = 1U;
  do{}while (NRF_RADIO->EVENTS_READY == 0U);//wait
  NRF_RADIO->EVENTS_END = 0U; 				//radio ready, clear flag
	NRF_RADIO->TASKS_START = 1;
	while(!tx_finished){}								//wait for tx to finish
  return 1;
}

//function to set in receive mode
//not finished
void hal_radio_set_rcv_mode(void)
{
		//radio should be configured at this point.  Power it up. 
		NRF_RADIO->EVENTS_READY = 0U; //clear the ready flad
    // Enable radio and wait for ready
    NRF_RADIO->TASKS_RXEN = 1U;
    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END = 0U; //radio ready, clear flag
}

/*function
* disables radio from receive mode
*/
void disable_radio(void)
{
	NRF_RADIO->TASKS_DISABLE = 1UL;
	do{/*wait*/}while(!NRF_RADIO->EVENTS_DISABLED);
	g_radio_state = DISABLED;
}

//function to send packet
//not finished
void hal_radio_send(uint8_t *p_data)
{
		//update data
    NRF_RADIO->PACKETPTR = (uint32_t)&(p_data[0]);
		//clear event
    NRF_RADIO->EVENTS_END = 0;
		//enable radio in tx mode
    NRF_RADIO->TASKS_START = 1;
}


//handler for radio interrupt. 
//update buffer pointer, restart rcv with new buffer pointer
void RADIO_IRQHandler(void)
{
	struct MSG_SYNC * thismsg;
	uint8_t check = 0;
	
	if(g_radio_state == RX_MODE)
	{
		thismsg = (struct MSG_SYNC * )&packet[packet_pointer][0];
		if (NRF_RADIO->CRCSTATUS==1)
		{
			rxcrc[packet_pointer]=NRF_RADIO->RXCRC; // grab checksum
			
			if (thismsg->Msg_hdr.Adv != SWARM_ADV_TYPE)check++;
			if (thismsg->Msg_hdr.Length != SWARM_LENGTH)check++;
			if (thismsg->Msg_hdr.Rfu1 != 0)check++;
			if (thismsg->Msg_hdr.Rfu2 != 0)check++;
			if (thismsg->Msg_hdr.NID != SWARM_ID)check++;
			if (thismsg->Msg_hdr.MID != SYNC)check++;
			
			if(check==0)														//if this is a sync packet
			{
				if(sync_tx_flag)											//first sync RX after a sync TX
				{
					NRF_TIMER0->TASKS_CAPTURE[2] = 1;	  //capture the timer value to CC[2] at first sync packet RX after a sync TX
					if((NRF_TIMER0->CC[2])>(NRF_TIMER0->CC[0]))
					{
						NRF_TIMER0->TASKS_CLEAR = 1;		  //if the timer has overrun the capture period, reset the timer counts
					}
					g_phi1 = NRF_TIMER0->CC[2];					//calculate phi1: difference between first RX and last TX
					NRF_TIMER0->TASKS_CAPTURE[1] = 1;		//capture the timer value to CC[1] at every sync packet RX
					g_phiMid = (g_phi1 + g_phi2)>>1;		//average of phi1 and phi2 is the midpoint of the two events adjacent to last TX
					g_deltaPhi = g_phi2 - g_phiMid;			//the distance from the midpoint is delta phi
					if( ! ( ( (int32_t)(g_deltaPhi) > (int32_t)(NRF_TIMER0->CC[0]) ) | ( (int32_t)(-g_deltaPhi) > (int32_t)(NRF_TIMER0->CC[0]) ) ) )
					{
						NRF_TIMER0->CC[0] = (uint32_t)(NRF_TIMER0->CC[0] - ALPHA*(g_deltaPhi));
					}
					sync_tx_flag = 0;									  //reset the sync transmit flag when receiving sync packet
				}
				else																	//not the first sync RX after a sync TX
				{
					NRF_TIMER0->TASKS_CAPTURE[1] = 1;		//capture the timer value to CC[1] at every sync packet RX
					if((NRF_TIMER0->CC[1])>(NRF_TIMER0->CC[0]))
					{
						NRF_TIMER0->TASKS_CLEAR = 1;		  //if the timer has overrun the capture period, reset the timer counts
					}
				}
			}			
		}
		else
		{
			rxcrc[packet_pointer]=0xFFFFFFFF; 
		}
		
		//update buffer pointer
		packet_pointer++;
		if (packet_pointer>=RADIO_BUFSIZE)
		{
			packet_pointer=0;
			radio_buf_flip = 1; //packet pointer has flipped made a loop on circ buffer
		}
		NRF_RADIO->PACKETPTR=(uint32_t)&packet[packet_pointer][0];
	}
	
	else if(g_radio_state == TX_MODE)
	{
		thismsg = (struct MSG_SYNC * )(NRF_RADIO->PACKETPTR);
		if (thismsg->Msg_hdr.Adv != SWARM_ADV_TYPE)check++;
		if (thismsg->Msg_hdr.Length != SWARM_LENGTH)check++;
		if (thismsg->Msg_hdr.Rfu1 != 0)check++;
		if (thismsg->Msg_hdr.Rfu2 != 0)check++;
		if (thismsg->Msg_hdr.NID != SWARM_ID)check++;
		if (thismsg->Msg_hdr.MID != SYNC)check++;
		
		if(check==0)														//if this is a sync packet
		{
			sync_tx_flag = 1;											//set the sync transmit flag
		}
		tx_finished = 1;
	}
	
	
	//clear the event
	NRF_RADIO->EVENTS_END=0;
	
	//restart
	NRF_RADIO->TASKS_START=1; //restart
	
	NVIC_ClearPendingIRQ(RADIO_IRQn);
}
