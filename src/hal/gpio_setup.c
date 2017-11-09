#include "nrf.h"
#include "gpio_setup.h"

void setup_gpio(void)
{
	//LEDS
	NRF_P0->PIN_CNF[BLUE_LED]=BLUE_LED_CFG;
	NRF_P0->PIN_CNF[RED_LED]=RED_LED_CFG;
	NRF_P0->PIN_CNF[GREE_LED]=GREE_LED_CFG;
	NRF_P0->PIN_CNF[ORNG_LED]=ORNG_LED_CFG;
}


void toggle_led(uint32_t LED)
{
	uint32_t scratch;
	scratch = (NRF_GPIO->OUT & (0x1<< LED));
	if (scratch == (0x1<< LED))
	{
			NRF_GPIO->OUTCLR= (1<< LED); //Turn LED on
	}
	else
	{
			NRF_GPIO->OUTSET= (1<< LED);
	}
}

void led_off(uint32_t LED)
{
	NRF_GPIO->OUTSET= (1<< LED);
}
