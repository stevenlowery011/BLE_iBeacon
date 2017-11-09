//function declarations

void setup_gpio(void);
void toggle_led(uint32_t );
void led_off(uint32_t LED);


//configure all the ding ding pins
//2066 of nrf51_bitfields.h for types
#define DINGDING_RXDPIN 	26														//uart rxd
#define RXD_DIR						GPIO_PIN_CNF_DIR_Input
#define RXD_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define RXD_PUL						GPIO_PIN_CNF_PULL_Disabled
#define RXD_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define RXD_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define DINGDING_TXDPIN 	27														//uart txd
#define TXD_DIR						GPIO_PIN_CNF_DIR_Output
#define TXD_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define TXD_PUL						GPIO_PIN_CNF_PULL_Disabled
#define TXD_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define TXD_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define DINGDING_SD_SCK0 	12														//SPI0 SCK
#define SCK0_DIR						GPIO_PIN_CNF_DIR_Output
#define SCK0_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define SCK0_PUL						GPIO_PIN_CNF_PULL_Pullup
#define SCK0_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define SCK0_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define DINGDING_MOSI 		15													//SPI0 MOSI
#define MOSI_DIR						GPIO_PIN_CNF_DIR_Output
#define MOSI_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define MOSI_PUL						GPIO_PIN_CNF_PULL_Pullup
#define MOSI_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define MOSI_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define DINGDING_MISO 		17													//SPI0_MiSO
#define MISO_DIR						GPIO_PIN_CNF_DIR_Input
#define MISO_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define MISO_PUL						GPIO_PIN_CNF_PULL_Disabled
#define MISO_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define MISO_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define DINGDING_SDCS 		16													// SPI0 Cable select
#define SDCS_DIR						GPIO_PIN_CNF_DIR_Output
#define SDCS_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define SDCS_PUL						GPIO_PIN_CNF_PULL_Pullup
#define SDCS_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define SDCS_SENSE					GPIO_PIN_CNF_SENSE_Disabled


#define BLUE_LED				 	5													// SPI0 Cable select
#define BLED_DIR						GPIO_PIN_CNF_DIR_Output
#define BLED_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define BLED_PUL						GPIO_PIN_CNF_PULL_Disabled
#define BLED_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define BLED_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define GREE_LED				 	6													// SPI0 Cable select
#define GLED_DIR						GPIO_PIN_CNF_DIR_Output
#define GLED_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define GLED_PUL						GPIO_PIN_CNF_PULL_Disabled
#define GLED_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define GLED_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define RED_LED				 		8													// SPI0 Cable select
#define RLED_DIR						GPIO_PIN_CNF_DIR_Output
#define RLED_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define RLED_PUL						GPIO_PIN_CNF_PULL_Disabled
#define RLED_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define RLED_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define ORNG_LED				 	7													// SPI0 Cable select
#define OLED_DIR						GPIO_PIN_CNF_DIR_Output
#define OLED_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define OLED_PUL						GPIO_PIN_CNF_PULL_Disabled
#define OLED_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define OLED_SENSE					GPIO_PIN_CNF_SENSE_Disabled



#define ANA01				 			30													// ain1
#define ANA01_DIR						GPIO_PIN_CNF_DIR_Input
#define ANA01_BUF						GPIO_PIN_CNF_INPUT_Connect
#define ANA01_PUL						GPIO_PIN_CNF_PULL_Disabled
#define ANA01_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define ANA01_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define ANA02				 			31													//ain 2
#define ANA02_DIR						GPIO_PIN_CNF_DIR_Input
#define ANA02_BUF						GPIO_PIN_CNF_INPUT_Connect
#define ANA02_PUL						GPIO_PIN_CNF_PULL_Disabled
#define ANA02_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define ANA02_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define I2CC				 			13													//ain 2
#define I2CC_DIR						GPIO_PIN_CNF_DIR_Output
#define I2CC_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define I2CC_PUL						GPIO_PIN_CNF_PULL_Disabled
#define I2CC_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define I2CC_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define I2CD				 			14											//ain 2
#define I2CD_DIR						GPIO_PIN_CNF_DIR_Output
#define I2CD_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define I2CD_PUL						GPIO_PIN_CNF_PULL_Disabled
#define I2CD_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define I2CD_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define BRES				 			12											//BNO reset pin
#define BRES_DIR						GPIO_PIN_CNF_DIR_Output
#define BRES_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define BRES_PUL						GPIO_PIN_CNF_PULL_Disabled 
#define BRES_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define BRES_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define STPA				 				18											//ain 2
#define STPA_DIR						GPIO_PIN_CNF_DIR_Output
#define STPA_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define STPA_PUL						GPIO_PIN_CNF_PULL_Disabled 
#define STPA_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define STPA_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define STPa				 				19											//ain 2
#define STPa_DIR						GPIO_PIN_CNF_DIR_Output
#define STPa_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define STPa_PUL						GPIO_PIN_CNF_PULL_Disabled 
#define STPa_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define STPa_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define STPB				 				20  										//ain 2
#define STPB_DIR						GPIO_PIN_CNF_DIR_Output
#define STPB_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define STPB_PUL						GPIO_PIN_CNF_PULL_Disabled 
#define STPB_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define STPB_SENSE					GPIO_PIN_CNF_SENSE_Disabled

#define STPb				 				11  										//ain 2
#define STPb_DIR						GPIO_PIN_CNF_DIR_Output
#define STPb_BUF						GPIO_PIN_CNF_INPUT_Disconnect
#define STPb_PUL						GPIO_PIN_CNF_PULL_Disabled 
#define STPb_DRIVE					GPIO_PIN_CNF_DRIVE_S0S1
#define STPb_SENSE					GPIO_PIN_CNF_SENSE_Disabled


//MAKE A CONFIG WORD FOR THE PIN CFG REGISTERS
#define DINGDING_SDCS_CFG 			(SDCS_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( SDCS_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( SDCS_PUL << GPIO_PIN_CNF_PULL_Pos )| ( SDCS_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( SDCS_SENSE << GPIO_PIN_CNF_SENSE_Pos )

#define BLUE_LED_CFG						(BLED_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( BLED_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( BLED_PUL << GPIO_PIN_CNF_PULL_Pos )| ( BLED_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( BLED_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define GREE_LED_CFG						(GLED_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( GLED_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( GLED_PUL << GPIO_PIN_CNF_PULL_Pos )| ( GLED_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( GLED_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define RED_LED_CFG							(RLED_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( RLED_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( RLED_PUL << GPIO_PIN_CNF_PULL_Pos )| ( RLED_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( RLED_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define ORNG_LED_CFG						(OLED_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( OLED_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( OLED_PUL << GPIO_PIN_CNF_PULL_Pos )| ( OLED_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( OLED_SENSE << GPIO_PIN_CNF_SENSE_Pos )


#define DINGDING_MISO_CFG				(MISO_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( MISO_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( MISO_PUL << GPIO_PIN_CNF_PULL_Pos )| ( MISO_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( MISO_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define DINGDING_MOSI_CFG				(MOSI_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( MOSI_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( MOSI_PUL << GPIO_PIN_CNF_PULL_Pos )| ( MOSI_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( MOSI_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define DINGDING_SD_SCK0_CFG		(SCK0_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( SCK0_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( SCK0_PUL << GPIO_PIN_CNF_PULL_Pos )| ( SCK0_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( SCK0_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define DINGDING_TXDPIN_CFG			(TXD_DIR << GPIO_PIN_CNF_DIR_Pos )  | ( TXD_BUF << GPIO_PIN_CNF_INPUT_Pos)  | ( TXD_PUL << GPIO_PIN_CNF_PULL_Pos ) | ( TXD_DRIVE << GPIO_PIN_CNF_DRIVE_Pos)  | ( TXD_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define DINGDING_RXDPIN_CFG			(RXD_DIR << GPIO_PIN_CNF_DIR_Pos )  | ( RXD_BUF << GPIO_PIN_CNF_INPUT_Pos)  | ( RXD_PUL << GPIO_PIN_CNF_PULL_Pos ) | ( RXD_DRIVE << GPIO_PIN_CNF_DRIVE_Pos)  | ( RXD_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define PIN8_CFG								(PIN8_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( PIN8_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( PIN8_PUL << GPIO_PIN_CNF_PULL_Pos )| ( PIN8_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( PIN8_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define ANA01_CFG								(ANA01_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( ANA01_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( ANA01_PUL << GPIO_PIN_CNF_PULL_Pos )| ( ANA01_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( ANA01_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define ANA02_CFG								(ANA02_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( ANA02_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( ANA02_PUL << GPIO_PIN_CNF_PULL_Pos )| ( ANA02_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( ANA02_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define I2CC_CFG								(I2CC_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( I2CC_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( I2CC_PUL << GPIO_PIN_CNF_PULL_Pos )| ( I2CC_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( I2CC_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define I2CD_CFG								(I2CD_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( I2CD_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( I2CD_PUL << GPIO_PIN_CNF_PULL_Pos )| ( I2CD_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( I2CD_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define BRES_CFG								(BRES_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( BRES_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( BRES_PUL << GPIO_PIN_CNF_PULL_Pos )| ( BRES_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( BRES_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define STPA_CFG								(STPA_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( STPA_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( STPA_PUL << GPIO_PIN_CNF_PULL_Pos )| ( STPA_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( STPA_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define STPa_CFG								(STPa_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( STPa_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( STPa_PUL << GPIO_PIN_CNF_PULL_Pos )| ( STPa_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( STPa_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define STPB_CFG								(STPB_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( STPB_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( STPB_PUL << GPIO_PIN_CNF_PULL_Pos )| ( STPB_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( STPB_SENSE << GPIO_PIN_CNF_SENSE_Pos )
#define STPb_CFG								(STPb_DIR << GPIO_PIN_CNF_DIR_Pos ) | ( STPb_BUF << GPIO_PIN_CNF_INPUT_Pos) | ( STPb_PUL << GPIO_PIN_CNF_PULL_Pos )| ( STPb_DRIVE << GPIO_PIN_CNF_DRIVE_Pos) | ( STPb_SENSE << GPIO_PIN_CNF_SENSE_Pos )

