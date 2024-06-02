/*
 * NRF24L01+.c
 *
 *      Credit/Source: https://www.youtube.com/watch?v=mB7LsiscM78&ab_channel=ControllersTech
 */
#include "stm32f3xx_hal.h"
#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi3; //Define SPI handler
#define NRF24_SPI &hspi3

#define NRF24_CE_PORT GPIOB
#define NRF24_CE_PIN  GPIO_PIN_7

#define NRF24_CSN_PORT GPIOB
#define NRF24_CSN_PIN  GPIO_PIN_6

void CSN_Select (void) //Makes CSN(active LOW) 0 and selects the device
{
	 HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN,0);
}

void CSN_UnSelect (void)//Makes CSN(active LOW) 1 and unselects the device
{
	 HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN,1);
}

void CE_Enable (void) //Makes CE 1 and Enables the device
{
	 HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN,1);
}

void CE_Disable (void) //Makes CE 0 and Disables the device
{
	 HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN,0);
}

void nrf24_WriteReg (uint8_t Reg, uint8_t Data) // Write 1 byte to the register
{
	uint8_t buf[2]; // Buffer that stores 2 bytes
	buf[0] = Reg|1<<5; //First Byte is Register address
	buf[1] = Data; //Second Byte is the Data
	// Select the device
	CSN_Select();

	HAL_SPI_Transmit(NRF24_SPI,buf,2,1000); // Send buffer with size 2

	// Release the device
	CSN_UnSelect();

}

void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *Data, int size) // Write many bytes starting from a particular register
{
	uint8_t buf[1]; // Buffer that stores 1 byte
	buf[0] = Reg|1<<5; //Byte is Register address

	// Select the device
	CSN_Select();

	HAL_SPI_Transmit(NRF24_SPI,buf,1,100); // Send Register address
	HAL_SPI_Transmit(NRF24_SPI,Data,size,100); //Send all data bytes at ones

	// Release the device
	CSN_UnSelect();

}

uint8_t nrf24_ReadReg (uint8_t Reg) // Read 1 byte
{
	uint8_t data=0; // Variable to store data
	// Select the device
	CSN_Select();

	HAL_SPI_Transmit(NRF24_SPI,&Reg,1,100); // Send register address where we want to read data from
	HAL_SPI_Receive(NRF24_SPI,&data,1,100); // Read 1 byte from register

	// Release the device
	CSN_UnSelect();

	return data;
}

void nrf24_ReadRegMulti (uint8_t Reg,uint8_t *data,int size) // Read multiple bytes starting from register pointed to variable where you want to store data
{
	// Select the device
	CSN_Select();

	HAL_SPI_Transmit(NRF24_SPI,&Reg,1,100); // Send register address where we want to read data from
	HAL_SPI_Receive(NRF24_SPI,data,size,1000); // Read X bytes from register

	// Release the device
	CSN_UnSelect();
}

//Send command to the NRF24
void nrfsendCmd(uint8_t cmd)
{
	// Select the device
	CSN_Select();

	HAL_SPI_Transmit(NRF24_SPI,&cmd,1,100);

	// Release the device
	CSN_UnSelect();
}

void nrf24_reset(uint8_t REG)
{
	if (REG == STATUS)
	{
		nrf24_WriteReg(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		nrf24_WriteReg(FIFO_STATUS, 0x11);
	}

	else {
	nrf24_WriteReg(CONFIG, 0x08);
	nrf24_WriteReg(EN_AA, 0x3F);
	nrf24_WriteReg(EN_RXADDR, 0x03);
	nrf24_WriteReg(SETUP_AW, 0x03);
	nrf24_WriteReg(SETUP_RETR, 0x03);
	nrf24_WriteReg(RF_CH, 0x02);
	nrf24_WriteReg(RF_SETUP, 0x0E);
	nrf24_WriteReg(STATUS, 0x00);
	nrf24_WriteReg(OBSERVE_TX, 0x00);
	nrf24_WriteReg(CD, 0x00);
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	nrf24_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
	nrf24_WriteReg(RX_ADDR_P2, 0xC3);
	nrf24_WriteReg(RX_ADDR_P3, 0xC4);
	nrf24_WriteReg(RX_ADDR_P4, 0xC5);
	nrf24_WriteReg(RX_ADDR_P5, 0xC6);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
	nrf24_WriteReg(RX_PW_P0, 0);
	nrf24_WriteReg(RX_PW_P1, 0);
	nrf24_WriteReg(RX_PW_P2, 0);
	nrf24_WriteReg(RX_PW_P3, 0);
	nrf24_WriteReg(RX_PW_P4, 0);
	nrf24_WriteReg(RX_PW_P5, 0);
	nrf24_WriteReg(FIFO_STATUS, 0x11);
	nrf24_WriteReg(DYNPD, 0);
	nrf24_WriteReg(FEATURE, 0);
	}
}

void NRF24_Init (void)
{
	//Disable chip before config
	CE_Disable();


	nrf24_reset(0);

	nrf24_WriteReg(CONFIG,0); // Will be configured later

	nrf24_WriteReg(EN_AA,0); // No Auto Ack

	nrf24_WriteReg(EN_RXADDR,0); // Not enabling any data pipe right now

	nrf24_WriteReg(SETUP_AW,0x03); // 5 bytes for the TX/RX address

	nrf24_WriteReg(SETUP_RETR,0); // No retransmision (no shockburst)

	nrf24_WriteReg(RF_CH,0); // setup during Tx or Rx

	nrf24_WriteReg(RF_SETUP,0x0E); // Power = 0db, Data Rate = 2Mbps

	//Disable chip before config
	CE_Enable();
}

//setup TX mode
void NRF24_TXMode(uint8_t *address, uint8_t channel) // adress of receiver pipe and channel number
{
	//Disable chip before config
	CE_Disable();

	nrf24_WriteReg(RF_CH,channel); // Select the channel

	nrf24_WriteRegMulti(TX_ADDR,address,5); // Write the TX adress 5 bytes;

	//Power up device without changing other bits of register
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config|1<<1;
	nrf24_WriteReg(CONFIG,config);

	//Disable chip before config
	CE_Enable();
}

//transmit data
uint8_t NRF24_Transmit (uint8_t *data)
{
	uint8_t cmdtosend = 0;

	// Select the device
	CSN_Select();

	//Payload command
	cmdtosend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI,&cmdtosend,1,100);

	// send the payload
	HAL_SPI_Transmit(NRF24_SPI,data,32,1000);

	// Select the device
	CSN_UnSelect();

	HAL_Delay(1);

	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	if ((fifostatus&(1<<4)) && (!(fifostatus&(1<<3))))
	{
		cmdtosend = FLUSH_TX;
		nrfsendCmd(cmdtosend);
		nrf24_reset(FIFO_STATUS);
		return 1;
	}

	return 0;
}

void NRF24_RXMode(uint8_t *address,uint8_t channel)
{
	//Disable chip before config
	CE_Disable();

	nrf24_WriteReg(RF_CH,channel); // Select the channel

	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
	en_rxaddr = en_rxaddr|(1<<1); // first make sure no other pipes get disabled.
	nrf24_WriteReg(EN_RXADDR,en_rxaddr); // select data pipe 1
	nrf24_WriteRegMulti(RX_ADDR_P1,address,5); // 5 byte address for data pipe1

	nrf24_WriteReg(RX_PW_P1,32); // 32 byte payload size for pipe1

	//Power up device without changing other bits of register
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config|(1<<1) | (1<<0);
	nrf24_WriteReg(CONFIG,config);

	//Disable chip before config
	CE_Enable();
}

uint8_t isDataAvailable(int pipenum)
{
	uint8_t status = nrf24_ReadReg(STATUS);

	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{
		nrf24_WriteReg(STATUS,(1<<6));
		return 1;
	}
	return 0;
}

void NRF24_Receive (uint8_t *data)
{
	uint8_t cmdtosend = 0;

	// Select the device
	CSN_Select();

	//Payload command
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI,&cmdtosend,1,100);

	// send the payload
	HAL_SPI_Receive(NRF24_SPI,data,19,1000);

	// Select the device
	CSN_UnSelect();

	HAL_Delay(1);

	cmdtosend = FLUSH_RX;
	nrfsendCmd(cmdtosend);
}























