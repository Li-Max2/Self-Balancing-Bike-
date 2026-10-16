/*
 * NRF24L01.c
 *
 *  Created on: Jul 29, 2026
 *      Author: maxim
 */

#include "stm32f4xx_hal.h"
#include "NRF24L01.h"

// Chip select and enable helper functions
void CS_Select(void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}

void CS_UnSelect(void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
}

void CE_Enable(void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

void CE_Disable(void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}

// Writing to a register
void NRF24_Write_Reg(uint8_t reg, uint8_t data){
	uint8_t buf[2]; //001A AAAA to write (A = reg) (AKA 0x20 FOR WRITE)
	buf[0] = reg | 1<<5;
	buf[1] = data;


	// pulling cs low to select device (since CSN)
	// first address, then data
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 100); // no similar fucntion to hal i2cmem read since SPI isnt a universally constant transmission

	// pull cs high to release device
	CS_UnSelect();
}

// Writing multiple bytes to a certain register
void NRF24_Write_Reg_Multi(uint8_t reg, const uint8_t *data, uint8_t size){
	uint8_t buf; //001A AAAA to write (A = reg) (AKA 0x20 FOR WRITE)
	buf = reg|1<<5;
	//buf[1] = Data;


	// pulling cs low to select device (since CSN)
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, &buf, 1, 100);
	HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);


	//pull cs high to release device
	CS_UnSelect();
}

// reading a register
uint8_t NRF24_Read_Reg(uint8_t reg){
	uint8_t data = 0; //000A AAAA to read (A = reg and no need to left shift to add a 1) (AKA 0x00 FOR READ)

	// pulling cs low to select device (since CSNOT)
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, &reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 100);
	// pull cs high to release device
	CS_UnSelect();
	return data;
}

// Sending a command to NRF
void NRF24_Send_Cmd(uint8_t cmd){

	// Pulling cs low to select device (since CSN)
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	// pull cs high to release device
	CS_UnSelect();
}

void NRF24_Init(void)
{
	//disable the chip during config
	CE_Disable();

	NRF24_reset(0);

	NRF24_Write_Reg(CONFIG, 0); //config reg
	NRF24_Write_Reg(EN_AA, 0); //No auto acknowledgment
	NRF24_Write_Reg(EN_RXADDR, 0); //No rx
	NRF24_Write_Reg(SETUP_AW, 0b11); //5 bytes for tx/rx address (ALL REG ADDRESSES ARE 5 bits)
	NRF24_Write_Reg(SETUP_RETR, 0); //no retransmission
	NRF24_Write_Reg(RF_CH, 0); // setup during tx/rx (channel)
	NRF24_Write_Reg(RF_SETUP, 0b1110); //Power = 0db, data rate = 2Mbps

	CE_Enable();
}

// setting up tx module
void NRF24_TxMode (uint8_t *Address, uint16_t channel){
	//disable the chip during config
	CE_Disable();

	NRF24_Write_Reg(RF_CH, channel); // selecting channel;
	NRF24_Write_Reg_Multi(TX_ADDR, Address, 5); //writing 5 byte TX address

	//powering up device through setting 1bit to 1 without changing other bits in config reg
	uint8_t config_bits = NRF24_Read_Reg(CONFIG);
	config_bits = (config_bits | (1<<1) & ~(1<<0)); //1st bit on for power up, 0 bit = 0 for tx mode
	NRF24_Write_Reg(CONFIG, config_bits);

	HAL_Delay(2);
	CE_Enable();
}

// setting up rx module
void NRF24_RxMode (uint8_t *Address, uint16_t channel){
	//disable the chip during config
	CE_Disable();

	NRF24_Write_Reg(RF_CH, channel); // slecting channel for rx;

	// Selecting 1 while preserving other pipes
	uint8_t enable_rxAddress = NRF24_Read_Reg(EN_RXADDR);
	enable_rxAddress = enable_rxAddress | (1<<1);

	NRF24_Write_Reg(EN_RXADDR, enable_rxAddress); // selecting data pipe (put 1 in whatever pipe you want ex. bit 1 pipe 1, bit 2 pipe 2, etc.)
	NRF24_Write_Reg_Multi(RX_ADDR_P1, Address, 5); //writing 5 byte RX address for recieving
	NRF24_Write_Reg(RX_PW_P1, 32); // 32 byte payload size for pipe 1 (can change)

	//powering up the device in RX mode
	uint8_t config_bits = NRF24_Read_Reg(CONFIG);
	config_bits = (config_bits | (1<<1) | (1<<0)); //1st bit on for power up, 0 bit = 1 for rx mode
	NRF24_Write_Reg(CONFIG, config_bits);

	CE_Enable();


}

// transmitting data
uint8_t NRF24_Transmit(uint8_t *data){

	uint8_t current_cmd = 0;

	// select device
	CS_Select();

	// SEND PAYLOAD CMD FIRST, THEN SEND 32 BIT PAYLOAD
	current_cmd = W_TX_PAYLOAD; // telling device we want to send payload thru payload cmd
	HAL_SPI_Transmit(NRF24_SPI, &current_cmd, 1, 100);

	//sending payload
	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

	CS_UnSelect();

	HAL_Delay(1);

	uint8_t fifostatus = NRF24_Read_Reg(FIFO_STATUS);

	if((fifostatus & (1<<4)) && (!(fifostatus & (1<<3)))){ //fifo status reg 4th bit is 1 when non empty, AND 3rd bit is 0 when connected

		current_cmd = FLUSH_TX; //if data transferred, flush the fifo
		NRF24_Send_Cmd(current_cmd);
		NRF24_reset(FIFO_STATUS);
		return 1;
	}

	return 0; //failed to send cmd
}

uint8_t isDataAvailable (int pipenum)
{
	uint8_t status = NRF24_Read_Reg(STATUS);

	if ((status&(1<<6)) && (status & (pipenum<<1))){
		NRF24_Write_Reg(STATUS, 1<<6); //clearing rx interrupt bit thru sending 1 (check datasheet)
		return 1;
	}
	return 0;
}

void NRF24_Receive(uint8_t *data){
	uint8_t current_cmd = 0;

	// select device
	CS_Select();

	// telling device we want to recieve payload thru recieve cmd
	current_cmd = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &current_cmd, 1, 100);

	//receiving payload
	HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

	// unselecting device
	CS_UnSelect();
	HAL_Delay(1);

	//flushing fifo
	current_cmd = FLUSH_RX;
	NRF24_Send_Cmd(FLUSH_RX);
}


// reset function in case things stop working (SOURCE: FROM controllers tech)
void NRF24_reset(uint8_t REG) // input 0 if u wanna reset everything, 0x07 for reg status, 0x17 for fifo status
{
	if (REG == STATUS)
	{
		NRF24_Write_Reg(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		NRF24_Write_Reg(FIFO_STATUS, 0x11);
	}

	else {
	NRF24_Write_Reg(CONFIG, 0x08);
	NRF24_Write_Reg(EN_AA, 0x3F);
	NRF24_Write_Reg(EN_RXADDR, 0x03);
	NRF24_Write_Reg(SETUP_AW, 0x03);
	NRF24_Write_Reg(SETUP_RETR, 0x03);
	NRF24_Write_Reg(RF_CH, 0x02);
	NRF24_Write_Reg(RF_SETUP, 0x0E);
	NRF24_Write_Reg(STATUS, 0x00);
	NRF24_Write_Reg(OBSERVE_TX, 0x00);
	NRF24_Write_Reg(CD, 0x00);
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	NRF24_Write_Reg_Multi(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	NRF24_Write_Reg_Multi(RX_ADDR_P1, rx_addr_p1_def, 5);
	NRF24_Write_Reg(RX_ADDR_P2, 0xC3);
	NRF24_Write_Reg(RX_ADDR_P3, 0xC4);
	NRF24_Write_Reg(RX_ADDR_P4, 0xC5);
	NRF24_Write_Reg(RX_ADDR_P5, 0xC6);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	NRF24_Write_Reg_Multi(TX_ADDR, tx_addr_def, 5);
	NRF24_Write_Reg(RX_PW_P0, 0);
	NRF24_Write_Reg(RX_PW_P1, 0);
	NRF24_Write_Reg(RX_PW_P2, 0);
	NRF24_Write_Reg(RX_PW_P3, 0);
	NRF24_Write_Reg(RX_PW_P4, 0);
	NRF24_Write_Reg(RX_PW_P5, 0);
	NRF24_Write_Reg(FIFO_STATUS, 0x11);
	NRF24_Write_Reg(DYNPD, 0);
	NRF24_Write_Reg(FEATURE, 0);
	}
}