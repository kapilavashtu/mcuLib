#include "usart.h"
#include "__libc.h"
#include "system.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

//interrupt handlers

//get buffer memory with malloc

uint8_t u1_init_state = 0;
uint8_t u2_init_state = 0;

buffer u1buff;
buffer u2buff;

uint8_t rx_buff[RX_BUF_SIZE];

void USART1_IRQHandler(void)
{
	printf("usart1 irq\n");
	if ((USART1->SR & USART_SR_RXNE) != (uint16_t)RESET) {
		errors = U1_RX_DATA;

		//if USART1 is init
		if (u1_init_state) {

			__disable_irq();
			if (USART1->DR == '\r') {
				toBuffer(&u1buff, USART1->DR);
				toBuffer(&u1buff, '\n');

			} else
				toBuffer(&u1buff, USART1->DR);
			__enable_irq();
		}
	} else if ((USART1->SR & USART_SR_ORE) != (uint16_t)RESET) {
		errors = U1_OVERRUN;
	} else if ((USART1->SR & USART_SR_FE) != (uint16_t)RESET) {
		errors = U1_FRAME_ERR;
	} else if ((USART1->SR & USART_SR_PE) != (uint16_t)RESET) {
		errors = U1_PARITY_ERR;
	}
}

void USART2_IRQHandler(void)
{
	if ((USART2->SR & USART_SR_RXNE) != (uint16_t)RESET) {
		errors = U2_RX_DATA;
	} else if ((USART2->SR & USART_SR_ORE) != (uint16_t)RESET) {
		errors = U2_OVERRUN;
	} else if ((USART2->SR & USART_SR_FE) != (uint16_t)RESET) {
		errors = U2_FRAME_ERR;
	} else if ((USART2->SR & USART_SR_PE) != (uint16_t)RESET) {
		errors = U2_PARITY_ERR;
	}
}

void USART_Init(uint16_t num, uint16_t baudrate, uint16_t modeset)
{
	switch (num) {
	case 1:
		//USART1 clock enable
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

		//USART1 GPIO SETUP
		//GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1 | GPIO_CRH_CNF10_0;

		GPIOA->CRH &= ~GPIO_CRH_CNF9;	// Clear CNF bit 9
		GPIOA->CRH |= GPIO_CRH_CNF9_1;	// Set CNF bit 9 to 10 - AFIO Push-Pull
		GPIOA->CRH |= GPIO_CRH_MODE9_0; // Set MODE bit 9 to Mode 01 = 10MHz

		GPIOA->CRH &= ~GPIO_CRH_CNF10;	// Clear CNF bit 9
		GPIOA->CRH |= GPIO_CRH_CNF10_0; // Set CNF bit 9 to 01 = HiZ
		GPIOA->CRH &= ~GPIO_CRH_MODE10; // Set MODE bit 9 to Mode 01 = 10MHz

		//USART1 BAUDRATE = 19200
		USART1->BRR = 0xEA6;
		USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

		//rx buffer init

		u1buff.mask = 0xff;
		u1buff.idxIN = 0x00;
		u1buff.idxOUT = 0x00;
		u1buff.buffer = rx_buff;
		//u1buff.buffer = malloc(RX_BUF_SIZE);
		u1_init_state = 1;
		break;

	case 2:
		//USART2 clock enable
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

		//USART2 GPIO SETUP
		//GPIOA->CRL |= GPIO_CRL_MODE2 | GPIO_CRL_CNF2_1 | GPIO_CRL_CNF3_0;

		// USART2 Settings
		// SET TX2
		GPIOA->CRL &= ~GPIO_CRL_CNF2;	// Clear CNF bit 2
		GPIOA->CRL |= GPIO_CRL_CNF2_1;	// Set CNF bit 2 to 10 - AFIO Push-Pull
		GPIOA->CRL |= GPIO_CRL_MODE2_0; // Set MODE bit 2 to Mode 01 = 10MHz

		// SET RX2
		GPIOA->CRL &= ~GPIO_CRL_CNF3;  // Clear CNF bit 3
		GPIOA->CRL |= GPIO_CRL_CNF3_0; // Set CNF bit 3 to 01 HiZ
		GPIOA->CRL &= ~GPIO_CRL_MODE3; // Set MODE bit 3 to Mode 01 = 10MHz

		//USART2 BAUDRATE = 19200
		//USART2->BRR = 0x753;
		USART2->BRR = 0x271; //57600
		USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

		//rx buffer init

		u2buff.mask = 0xff;
		u2buff.idxIN = 0x00;
		u2buff.idxOUT = 0x00;
		u2buff.buffer = rx_buff;
		//u2buff.buffer = malloc(RX_BUF_SIZE);
		u2_init_state = 1;
		break;

	default:
		break;
	}
}

//USART1
void USART1_SendString(char string[])
{

	uint32_t n = 0;
	while (string[n] != '\0') {
		USART1_SendChar(string[n]);
		n++;
	}
}

void USART1_SendStringEnd(char string[])
{
	uint32_t n = 0;
	while (string[n] != '\0') {
		USART1_SendChar(string[n]);
		n++;
	}
	USART1_SendChar('\r');
	USART1_SendChar('\n');
}

void USART1_SendChar(uint8_t data)
{
	while (!(USART1->SR & USART_SR_TC))
		;
	USART1->DR = data;
}

void USART1_SendInt(uint32_t data)
{
	if (data < 0) {
		USART1_SendChar('-');
		data = -data;
	}
	if (data / 10) {
		USART1_SendInt(data / 10);
	}
	USART1_SendChar(data % 10 + '0');
}

void USART1_SendNewLine()
{
	USART1_SendChar('\r');
	USART1_SendChar('\n');
}

void Rx_Save1(char *buffer, uint32_t *val)
{
	if ((USART1->SR & USART_SR_RXNE) != (uint16_t)RESET) {
		buffer[*val] = USART1->DR;
		*val++;
	}
}

//USART2
void USART2_SendString(char string[])
{
	uint32_t n = 0;
	while (string[n] != '\0') {
		USART2_SendChar(string[n]);
		n++;
	}
}

void USART2_SendStringEnd(char string[])
{
	uint32_t n = 0;
	while (string[n] != '\0') {
		USART2_SendChar(string[n]);
		n++;
	}
	USART2_SendChar('\r');
	USART2_SendChar('\n');
}

void USART2_SendChar(uint8_t data)
{
	while (!(USART2->SR & USART_SR_TC))
		;
	USART2->DR = data;
}

void USART2_SendInt(uint32_t data)
{
	if (data < 0) {
		USART2_SendChar('-');
		data = -data;
	}
	if (data / 10) {
		USART2_SendInt(data / 10);
	}
	USART2_SendChar(data % 10 + '0');
}

void USART2_SendNewLine()
{
	USART2_SendChar('\r');
	USART2_SendChar('\n');
}

void Rx_Save2(char *buffer, uint32_t *val)
{
	if ((USART2->SR & USART_SR_RXNE) != (uint16_t)RESET) {
		buffer[*val] = USART2->DR;
		*val++;
	}
}

//utils
uint16_t getBufLen(buffer *buf)
{
	if (buf->idxIN >= buf->idxOUT)
		return buf->idxIN - buf->idxOUT;
	else
		return ((RX_BUF_SIZE - buf->idxOUT) + buf->idxIN);
}

void clearBuffer(buffer *buf)
{
	buf->idxIN = buf->idxOUT;
}

uint8_t emptyBuffer(buffer *buf)
{
	if (buf->idxIN == buf->idxOUT)
		return 1;
	else
		return 0;
}

void toBuffer(buffer *buf, uint8_t data)
{
	buf->buffer[buf->idxIN++] = data;
	if (buf->mask != 0xff)
		buf->idxIN &= buf->mask;
}

uint8_t fromBuffer(buffer *buf)
{
	uint8_t res = buf->buffer[buf->idxOUT++];
	if (buf->mask != 0xff)
		buf->idxOUT &= buf->mask;
	return res;
}

int8_t closeBuffer(buffer *buf)
{
	if (buf->buffer != NULL) {
		//if d.allocate
		free(buf->buffer);
		return 0;
	} else
		return -1;
}

//serial stream library function
uint16_t available(void)
{
	return getBufLen(&u1buff);
}

uint8_t read(void)
{
	return fromBuffer(&u1buff);
}

//stdio realisation
int xprintf(const char *fmt, ...)
{
	char buffer[128];
	va_list args;
	va_start(args, fmt);
	int n = vsnprintf(buffer, sizeof(buffer), fmt, args);
	//SEGGER_RTT_Write(0, buffer, n);
	write(USART1, (uint8_t *)&buffer, n);
	va_end(args);
	return n;
}

int16_t write(USART_TypeDef *port, uint8_t *src, uint16_t len)
{
	uint8_t c = 0;
	while (c < len) {
		while (!(port->SR & USART_SR_TC))
			;
		port->DR = src[c++];
	}
	return c;
}

int xgetchar(void)
{
	if (emptyBuffer(&u1buff))
		return -1;
	else
		return (int)fromBuffer(&u1buff);
}

uint8_t getCurINidx(void)
{
	return u1buff.idxIN;
}

uint8_t getCurOUTidx(void)
{
	return u1buff.idxOUT;
}