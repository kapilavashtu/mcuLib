#include "spi.h"
#include <stdio.h>

void SPI1_IRQHandler(void) { printf("spi\n"); }

void SPI1_Init(void)
{
	SPI1->CR1 = 0x0000;
	SPI1->CR2 = 0x0000;

	//PORT A clock enable & Alternate Function I/O clock enable
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

	//SPI1 PIN SETUP(clear work fields)
	GPIOA->CRL &= ~((uint32_t) 0xFFFF0000);

        //NSS is HIGH
        GPIOA->CRL |= 0x00001000;

	//SCK (as push/pull alternative)
	GPIOA->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1;

	//MISO (as input float)
	GPIOA->CRL |= GPIO_CRL_CNF6_0;

	//MOSI (as push/pull alternative)
	GPIOA->CRL |= GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;

	//SPI1 clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	//set baudrate
	SPI1->CR1 &= ~(SPI_CR1_BR);
	SPI1->CR1 |= FPCLK_256;

	//set polarity & phase clock
	SPI1->CR1 &= ~SPI_CR1_CPOL;
	SPI1->CR1 &= ~SPI_CR1_CPHA;

	//set data frame format 8 bit
	SPI1->CR1 &= ~(SPI_CR1_DFF);

	//set MSB first
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);

	//set Software slave management & Internal slave select
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;

	//set mode MASTER
	SPI1->CR1 |= SPI_CR1_MSTR;

	//set enable SPI1
	SPI1->CR1 |= SPI_CR1_SPE;
}

int16_t SPI_init(uint16_t spi_num, spi_cfg *config)
{
	if (spi_num == 1) {
		SPI1->CR1 = 0x0000;
		SPI1->CR2 = 0x0000;

		//PORT A clock enable & Alternate Function I/O clock enable
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

		//SPI1 PIN SETUP(clear work fields)
		GPIOA->CRL &= ~((uint32_t) 0xFFFF0000);

		//SCK (as push/pull alternative)
		GPIOA->CRL |= GPIO_CRL_MODE5_0 | GPIO_CRL_CNF5_1;

		//MISO (as input float)
		GPIOA->CRL |= GPIO_CRL_CNF7_0;

		//MOSI (as push/pull alternative)
		GPIOA->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_1;

		//SPI1 clock enable
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

		SPI1->CR1 &= ~(SPI_CR1_BR);
		//SPI1->CR1 |= clock;

	} else {
		SPI2->CR1 = 0x0000;
		SPI2->CR2 = 0x0000;

		//SPI2 PIN SETUP
		GPIOB->CRH = 0x00000000;
		GPIOD->CRL = 0x00000000;

		//PORT B clock enable &
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

		//SPI2 clock enable
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	}
	SPI_set_baudrate(spi_num, config->clock);
	SPI_set_type(spi_num, config->spi_type);
	SPI_set_frame_format(spi_num, config->frame_format);
	SPI_set_ending(spi_num, config->ending);
	SPI_set_irq(spi_num, config->irq_en);
	SPI_set_role(spi_num, config->role);
	SPI_set_enable(spi_num, 1);

	//reset register
	if (spi_num == 1) {
		//SPI1 PIN SETUP(clear work fields)
		GPIOA->CRL &= ~((uint32_t) 0xFFFF0000);

		//SCK (as push/pull alternative)
		GPIOA->CRL |= GPIO_CRL_MODE5_0 | GPIO_CRL_CNF5_1;

		//MISO (as input float)
		GPIOA->CRL |= GPIO_CRL_CNF7_0;

		//MOSI (as push/pull alternative)
		GPIOA->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_1;

	} else {
		//SCK (as push/pull alternative)
		GPIOB->CRH |= GPIO_CRH_MODE13_1 | GPIO_CRH_CNF13_1;

		//MISO (as input float)
		GPIOB->CRH |= GPIO_CRH_MODE14_1 | GPIO_CRH_CNF14_1;

		//MOSI (as push/pull alternative)
		GPIOB->CRH |= GPIO_CRH_MODE15_1 | GPIO_CRH_CNF15_1;

		//CS (as push/pull) FREQ=10MHZ
		GPIOD->CRL |= GPIO_CRL_MODE2;

		//CHIP ENABLE (GPIO PUSH/PULL 10 MHZ)
		GPIOA->CRL |= GPIO_CRL_MODE4_0;

		//CHIP SELECT (GPIO PUSH/PULL 50 MHZ)
		GPIOA->CRL |= GPIO_CRL_MODE7;
	}

	return 0;
}

int16_t SPI_set_baudrate(uint16_t spi_num, uint16_t clock)
{
	switch (spi_num) {
	case 1:
		if (!clock)
			SPI1->CR1 &= ~(SPI_CR1_BR); //reset baudrate bits
		else {
			SPI1->CR1 &= ~(SPI_CR1_BR);
			SPI1->CR1 |= clock;
		}
		break;

	case 2:
		if (!clock)
			SPI2->CR1 &= ~(SPI_CR1_BR);
		else {
			SPI2->CR1 &= ~(SPI_CR1_BR);
			SPI2->CR1 |= clock;
		}
		break;
	}
	return 0;
}

//This bit should not be changed when communication is ongoing.
int16_t SPI_set_ending(uint16_t spi_num, uint16_t ending)
{
	switch (spi_num) {
	case 1:
		if (!ending)
			SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);
		else
			SPI1->CR1 |= SPI_CR1_LSBFIRST;
		break;

	case 2:
		if (!ending)
			SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);
		else
			SPI2->CR1 |= SPI_CR1_LSBFIRST;
		break;
	}
	return 0;
}

int16_t SPI_set_role(uint16_t spi_num, uint16_t role)
{
	switch (spi_num) {
	case 1:
		if (!role)
			SPI1->CR1 &= ~(SPI_CR1_MSTR);
		else {
			SPI1->CR1 |= SPI_CR1_MSTR;
			SPI1->CR2 |= SPI_CR2_SSOE;
		}
		break;

	case 2:
		if (!role)
			SPI2->CR1 &= ~(SPI_CR1_MSTR);
		else {
			SPI2->CR1 |= SPI_CR1_MSTR;
			SPI2->CR2 |= SPI_CR2_SSOE;
		}
		break;
	}
	return 0;
}

int16_t SPI_set_frame_format(uint16_t spi_num, uint16_t format)
{
	switch (spi_num) {
	case 1:
		if (!format)
			SPI1->CR1 &= ~(SPI_CR1_DFF);
		else
			SPI1->CR1 |= SPI_CR1_DFF;
		break;

	case 2:
		if (!format)
			SPI2->CR1 &= ~(SPI_CR1_DFF);
		else
			SPI2->CR1 |= SPI_CR1_DFF;
		break;
	}
	return 0;
}

int16_t SPI_set_enable(uint16_t spi_num, uint16_t state)
{
	switch (spi_num) {
	case 1:

		if (!state)
			SPI1->CR1 &= ~(SPI_CR1_SPE);
		else
			SPI1->CR1 |= SPI_CR1_SPE;

		break;

	case 2:

		if (!state)
			SPI2->CR1 &= ~(SPI_CR1_SPE);
		else
			SPI2->CR1 |= SPI_CR1_SPE;

		break;
	}
	return 0;
}

int16_t SPI_set_type(uint16_t spi_num, uint16_t type)
{
	switch (spi_num) {
	case 1:
		SPI1->CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);
		switch (type) {
		case 1:

			SPI1->CR1 |= SPI_CR1_CPHA;
			break;

		case 2:

			SPI1->CR1 |= SPI_CR1_CPOL;
			break;

		case 3:

			SPI1->CR1 |= SPI_CR1_CPHA;
			SPI1->CR1 |= SPI_CR1_CPOL;
			break;
		}
		break;

	case 2:
		SPI2->CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);
		switch (type) {

		case 1:
			SPI1->CR1 |= SPI_CR1_CPHA;
			break;

		case 2:
			SPI1->CR1 |= SPI_CR1_CPOL;
			break;

		case 3:
			SPI1->CR1 |= SPI_CR1_CPHA;
			SPI1->CR1 |= SPI_CR1_CPOL;
			break;
		}
		break;
	}
	return 0;
}

int16_t SPI_set_irq(uint16_t spi_num, uint16_t irq)
{
	switch (spi_num) {
	case 1:
		if (!irq)
			SPI1->CR1 &= ~(SPI_CR2_ERRIE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
		else
			SPI1->CR1 |= irq;
		break;

	case 2:
		if (!irq)
			SPI2->CR1 &= ~(SPI_CR2_ERRIE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
		else
			SPI2->CR1 |= irq;
		break;
	}
	return 0;
}

int16_t SPI_read(uint16_t spi_num, uint8_t *dest, uint16_t count, uint16_t flags)
{
	if (flags & BLK_OP)
		__ASM volatile("cpsid i"
			       :
			       :
			       : "memory");

	switch (spi_num) {
	case 1:
		CS_LOW;
		for (uint16_t i = 0; i < count; i++) {
			while (!(SPI1->SR & SPI_SR_TXE))
				;
			SPI1->DR = 0xFF;
			while (!(SPI1->SR & SPI_SR_TXE))
				;
			dest[i] = SPI1->DR;
		}
		CS_HIGH;
		break;

	case 2:
		CS_LOW;
		for (uint16_t i = 0; i < count; i++) {
			while (!(SPI2->SR & SPI_SR_TXE))
				;
			SPI2->DR = 0xFF;
			while (!(SPI2->SR & SPI_SR_TXE))
				;
			dest[i] = SPI2->DR;
		}
		CS_HIGH;

		break;
	}

	if (flags & BLK_OP)
		__ASM volatile("cpsie i"
			       :
			       :
			       : "memory");
}

uint16_t SPI_write(uint16_t spi_num, uint8_t data)
{
	switch (spi_num) {
	case 1:
		while (!(SPI1->SR & SPI_SR_TXE))
			;
		CS_LOW;
		SPI1->DR = data & 0xFF;
		while (!(SPI1->SR & SPI_SR_TXE))
			;
		data = SPI1->DR;
		CS_HIGH;
		return data;
		break;

	case 2:
		while (!(SPI2->SR & SPI_SR_TXE))
			;
		CS_LOW;
		SPI2->DR = data & 0xFF;
		while (!(SPI2->SR & SPI_SR_TXE))
			;
		data = SPI2->DR;
		CS_HIGH;
		return data;
		break;
	}
}

int16_t SPI_write_block(uint16_t spi_num, uint8_t *block, uint16_t count, uint16_t flags)
{
	if (flags & BLK_OP)
		__ASM volatile("cpsid i"
			       :
			       :
			       : "memory");
	switch (spi_num) {
	case 1:
		CS_LOW;
		for (uint16_t i = 0; i < count; i++) {
			while (!(SPI1->SR & SPI_SR_TXE))
				;
			SPI1->DR = block[i] & 0xFF;
		}
		CS_HIGH;
		break;

	case 2:
		CS_LOW;
		for (uint16_t i = 0; i < count; i++) {
			while (!(SPI2->SR & SPI_SR_TXE))
				;
			SPI2->DR = block[i] & 0xFF;
		}
		CS_HIGH;

		break;
	}
	if (flags & BLK_OP)
		__ASM volatile("cpsie i"
			       :
			       :
			       : "memory");
}