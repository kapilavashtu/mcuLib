#include "system.h"
#include "usart.h"
#include <stdio.h>

//system irq handlers
uint16_t errors = 0;
uint32_t ram_ptr = 0;

void handlerTrapMesg(void)
{
	if (errors != 0) {

		printf("irq handlers code:%d\n", errors);
		errors = 0;
	}
}

void SVC_Handler(void)
{
	asm("TST LR, #0x4");	    // check bit[2] of EXC_RETURN val into a LR
	asm("ITE EQ");		    // if eq to zero, then use main stack, else uses process stack
	asm("MRSEQ R0, MSP");	    // and move are current stack pointer to RO
	asm("MRSNE R0, PSP");	    //
	asm("LDR R1, [R0, #24]");   // read from from stack saved value (command code number)
	asm("LDRB R0, [R1, # -2]"); // save code val number to R0
	    //end
}

void NMI_Handler(void) { printf("non masked interrupt\n"); }

void HardFault_Handler(void)
{
	//printf("ram_ptr add = %d\n", ram_ptr);
	if (SCB->HFSR & SCB_HFSR_VECTTBL) {
		printf("hard fault interrupt:vector table read on exception processing\n");
		SCB->HFSR &= ~SCB_HFSR_VECTTBL;
	}

	else if (SCB->HFSR & SCB_HFSR_FORCED) {
		printf("hard fault interrupt:Hard Fault activated when a configurable "
		       "Fault was received and cannot activate\n");
		SCB->HFSR &= ~SCB_HFSR_FORCED;
	}

	else if (SCB->HFSR & SCB_HFSR_DEBUGEVT) {
		printf("hard fault interrupt:related to debug\n");
		SCB->HFSR &= ~SCB_HFSR_DEBUGEVT;
	}
}

void MemManage_Handler(void)
{
	printf("memory manage fault int:\n");
}

void BusFault_Handler(void) { printf("bus fault\n"); }

void UsageFault_Handler(void) { printf("usage fault\n"); }

void Init_Peripheral()
{
	//PORT A clock enable
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

        //PORT B clock enable
	//RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	//PORT D clock enable
	//RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;

	//PORT E clock enable
	//RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;

	//PORT G clock enable
	//RCC->APB2ENR |= RCC_APB2ENR_IOPGEN;

	//Alternate Function I/O clock enable
	//RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	//Alternative Function I/O clock enable
	//RCC->APB1ENR |= RCC_APB1ENR_AFIOEN;

	//Alternate Function I/O clock enable
	//RCC->APB1ENR |= RCC_APB1ENR_AFIOEN;

	//enable lsi rc (WDGTMR)
	//RCC->CSR |= RCC_CSR_LSION;

	//External interrupt configuration register 1 (AFIO_EXTICR1) - pe2 ext2 interrupt enable
	//AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PE;

	//Interrupt mask register - pe2 ext2 interrupt enable
	//EXTI->IMR |= AFIO_EXTICR1_EXTI0_PE;

	//Falling trigger selection register (EXTI_FTSR) - falling on 2 pin port e
	//EXTI->FTSR |= EXTI_FTSR_TR2;

	//GPIO LED PIN(GPIO PUSH/PULL 50 MHZ)
	//GPIOB->CRL = (uint32_t) 0;
	//GPIOB->CRL = (uint32_t) 0x22222222;
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

	//PORTS SETUP
	//GPIOE->CRL = 0x11111111;
	//GPIOE->CRH = 0x11111111;

	//PG8 INPUT PULLUP
	//GPIOG->CRH |= GPIO_CRH_CNF8_1;
	//GPIOG->ODR |= GPIO_ODR_ODR8;

	//GPIOB->CRH &= ~(GPIO_CRH_CNF13_0 | GPIO_CRH_CNF15_0 | GPIO_CRH_CNF10_0);
	//GPIOB->CRH |= GPIO_CRH_CNF13_1 | GPIO_CRH_CNF15_1;
	//GPIOB->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_MODE13_1 | GPIO_CRH_MODE15_1;

	//GPIOE PIN2 (as input pullup/pulldown)
	//! GPIOE->CRL = GPIO_CRL_CNF2_1;
	//! GPIOE->ODR = 0x0004; // PORTE2 is HIGH

	//GPIOE->CRL |= GPIO_CRL_MODE5;
	//FLASH_init();
	//DMA Init
}

void Init_IWDG(uint32_t prescaler, uint32_t count_val)
{
	//unlock registers
	IWDG->KR = 0X5555;

	//set prescaler value
	IWDG->PR = prescaler;

	//load reload val
	IWDG->RLR = count_val * 40 / 256;

	//reset timer
	IWDG->KR = 0xAAAA;
	IWDG->KR = 0xCCCC;
}

//INDEPENDENCE WATCHDOG TIMER
void IWDG_Reset(void)
{
	IWDG->KR = 0xAAAA;
}

//FLASH OPERATION
void FLASH_init(void)
{
	FLASH->ACR |= FLASH_ACR_LATENCY_2;
}

void FLASH_unlock(void)
{
	FLASH->KEYR = KEY1;
	FLASH->KEYR = KEY2;
}

void FLASH_lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}

int16_t FLASH_write(uint16_t val, uint32_t address)
{
	int16_t x = 0;
	if (FLASH->CR & FLASH_CR_LOCK)
		return -1;
	if (FLASH->SR & FLASH_SR_BSY)
		return 0;
	FLASH->CR |= FLASH_CR_PG;
	*(__IO uint16_t *)(address) = val;

	while (FLASH->SR & FLASH_SR_BSY)
		;
	x = *(__IO uint16_t *)(address);
	if (x != val)
		return 0;
	if (FLASH->SR & FLASH_SR_PGERR)
		USART1_SendString("FLASH write error!\r\n");
	return x;
}

int16_t FLASH_check_page(uint32_t page_address)
{
	//check as words
	uint32_t counter;
	for (counter = 0; counter < PAGE_SIZE; counter++) {
		if (!(*(__IO uint32_t *)(page_address + counter)))
			return -1;
	}

	return 0;
}

int16_t FLASH_page_erase(uint32_t page)
{
	if (FLASH->CR & FLASH_CR_LOCK)
		return -1;
	if (FLASH->SR & FLASH_SR_BSY)
		return 0;

	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = page;
	FLASH->CR |= FLASH_CR_STRT;

	while (FLASH->SR & FLASH_SR_BSY)
		;

	//change on select address from array
	if (FLASH_check_page(PAGE254))
		return -10;
	return 0;
}

int16_t save_data(void)
{
	//check entry
	uint16_t key = 0x12CF;
	uint16_t cx = *(__IO uint16_t *)(PAGE254);
	USART1_SendInt(cx);
	USART1_SendString("\r\n");
	if (cx == key) {
		USART1_SendString("entry exist = !\r\n");
		USART1_SendInt(cx);
		USART1_SendString("\r\n");
		return 0;
	}

	//check page
	FLASH_unlock();
	int16_t z = FLASH_check_page(PAGE254);
	if (z) {
		USART1_SendString("Page not clear!\r\n");

		z = FLASH_page_erase(PAGE254);

		if (z == -1)
			USART1_SendString("FLASH locked!\r\n");
		else if (z == -10)
			USART1_SendString("FLASH bad erasing!\r\n");
		else if (!z)
			USART1_SendString("FLASH erased!\r\n");
	}

	z = FLASH_write(key, PAGE254);
	if (z)
		USART1_SendString("FLASH write ok!\r\n");
	else
		USART1_SendString("FLASH write bad!\r\n");
	FLASH_lock();
	return 0;
}

//OPTION BYTES
/*
uint8_t OPTBYTE_write(uint16_t val)
{
  //check flash busy flag
  if(FLASH->SR & FLASH_SR_BSY) return 0;
  //unlock OPT
  FLASH->OPTKEYR = KEY1;
  FLASH->OPTKEYR = KEY2;
  if(!(FLASH->CR & FLASH_CR_OPTWRE)) return 0;
 
  FLASH->CR |= FLASH_CR_OPTER;
  FLASH->CR |= FLASH_CR_STRT;
  while(FLASH->SR & FLASH_SR_BSY);
  //clear erase flag
  FLASH->CR &= ~FLASH_CR_OPTER;
 
  FLASH->CR |= FLASH_CR_OPTPG;
  //write data
  //
  *(uint16_t*)(0x1FFFF804) = (val >> 8) & 0xFF;
  //waint for BSY
  while(FLASH->SR & FLASH_SR_BSY);
  *(uint16_t*)(0x1FFFF806) = val & 0xFF;
  //waint for BSY
  while(FLASH->SR & FLASH_SR_BSY);
 
  FLASH->CR &= ~FLASH_CR_OPTPG;
 
  FLASH->CR = FLASH_CR_LOCK;
  return 0;
}
*/

uint8_t OPTBYTE_write(uint16_t data1, uint16_t data0)
{
	//check flash busy flag
	if (FLASH->SR & FLASH_SR_BSY)
		return 0;
	//unlock OPT
	FLASH->OPTKEYR = KEY1;
	FLASH->OPTKEYR = KEY2;
	if (!(FLASH->CR & FLASH_CR_OPTWRE))
		return 0;
	//set OPTER
	FLASH->CR |= FLASH_CR_OPTER;
	//set STRT
	FLASH->CR |= FLASH_CR_STRT;
	//waint for BSY
	while (FLASH->SR & FLASH_SR_BSY)
		;
	FLASH->CR &= ~FLASH_CR_OPTER;

	//set OPTPG to FLASH_CR
	FLASH->CR |= FLASH_CR_OPTPG;
	//write data
	*(uint16_t *)(0x1FFFF804) = data1 & 0x00FF;
	//waint for BSY
	while (FLASH->SR & FLASH_SR_BSY)
		;
	*(uint16_t *)(0x1FFFF806) = data0 & 0x00FF;
	//waint for BSY
	while (FLASH->SR & FLASH_SR_BSY)
		;
	FLASH->CR &= ~FLASH_CR_OPTPG;
	FLASH->CR = FLASH_CR_LOCK;

	return 0;
}

int16_t OPTBYTE_erase(void)
{
	//unlock FPEC
	FLASH_unlock();
	//check flash busy flag
	if (FLASH->SR & FLASH_SR_BSY)
		return -1;
	//unlock OPTWRE in FLASH_CR
	FLASH->CR |= FLASH_CR_OPTWRE;
	//set OPTER
	FLASH->CR |= FLASH_CR_OPTER;
	//set STRT
	FLASH->CR |= FLASH_CR_STRT;
	//waint for BSY
	while (FLASH->SR & FLASH_SR_BSY)
		;
	//check erase errors
	return 0;
}

uint8_t rewrite_option_bytes(const OB_TypeDef *new_values)
{
	FLASH->OPTKEYR = 0x45670123;
	FLASH->OPTKEYR = 0xCDEF89AB;

	USART1_SendString("OPTBYTES unlocking !\r\n");
	if (!(FLASH->CR & FLASH_CR_OPTWRE))
		return 0;
	USART1_SendString("OPTBYTES unlock !\r\n");
	FLASH->CR |= FLASH_CR_OPTER;
	FLASH->CR |= FLASH_CR_STRT;
	while (FLASH->SR & FLASH_SR_BSY)
		;
	USART1_SendString("OPTBYTES erase !\r\n");
	uint16_t const *pNew = (uint16_t const *)new_values;
	uint16_t *pOld = (uint16_t *)OB;
	for (uint_fast8_t i = 0; i < sizeof(*OB) / sizeof(uint16_t); ++i) {
		if ((*pNew & 0xFF) != (*pOld & 0xFF)) {
			FLASH->CR = FLASH_CR_OPTPG;
			*pOld = *pNew;
			while (FLASH->SR & FLASH_SR_BSY)
				;
			USART1_SendString("OPTBYTES write entry!\r\n");
			FLASH->CR = 0;
			if ((*pNew & 0xFF) != (*pOld & 0xFF))
				return 0;
		}
		pNew++;
		pOld++;
	}
	return 1;
}

//=======================================================================
void usart_status(uint16_t dev)
{
	switch (dev) {
	case 1:
		xprintf("USART%d system status: control register 1\n", dev);

		if (USART1->CR1 & USART_CR1_UE)
			xprintf("\tUSART: enable\n");
		else
			xprintf("\tUSART: disable\n");

		if (USART1->CR1 & USART_CR1_M)
			xprintf("\t1 start bit, 8 data bit\n");
		else
			xprintf("\t1 start bit, 8 data bit\n");

		if (USART1->CR1 & USART_CR1_WAKE)
			xprintf("\twakeup method: address mark\n");
		else
			xprintf("\twakeup method: idle line\n");

		if (USART1->CR1 & USART_CR1_PCE)
			xprintf("\tparity control:enable\n");
		else
			xprintf("\tparity control:disable\n");

		if (USART1->CR1 & USART_CR1_PS)
			xprintf("\tparity: odd\n");
		else
			xprintf("\tparity: even\n");

		if (USART1->CR1 & USART_CR1_PEIE)
			xprintf("\tPE interrupt: enable\n");
		else
			xprintf("\tPE interrupt: disable\n");

		if (USART1->CR1 & USART_CR1_TXEIE)
			xprintf("\tTXE interrupt: enable\n");
		else
			xprintf("\tTXE interrupt: disable\n");

		if (USART1->CR1 & USART_CR1_TCIE)
			xprintf("\ttransmission complete interrupt: enable\n");
		else
			xprintf("\ttransmission complete interrupt: disable\n");

		if (USART1->CR1 & USART_CR1_RXNEIE)
			xprintf("\tRXNE interrupt: enable\n");
		else
			xprintf("\tRXNE interrupt: disable\n");

		if (USART1->CR1 & USART_CR1_IDLEIE)
			xprintf("\tIDLE interrupt: enable\n");
		else
			xprintf("\tIDLE interrupt: disable\n");

		if (USART1->CR1 & USART_CR1_TE)
			xprintf("\ttransmitter: enable\n");
		else
			xprintf("\ttransmitter: disable\n");

		if (USART1->CR1 & USART_CR1_RE)
			xprintf("\treciever: enable\n");
		else
			xprintf("\treciever: disable\n");

		if (USART1->CR1 & USART_CR1_RWU)
			xprintf("\treciever wakeup: mute mode\n");
		else
			xprintf("\treciever wakeup: active mode\n");

		if (USART1->CR1 & USART_CR1_SBK)
			xprintf("\tbreak char will be transmitted\n");
		else
			xprintf("\tno break char is transmitted\n");

		break;

	case 2:
		xprintf("USART%d system status: control register 1\n", dev);

		if (USART2->CR1 & USART_CR1_UE)
			xprintf("\tUSART: enable\n");
		else
			xprintf("\tUSART: disable\n");

		if (USART2->CR1 & USART_CR1_M)
			xprintf("\t1 start bit, 8 data bit\n");
		else
			xprintf("\t1 start bit, 8 data bit\n");

		if (USART2->CR1 & USART_CR1_WAKE)
			xprintf("\twakeup method: address mark\n");
		else
			xprintf("\twakeup method: idle line\n");

		if (USART2->CR1 & USART_CR1_PCE)
			xprintf("\tparity control:enable\n");
		else
			xprintf("\tparity control:disable\n");

		if (USART2->CR1 & USART_CR1_PS)
			xprintf("\tparity: odd\n");
		else
			xprintf("\tparity: even\n");

		if (USART2->CR1 & USART_CR1_PEIE)
			xprintf("\tPE interrupt: enable\n");
		else
			xprintf("\tPE interrupt: disable\n");

		if (USART2->CR1 & USART_CR1_TXEIE)
			xprintf("\tTXE interrupt: enable\n");
		else
			xprintf("\tTXE interrupt: disable\n");

		if (USART2->CR1 & USART_CR1_TCIE)
			xprintf("\ttransmission complete interrupt: enable\n");
		else
			xprintf("\ttransmission complete interrupt: disable\n");

		if (USART2->CR1 & USART_CR1_RXNEIE)
			xprintf("\tRXNE interrupt: enable\n");
		else
			xprintf("\tRXNE interrupt: disable\n");

		if (USART2->CR1 & USART_CR1_IDLEIE)
			xprintf("\tIDLE interrupt: enable\n");
		else
			xprintf("\tIDLE interrupt: disable\n");

		if (USART2->CR1 & USART_CR1_TE)
			xprintf("\ttransmitter: enable\n");
		else
			xprintf("\ttransmitter: disable\n");

		if (USART2->CR1 & USART_CR1_RE)
			xprintf("\treciever: enable\n");
		else
			xprintf("\treciever: disable\n");

		if (USART2->CR1 & USART_CR1_RWU)
			xprintf("\treciever wakeup: mute mode\n");
		else
			xprintf("\treciever wakeup: active mode\n");

		if (USART2->CR1 & USART_CR1_SBK)
			xprintf("\tbreak char will be transmitted\n");
		else
			xprintf("\tno break char is transmitted\n");
		break;
	}
}

void spi_status(uint16_t dev)
{
	switch (dev) {
	case 1:
		xprintf("SPI%d system status: control register 1\n", dev);

		if (SPI1->CR1 & SPI_CR1_SPE)
			xprintf("\tSPI: enable\n");
		else
			xprintf("\tSPI: disable\n");

		if (SPI1->CR1 & SPI_CR1_LSBFIRST)
			xprintf("\tLSB trasmitted first\n");
		else
			xprintf("\tMSB transmitted first\n");

		if (SPI1->CR1 & SPI_CR1_CPHA)
			xprintf("\tCPHA: 1\n");
		else
			xprintf("\tCPHA: 0\n");

		if (SPI1->CR1 & SPI_CR1_CPOL)
			xprintf("\tCPOL: 1\n");
		else
			xprintf("\tCPOL: 0\n");

		if (SPI1->CR1 & SPI_CR1_MSTR)
			xprintf("\tMODE: master\n");
		else
			xprintf("\tMODE: slave\n");

		if (SPI1->CR1 & SPI_CR1_BR)
			;
		//xprintf("\tTXE interrupt: enable\n");s
		else
			;
		//xprintf("\tTXE interrupt: disable\n");

		if (SPI1->CR1 & SPI_CR1_SSM)
			xprintf("\tsoftware slave management: enable\n");
		else
			xprintf("\tsoftware slave management: disable\n");

		if (SPI1->CR1 & SPI_CR1_RXONLY)
			xprintf("\recieve only: output disabled (rx only)\n");
		else
			xprintf("\trecieve only: full duplex (tx + rx)\n");

		if (SPI1->CR1 & SPI_CR1_DFF)
			xprintf("\tdata frame format: 16 bit\n");
		else
			xprintf("\tdata frame format: 8 bit\n");

		if (SPI1->CR1 & SPI_CR1_CRCNEXT)
			xprintf("\tCRC transfer next: next transfer is CRC (CRC phase)\n");
		else
			xprintf("\tCRC transfer next: data phase (no CRC phase)\n");

		if (SPI1->CR1 & SPI_CR1_CRCEN)
			xprintf("\tCRC calculation: enable\n");
		else
			xprintf("\tCRC calculation: disable\n");

		if (SPI1->CR1 & SPI_CR1_BIDIOE)
			xprintf("\toutput enable in bidirectional mode: output enabled (TX only)\n");
		else
			xprintf("\toutput enable in bidirectional mode: output disabled (RX only)\n");

		if (SPI1->CR1 & SPI_CR1_BIDIMODE)
			xprintf("\tbidirectional data mode enable: 1-line mode\n");
		else
			xprintf("\tbidirectional data mode enable: 2-line mode\n");

		break;

	case 2:
		xprintf("SPI%d system status: control register 1\n", dev);

		if (SPI2->CR1 & SPI_CR1_SPE)
			xprintf("\tSPI: enable\n");
		else
			xprintf("\tSPI: disable\n");

		if (SPI2->CR1 & SPI_CR1_LSBFIRST)
			xprintf("\tLSB trasmitted first\n");
		else
			xprintf("\tMSB transmitted first\n");

		if (SPI2->CR1 & SPI_CR1_CPHA)
			xprintf("\tCPHA: 1\n");
		else
			xprintf("\tCPHA: 0\n");

		if (SPI2->CR1 & SPI_CR1_CPOL)
			xprintf("\tCPOL: 1\n");
		else
			xprintf("\tCPOL: 0\n");

		if (SPI2->CR1 & SPI_CR1_MSTR)
			xprintf("\tMODE: master\n");
		else
			xprintf("\tMODE: slave\n");

		if (SPI2->CR1 & SPI_CR1_BR)
			;
		//xprintf("\tTXE interrupt: enable\n");s
		else
			;
		//xprintf("\tTXE interrupt: disable\n");

		if (SPI2->CR1 & SPI_CR1_SSM)
			xprintf("\tsoftware slave management: enable\n");
		else
			xprintf("\tsoftware slave management: disable\n");

		if (SPI2->CR1 & SPI_CR1_RXONLY)
			xprintf("\recieve only: output disabled (rx only)\n");
		else
			xprintf("\trecieve only: full duplex (tx + rx)\n");

		if (SPI2->CR1 & SPI_CR1_DFF)
			xprintf("\tdata frame format: 16 bit\n");
		else
			xprintf("\tdata frame format: 8 bit\n");

		if (SPI2->CR1 & SPI_CR1_CRCNEXT)
			xprintf("\tCRC transfer next: next transfer is CRC (CRC phase)\n");
		else
			xprintf("\tCRC transfer next: data phase (no CRC phase)\n");

		if (SPI2->CR1 & SPI_CR1_CRCEN)
			xprintf("\tCRC calculation: enable\n");
		else
			xprintf("\tCRC calculation: disable\n");

		if (SPI2->CR1 & SPI_CR1_BIDIOE)
			xprintf("\toutput enable in bidirectional mode: output enabled (TX only)\n");
		else
			xprintf("\toutput enable in bidirectional mode: output disabled (RX only)\n");

		if (SPI2->CR1 & SPI_CR1_BIDIMODE)
			xprintf("\tbidirectional data mode enable: 1-line mode\n");
		else
			xprintf("\tbidirectional data mode enable: 2-line mode\n");

		break;
	}
}

void tim_status(uint16_t dev);
void adc_status(uint16_t dev);
void dac_status(uint16_t dev);
void gpio_status(uint16_t dev);
void rtc_status(void);
void bkup_reg_status(uint16_t dev);
void sdio_status(void);
void iwdg_status(void);
void wwdg_status(void);
void fsmc_status(void);
void usb_status(void);
void irq_status(uint16_t dev);