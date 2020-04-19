#include "control_C.h"
#include "macros.h"
#include "stm32f10x.h"
/*
void Init_Peripheral()
{

  //CLOCK AND POWER SETUP
  
  //PORT A clock enable
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

  //PORT B clock enable
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  //PORT D clock enable
  RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;

  //PORT E clock enable
  RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;

  //PORT G clock enable
  RCC->APB2ENR |= RCC_APB2ENR_IOPGEN;

  //USART1 clock enable
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  //USART2 clock enable
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  //SPI1 clock enable
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  //Alternate Function I/O clock enable
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

  //SPI2 clock enable
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

  //Alternate Function I/O clock enable
  //RCC->APB1ENR |= RCC_APB1ENR_AFIOEN;

  //enable lsi rc (WDGTMR)
  RCC->CSR |= RCC_CSR_LSION;

  //PORTS SETUP
  //=============================================================== USART =============================================================
  //USART1 GPIO SETUP
  GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1 | GPIO_CRH_CNF10_0;

  //USART2 GPIO SETUP
  GPIOA->CRL |= GPIO_CRL_MODE2 | GPIO_CRL_CNF2_1 | GPIO_CRL_CNF3_0;

  
  //SPI1 PIN SETUP
  GPIOA->CRL = 0x00000000;

  //SCK (as push/pull alternative)
  GPIOA->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1;

  //MISO (as input float)
  GPIOA->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1;

  //MOSI (as push/pull alternative)
  GPIOA->CRL |= GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;

  //SPI2 PIN SETUP
  GPIOB->CRH = 0x00000000;
  GPIOD->CRL = 0x00000000;

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

  //PG8 INPUT PULLUP
  GPIOG->CRH |= GPIO_CRH_CNF8_1;
  GPIOG->ODR |= GPIO_ODR_ODR8;
  
  //GPIOB->CRH &= ~(GPIO_CRH_CNF13_0 | GPIO_CRH_CNF15_0 | GPIO_CRH_CNF10_0);
  //GPIOB->CRH |= GPIO_CRH_CNF13_1 | GPIO_CRH_CNF15_1;
  //GPIOB->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_MODE13_1 | GPIO_CRH_MODE15_1;

  //GPIOE PIN2 (as input pullup/pulldown)
  //! GPIOE->CRL = GPIO_CRL_CNF2_1;
  //! GPIOE->ODR = 0x0004; // PORTE2 is HIGH

  //External interrupt configuration register 1 (AFIO_EXTICR1) - pe2 ext2 interrupt enable
  AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PE;

  //Interrupt mask register - pe2 ext2 interrupt enable
  EXTI->IMR |= AFIO_EXTICR1_EXTI0_PE;

  //Falling trigger selection register (EXTI_FTSR) - falling on 2 pin port e
  EXTI->FTSR |= EXTI_FTSR_TR2;

  //USART1 INIT

  //USART1 BAUDRATE = 19200
  USART1->BRR = 0xEA6;
  USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  //USART2 INIT
  //USART2 BAUDRATE = 19200
  USART2->BRR = 0xEA6;
  USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

   //GPIO LED PIN(GPIO PUSH/PULL 50 MHZ)
  GPIOB->CRL |= GPIO_CRL_MODE5;
  GPIOE->CRL |= GPIO_CRL_MODE5;

  FLASH_init();
  //SPI1_Init();
  SPI2_Init();

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

void SPI1_Init(void)
{
  
  //SPI CLOCK = F/256
  SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;

  //CPOL (Polarity cls signal CPOL = 0)
  SPI1->CR1 &= ~SPI_CR1_CPOL;

  //CPHA (Phase cls signal    CPHA = 0)
  SPI1->CR1 &= ~SPI_CR1_CPHA;

  //DATA FRAME FORMAT (16 bit)
  //SPI1->CR1 |= SPI_CR1_DFF;

  //ENDING FRAME FORMAT (MSB will be first)
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST; 

  //SOFTWARE SLAVE MANAGEMENT=OFF
  //SPI1->CR1 |= SPI_CR1_SSM;

  //SSI (Internal slave select)=OFF
  //SPI1->CR1 |= SPI_CR1_SSI;

  //ENABLE RXNEIE IRQ (recive buffer not empty flag)
  //SPI1->CR2 |= SPI_CR2_RXNEIE;

  //ENABLE TXEIE IRQ(transmit buffer is empty)
  SPI1->CR2 |= SPI_CR2_TXEIE;

  //ENABLE ERROR IRQ
  SPI1->CR2 |= SPI_CR2_ERRIE;

  //NVIC_EnableIRQ(SPI1_IRQn);

  //SPI MODE = MASTER
  SPI1->CR2 |= SPI_CR2_SSOE;
  SPI1->CR1 |= SPI_CR1_MSTR;

  //SPI ENABLE
  SPI1->CR1 |= SPI_CR1_SPE;
 
}

void SPI2_Init(void)
{
  
  //SPI CLOCK = F/256
  SPI2->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;

  //CPOL (Polarity cls signal CPOL = 0)
  SPI2->CR1 &= ~SPI_CR1_CPOL;

  //CPHA (Phase cls signal    CPHA = 0)
  SPI2->CR1 &= ~SPI_CR1_CPHA;

  //DATA FRAME FORMAT (16 bit)
  //SPI1->CR1 |= SPI_CR1_DFF;

  //ENDING FRAME FORMAT (MSB will be first)
  SPI2->CR1 &= ~SPI_CR1_LSBFIRST; 

  //SOFTWARE SLAVE MANAGEMENT=OFF
  //SPI1->CR1 |= SPI_CR1_SSM;

  //SSI (Internal slave select)=OFF
  //SPI1->CR1 |= SPI_CR1_SSI;

  //ENABLE RXNEIE IRQ (recive buffer not empty flag)
  //SPI1->CR2 |= SPI_CR2_RXNEIE;

  //ENABLE TXEIE IRQ(transmit buffer is empty)
  SPI2->CR2 |= SPI_CR2_TXEIE;

  //ENABLE ERROR IRQ
  SPI2->CR2 |= SPI_CR2_ERRIE;

  //NVIC_EnableIRQ(SPI1_IRQn);

  //SPI MODE = MASTER
  SPI2->CR2 |= SPI_CR2_SSOE;
  SPI2->CR1 |= SPI_CR1_MSTR;

  //SPI ENABLE
  SPI2->CR1 |= SPI_CR1_SPE;
 
}

//SPI1 
uint16_t SPI1_Send_Data(uint8_t data)
{
  //wait to tx empty
  while (!(SPI1->SR & SPI_SR_TXE));
  //SEGGER_RTT_WriteString(0, "wait to tx empty \n");
  //Delay(10);
  CS_LOW;
  SPI1->DR = data & 0xFF;
  while (!(SPI1->SR & SPI_SR_TXE));
  data = SPI1->DR;
  CS_HIGH;
  return data;
}

//SPI2 
uint16_t SPI2_Send_Data(uint8_t data)
{
  //wait to tx empty
  while (!(SPI2->SR & SPI_SR_TXE));
  //DIGITAL_WRITE(GPIOD, 2, HIGH);
  SPI2->DR = data & 0xFF;
  while (!(SPI2->SR & SPI_SR_TXE));
  data = SPI2->DR;
  //DIGITAL_WRITE(GPIOD, 2, LOW);
  return data;
}

//INDEPENDENCE WATCHDOG TIMER
//==========================
void IWDG_Reset(void)
{
  IWDG->KR = 0xAAAA;
}

//USART1 
void USART1_SendString(char string[])
{

  uint32_t n = 0;
  while (string[n] != '\0')
    {
      USART1_SendChar(string[n]);
      n++;
    }
}

void USART1_SendStringEnd(char string[])
{

  uint32_t n = 0;
  while (string[n] != '\0')
    {
      USART1_SendChar(string[n]);
      n++;
    }
  USART1_SendChar('\r');
  USART1_SendChar('\n');
}

void USART1_SendChar(uint8_t data)
{

  while (!(USART1->SR & USART_SR_TC));               
  USART1->DR = data; 
}

void USART1_SendInt(uint32_t data)
{
  if (data < 0)
    {
      USART1_SendChar('-');
      data = -data;
    }
  if (data / 10)
    {
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
  if ((USART1->SR & USART_SR_RXNE) != (uint16_t)RESET)
    {
      buffer[*val] = USART1->DR;
      *val++;
    }
}

//USART2 
void USART2_SendString(char string[])
{

  uint32_t n = 0;
  while (string[n] != '\0')
    {
      USART2_SendChar(string[n]);
      n++;
    }
}

void USART2_SendStringEnd(char string[])
{

  uint32_t n = 0;
  while (string[n] != '\0')
    {
      USART2_SendChar(string[n]);
      n++;
    }
  USART2_SendChar('\r');
  USART2_SendChar('\n');
}

void USART2_SendChar(uint8_t data)
{

  while (!(USART2->SR & USART_SR_TC));             
  USART2->DR = data; 
}

void USART2_SendInt(uint32_t data)
{
  if (data < 0)
    {
      USART2_SendChar('-');
      data = -data;
    }
  if (data / 10)
    {
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
  if ((USART2->SR & USART_SR_RXNE) != (uint16_t) RESET)
    {
      buffer[*val] = USART2->DR;
      *val++;
    }
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
  if(FLASH->CR & FLASH_CR_LOCK) return -1;
  if(FLASH->SR & FLASH_SR_BSY) return 0;
   FLASH->CR |= FLASH_CR_PG;
   *(__IO uint16_t *)(address) = val;
   //здесь добавить обработку таймаута операции
   while(FLASH->SR & FLASH_SR_BSY);
   x = *(__IO uint16_t *)(address);
   if(x != val) return 0;
   if(FLASH->SR & FLASH_SR_PGERR) USART1_SendString("FLASH write error!\r\n");
   return x;

}

int16_t FLASH_check_page(uint32_t page_address)
{
  //check as words
  uint32_t counter;
  for(counter = 0; counter < PAGE_SIZE; counter++)
  {
    if(!(*(__IO uint32_t *)(page_address + counter))) return -1;
  }
  
  return 0;
}

int16_t FLASH_page_erase(uint32_t page)
{
  if(FLASH->CR & FLASH_CR_LOCK) return -1;
  if(FLASH->SR & FLASH_SR_BSY) return 0;

  //USART1_SendString("check 1\r\n");
  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = page;
  FLASH->CR |= FLASH_CR_STRT;

  //здесь добавить обработку таймаута операции
  while(FLASH->SR & FLASH_SR_BSY);
  //USART1_SendString("check 2\r\n");
  //change on select address from array
  if(FLASH_check_page(PAGE254)) return -10;
  return 0;
}

int16_t save_data(void)
{
  //check entry
  uint16_t key = 0x12CF;
  uint16_t cx = *(__IO uint16_t *)(PAGE254);
  USART1_SendInt(cx);
  USART1_SendString("\r\n");
  if(cx == key ) 
  {
    USART1_SendString("entry exist = !\r\n");
    USART1_SendInt(cx);
    USART1_SendString("\r\n");
    return 0;
  }
  
  //check page
  FLASH_unlock();
  int16_t z = FLASH_check_page(PAGE254);
  if(z) 
  {
    //USART1_SendInt(xxt);
    USART1_SendString("Page not clear!\r\n");

    z = FLASH_page_erase(PAGE254);
    //USART1_SendString("check 0\r\n");
    if(z == -1) USART1_SendString("FLASH locked!\r\n");
    else if(z == -10) USART1_SendString("FLASH bad erasing!\r\n");
    else if(!z) USART1_SendString("FLASH erased!\r\n");
  }
    
  z = FLASH_write(key,PAGE254);
  if(z) USART1_SendString("FLASH write ok!\r\n");
  else USART1_SendString("FLASH write bad!\r\n");
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
//=========
/*
uint8_t OPTBYTE_write(uint16_t data1, uint16_t data0)
{
  //check flash busy flag
  if(FLASH->SR & FLASH_SR_BSY) return 0;
  //unlock OPT
  FLASH->OPTKEYR = KEY1;
  FLASH->OPTKEYR = KEY2;
  if(!(FLASH->CR & FLASH_CR_OPTWRE)) return 0;
  //set OPTER
  FLASH->CR |= FLASH_CR_OPTER;
  //set STRT 
  FLASH->CR |= FLASH_CR_STRT;
  //waint for BSY
  while(FLASH->SR & FLASH_SR_BSY);
  FLASH->CR &= ~FLASH_CR_OPTER;

  //set OPTPG to FLASH_CR
  FLASH->CR |= FLASH_CR_OPTPG;
  //write data
  *(uint16_t *)(0x1FFFF804) = data1 & 0x00FF;
  //waint for BSY
  while(FLASH->SR & FLASH_SR_BSY);
  *(uint16_t *)(0x1FFFF806) = data0 & 0x00FF;
  //waint for BSY
  while(FLASH->SR & FLASH_SR_BSY);
  FLASH->CR &= ~FLASH_CR_OPTPG;
  FLASH->CR = FLASH_CR_LOCK;
   
  return 0;
}
 
int16_t OPTBYTE_erase(void)
{
  //unlock FPEC
  FLASH_unlock();
  //check flash busy flag
  if(FLASH->SR & FLASH_SR_BSY) return -1;
  //unlock OPTWRE in FLASH_CR
  FLASH->CR |= FLASH_CR_OPTWRE;
  //set OPTER
  FLASH->CR |= FLASH_CR_OPTER;
  //set STRT 
  FLASH->CR |= FLASH_CR_STRT;
  //waint for BSY
  while(FLASH->SR & FLASH_SR_BSY);
  //check erase errors
  return 0;  

}

uint8_t rewrite_option_bytes(const OB_TypeDef *new_values)
{
    FLASH->OPTKEYR = 0x45670123;
    FLASH->OPTKEYR = 0xCDEF89AB;
    USART1_SendString("OPTBYTES unlocking !\r\n");
    if(!(FLASH->CR & FLASH_CR_OPTWRE)) return 0;
    USART1_SendString("OPTBYTES unlock !\r\n");
    FLASH->CR |= FLASH_CR_OPTER;
    FLASH->CR |= FLASH_CR_STRT;
    while(FLASH->SR & FLASH_SR_BSY);
    USART1_SendString("OPTBYTES erase !\r\n");
    uint16_t const * pNew = (uint16_t const *)new_values;
    uint16_t * pOld = (uint16_t *)OB;
    for(uint_fast8_t i = 0; i < sizeof(*OB) / sizeof(uint16_t); ++i)
    {
        if((*pNew & 0xFF) != (*pOld & 0xFF))
        {
            FLASH->CR = FLASH_CR_OPTPG;
            *pOld = *pNew;
            while(FLASH->SR & FLASH_SR_BSY);
            USART1_SendString("OPTBYTES write entry!\r\n");
            FLASH->CR = 0;
            if((*pNew & 0xFF) != (*pOld & 0xFF))
                return 0;
        }
        pNew++;
        pOld++;
    }
    return 1;
}

//UTILS
void print_bin8(uint8_t val)
{
  for(int8_t i = 7; i >= 0; i--)
  {
    if(BIT_IS_SET8(val,i)) USART1_SendChar('1');
    else USART1_SendChar('0');
  }
  USART1_SendChar('\r');
  USART1_SendChar('\n');
}

void print_bin16(uint16_t val)
{
  for(int8_t i = 15; i >= 0; i--)
  {
    if(BIT_IS_SET16(val,i)) USART1_SendChar('1');
    else USART1_SendChar('0');
  }
  USART1_SendChar('\r');
  USART1_SendChar('\n');
}

void print_bin32(uint32_t val)
{
    for(int8_t i = 31; i >= 0; i--)
  {
    if(BIT_IS_SET32(val,i)) USART1_SendChar('1');
    else USART1_SendChar('0');
  }
  USART1_SendChar('\r');
  USART1_SendChar('\n');
}

void print_bin64(uint64_t val)
{
  for(int8_t i = 63; i >= 0; i--)
  {
    if(BIT_IS_SET64(val,i)) USART1_SendChar('1');
    else USART1_SendChar('0');
  }
  USART1_SendChar('\r');
  USART1_SendChar('\n');
}

*/