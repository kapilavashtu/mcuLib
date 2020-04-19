#include "sd_card.h"
#include "spi.h"
#include <stdio.h>


uint8_t response_r7[4];
uint8_t response_r1;
uint8_t response_r2[17];
uint8_t input_bufer[512];
uint8_t output_buffer[512];


void SD_powerup(void)
{
  //delay power up
  CS_HIGH;

  for(uint16_t i = 0; i < 10; i++)
  {
    SPI_write(2, EMPTY);
  }

  //remake SPI_send() - sync CS to end of TX data ++??
  while ((SPI2->SR & SPI_SR_BSY) && (SPI2->SR & SPI_SR_RXNE));
  CS_LOW;
  //goto idle state init
}

void SD_idle_state(void)
{
  //send CMD0
  SPI_write(2,0x40);//start
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x95);//crc
 
  //wait for responce
  //later do check timeout
  while((response_r1 = SPI_write(2, EMPTY)) != 0x01);
  //goto ready state init
}

void SD_ready_state(void)
{
  //send CMD8 - voltage range check
  SPI_write(2,0x48);//start
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x01);//Vdd = 2.7-3.6
  SPI_write(2,0xAA);//byte order
  SPI_write(2,0x87);//crc

  uint8_t card_type = 0;
  uint8_t status = 0;
  //TODO
  //do it non-blocking
  while(1)
  {
    status = SPI_write(2, EMPTY);
    if(status == 0x04)
    {
      printf("SD: v1\n");
      card_type = 1;
      break;
    } else if(status == 0x01)
    {
      printf("SD: v1\n");
      card_type = 2;
      break;
    }
  }

  //4 byte shifting(pass) from card response(CMD8 response)
  for (uint8_t i = 0; i < 4; SPI_write(2, EMPTY), i++);

  if(card_type == 1) SD_version1_start();
  else if(card_type == 2) SD_version2_start();

  CS_HIGH;
  //init end (5 ms left)
  //do read or write,otherwise re-init from step
  
}

void SD_version1_start(void)
{
  printf("SD init version 1...\n");
  //send empty CMD1
  while(1)
  {
    if(SPI_write(2, 0x48) == 0x00) break;
    if(SPI_write(2, 0x00) == 0x00) break;
    if(SPI_write(2, 0x00) == 0x00) break;
    if(SPI_write(2, 0x00) == 0x00) break;
    if(SPI_write(2, 0x00) == 0x00) break;
    if(SPI_write(2, 0x00) == 0x00) break;
  }

}

void SD_version2_start(void)
{
  printf("SD init version 2...\n");
  uint8_t state = 0;
  response_r7[1] = 0;

  //CMD58 - response R3(5 byte)
  SPI_write(2,0x7A);//start
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);//crc

  //wait response
  while((state = SPI_write(2, EMPTY)) != 0x01);
  //save response
  for(uint8_t i = 0; i < 4; response_r7[i] = SPI_write(2, EMPTY), i++);

  //CMD55
  SPI_write(2,0x77);//start
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);//crc

  //wait response
  while((state = SPI_write(2, EMPTY)) != 0x01);
  
  //CMD41
  SPI_write(2,0x69);//start
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);//crc 

  //wait response
  while((state = SPI_write(2, EMPTY)) != 0x01);
  CS_HIGH;
}

void SD_read_CID(void)
{
  CS_LOW; 
  //CMD41
  SPI_write(2,0x4A);//start
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);//crc  

  //wait response
  while(SPI_write(2, EMPTY) != 0x01);
  
  //wait token
  while(SPI_write(2, EMPTY) != 0xFE);
 
   //save response
  for(uint8_t i = 0; i < 17; response_r2[i] = SPI_write(2, EMPTY), i++);

  SPI_write(2,EMPTY);
  SPI_write(2,EMPTY);

  while ((SPI2->SR & SPI_SR_BSY) && (SPI2->SR & SPI_SR_RXNE));

  CS_HIGH;
  printf("SD ID...\n");
  //for(uint8_t i = 0; i < 17; USART1_SendChar(response_r2[i]),i++);
}

void SD_read_block(uint32_t address, uint8_t *buffer)
{
  CS_LOW;
  //CMD17
  SPI_write(2,0x51);//start
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);//crc 
  
  //wait ack
  while(SPI_write(2, EMPTY) != 0x01);

  //wait token
  while(SPI_write(2, EMPTY) != 0xFE);

  //read block
  for(uint16_t i = 0; i < 512; input_bufer[i] = SPI_write(2, EMPTY), i++);

  //CRC
  SPI_write(2, 0x00);
  SPI_write(2, 0x00);
  CS_HIGH;
}

void SD_write_block(uint32_t address, uint8_t *block)
{
  CS_LOW;
  //CMD24 
  SPI_write(2,0x58);//start
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);
  SPI_write(2,0x00);//crc 

  while(SPI_write(2, EMPTY) != 0x01);
  SPI_write(2, EMPTY);
  SPI_write(2, EMPTY);
  for(uint16_t i = 0; i < 512; SPI_write(2, output_buffer[i]), i++);
  
  //CRC
  SPI_write(2, 0x00);
  SPI_write(2, 0x00);

  //wait ack
  while(SPI_write(2, EMPTY) != 0x05);

  CS_HIGH;
  //delay
  SPI_write(2, 0x00);
  SPI_write(2, 0x00);
  SPI_write(2, 0x00);
  SPI_write(2, 0x00);
}

void SD_erase_block(uint32_t address)
{

}

void SD_init(void)
{
  printf("SD init start...\n");
  SD_powerup();
  SPI_set_baudrate(2, 0x0038);
  while(!(SPI2->CR1 & SPI_CR1_BR_2)) SPI2->CR1 |= SPI_CR1_BR_2;
  SD_idle_state();
  SD_ready_state();
  //SD_read_CID();
/*
  for(uint16_t i = 0; i < 512; i++)
  {
    if(i > 255) output_buffer[i] = 512 - i;
    else output_buffer[i] = i;
   }
*/
  //SD_write_block(0x00000000,output_buffer);
  SD_read_block(0x00000000, input_bufer);

  printf("SD read block...\n");
  for(uint16_t i = 0; i < 512; i++)
  {
    printf("%c\n",input_bufer[i]);
    Delay(1);
  }
}