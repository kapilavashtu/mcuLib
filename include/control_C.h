#ifndef CONTROL_C_H
#define CONTROL_C_H


#include <inttypes.h>
#include "stm32f10x.h"

typedef struct 
{
  uint64_t val;
  uint64_t data;
} XTime;

typedef struct 
{
  uint8_t command_recieve;
  uint8_t command_size;

} XFlags;

typedef struct 
{
  uint16_t version;
  uint16_t cpu_info;
  uint16_t id;
  uint16_t flash_size;
  uint16_t ram_size;
  uint16_t ram_end;
  uint16_t ram_start;
  char lexem_array[1023];

} XSettings;

void Init_Peripheral(void);


//init timers
void Init_IWDG(uint32_t prescaler, uint32_t count_val);
void IWDG_Reset(void);

//SPI
void SPI1_Init(void);
uint16_t SPI1_Send_Data(uint8_t data);
void SPI2_Init(void);
uint16_t SPI2_Send_Data(uint8_t data);

//init usart
void USART1_Init(void);
void USART1_SendString(char string[]);
void USART1_SendStringEnd(char string[]);
void USART1_SendChar(uint8_t data);
void USART1_SendInt(uint32_t data);
void USART1_SendNewLine(void);
void Rx_Save1(char *buffer, uint32_t *val);

void USART2_Init(void);
void USART2_SendString(char string[]);
void USART2_SendStringEnd(char string[]);
void USART2_SendChar(uint8_t data);
void USART2_SendInt(uint32_t data);
void USART2_SendNewLine(void);
void Rx_Save2(char *buffer, uint32_t *val);

//flash
void FLASH_init(void);
void FLASH_unlock(void);

int16_t FLASH_write(uint16_t val, uint32_t address);
int16_t FLASH_check_page(uint32_t page_address);
int16_t FLASH_page_erase(uint32_t page);


uint8_t OPTBYTE_write(uint16_t data1, uint16_t data0);
int16_t OPTBYTE_erase(void);

int16_t save_data(void);
uint8_t rewrite_option_bytes(const OB_TypeDef *new_values);

//utils
void print_bin8(uint8_t val);
void print_bin16(uint16_t val);
void print_bin32(uint32_t val);
void print_bin64(uint64_t val);


#endif