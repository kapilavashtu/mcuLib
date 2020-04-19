#ifndef SYSTEM_H
#define SYSTEM_H

#include "defines.h"

extern uint16_t errors;
 

typedef struct 
{
  uint64_t val;
  uint64_t data;
} Time;

typedef struct 
{
  uint8_t command_recieve;
  uint8_t command_size;

} Flags;

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

} Settings;

void Init_Peripheral(void);

//init timers
void Init_IWDG(uint32_t prescaler, uint32_t count_val);
void IWDG_Reset(void);

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

//nandler error messages 
void handlerTrapMesg(void);

//device status
void usart_status(uint16_t dev);
void spi_status(uint16_t dev);
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


#endif
