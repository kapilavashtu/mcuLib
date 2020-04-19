#ifndef SPI_H
#define SPI_H

#include "defines.h"
#include <inttypes.h>

#define SPI_RX_BUFFER_SIZE              4096
#define SPI_TX_BUFFER_SIZE              512

#define FPCLK_2                         (uint16_t) 0x0000
#define FPCLK_4                         (uint16_t) 0x0008
#define FPCLK_8                         (uint16_t) 0x0010
#define FPCLK_16                        (uint16_t) 0x0018
#define FPCLK_32                        (uint16_t) 0x0020
#define FPCLK_64                        (uint16_t) 0x0028
#define FPCLK_128                       (uint16_t) 0x0030
#define FPCLK_256                       (uint16_t) 0x0038     

extern uint16_t spi1_rx_buffer[SPI_RX_BUFFER_SIZE];
extern uint16_t spi1_rx_counter;
extern uint16_t spi1_tx_buffer[SPI_TX_BUFFER_SIZE];
extern uint16_t spi1_tx_counter;


typedef struct 
{
  uint16_t clock;
  uint16_t spi_type;
  uint16_t frame_format;
  uint16_t ending;
  uint16_t mode;
  uint16_t role;
  uint8_t irq_en;
} spi_cfg; 


int16_t SPI_init(uint16_t spi_num, spi_cfg *config);
int16_t SPI_set_baudrate(uint16_t spi_num,uint16_t baudrate_val);
int16_t SPI_set_role(uint16_t spi_num, uint16_t role);
int16_t SPI_set_frame_format(uint16_t spi_num, uint16_t format);
int16_t SPI_set_enable(uint16_t spi_num, uint16_t state);
int16_t SPI_set_type(uint16_t spi_num,uint16_t type);
int16_t SPI_set_irq(uint16_t spi_num, uint16_t irq);
int16_t SPI_set_ending(uint16_t spi_num, uint16_t ending);

void SPI1_Init(void);
void SPI2_Init(void);

int16_t SPI_read(uint16_t spi_num, uint8_t *dest, uint16_t count, uint16_t flags);
uint16_t SPI_write(uint16_t spi_num, uint8_t data);
int16_t SPI_write_block(uint16_t spi_num, uint8_t *block, uint16_t count, uint16_t flags);

#endif

