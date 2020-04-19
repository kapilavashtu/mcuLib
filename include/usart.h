#ifndef USART_H
#define USART_H

#include "defines.h"

#define RX_BUF_SIZE (0xff)
#define TX_BUF_SIZE (0xff)

#define RX_BUF_MASK (RX_BUF_SIZE - 1)
#define TX_BUF_MASK (TX_BUF_SIZE - 1)

typedef struct Buffer {
	uint8_t idxIN;
	uint8_t idxOUT;
	uint8_t mask;
	uint8_t *buffer;

} buffer;

void USART_Init(uint16_t num, uint16_t baudrate, uint16_t modeset);

void USART1_SendString(char string[]);
void USART1_SendStringEnd(char string[]);
void USART1_SendChar(uint8_t data);
void USART1_SendInt(uint32_t data);
void USART1_SendNewLine(void);
void Rx_Save1(char *buffer, uint32_t *val);

uint16_t available(void);
uint8_t read(void);
//std io
int16_t write(USART_TypeDef *port, uint8_t *src, uint16_t len);


void USART2_SendString(char string[]);
void USART2_SendStringEnd(char string[]);
void USART2_SendChar(uint8_t data);
void USART2_SendInt(uint32_t data);
void USART2_SendNewLine(void);
void Rx_Save2(char *buffer, uint32_t *val);

//special
int xprintf(const char *fmt, ...);
int xgetchar(void);

//utils
uint16_t getBufLen(buffer *buf);
void clearBuffer(buffer *buf);
uint8_t emptyBuffer(buffer *buf);
void toBuffer(buffer *buf, uint8_t data);
uint8_t fromBuffer(buffer *buf);
int8_t closeBuffer(buffer *buf);
uint8_t getCurINidx(void);
uint8_t getCurOUTidx(void);

#endif