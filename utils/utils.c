#include "utils.h"
#include "usart.h"


//UTILS
void print_bin8(uint8_t val)
{
  for (int8_t i = 7; i >= 0; i--)
    {
      if (BIT_IS_SET8(val, i))
        USART1_SendChar('1');
      else
        USART1_SendChar('0');
    }
  USART1_SendChar('\r');
  USART1_SendChar('\n');
}


void print_bin16(uint16_t val)
{
  for (int8_t i = 15; i >= 0; i--)
    {
      if (BIT_IS_SET16(val, i))
        USART1_SendChar('1');
      else
        USART1_SendChar('0');
    }
  USART1_SendChar('\r');
  USART1_SendChar('\n');
}


void print_bin32(uint32_t val)
{
  for (int8_t i = 31; i >= 0; i--)
    {
      if (BIT_IS_SET32(val, i))
        USART1_SendChar('1');
      else
        USART1_SendChar('0');
    }
  USART1_SendChar('\r');
  USART1_SendChar('\n');
}


void print_bin64(uint64_t val)
{
  for (int8_t i = 63; i >= 0; i--)
    {
      if (BIT_IS_SET64(val, i))
        USART1_SendChar('1');
      else
        USART1_SendChar('0');
    }
  USART1_SendChar('\r');
  USART1_SendChar('\n');
}