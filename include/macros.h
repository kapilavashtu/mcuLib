#ifndef MACROS_H
#define MACROS_H


#define   SET_BIT8(reg, bit)                                    reg |= ((uint8_t) 1 << bit)
#define   CLEAR_BIT8(reg, bit)                                  reg &= (~((uint8_t) 1 << bit))
#define   INV_BIT8(reg, bit)                                    reg ^= ((uint8_t) 1 << bit)
#define   BIT_IS_SET8(reg, bit)                                 ((reg & ((uint8_t) 1 << bit)) != 0)
#define   BIT_IS_CLEAR8(reg, bit)                               ((reg & ((uint8_t) 1 << bit)) == 0)

#define   SET_BIT_16(reg, bit)                                  reg |= ((uint16_t) 1 << bit)
#define   CLEAR_BIT16(reg, bit)                                 reg &= (~((uint16_t) 1 << bit))
#define   INV_BIT16(reg, bit)                                   reg ^= ((uint16_t) 1 << bit)
#define   BIT_IS_SET16(reg, bit)                                ((reg & ((uint16_t) 1 << bit)) != 0)
#define   BIT_IS_CLEAR16(reg, bit)                              ((reg & ((uint16_t) 1 << bit)) == 0)

#define   SET_BIT32(reg, bit)                                   reg |= ((uint32_t) 1 << bit)
#define   CLEAR_BIT32(reg, bit)                                 reg &= (~((uint32_t) 1 << bit))
#define   INV_BIT32(reg, bit)                                   reg ^= ((uint32_t) 1 << bit)
#define   BIT_IS_SET32(reg, bit)                                ((reg & ((uint32_t) 1 << bit)) != 0)
#define   BIT_IS_CLEAR32(reg, bit)                              ((reg & ((uint32_t) 1 << bit)) == 0)

#define   SET_BIT64(reg, bit)                                   reg |= ((uint64_t) 1 << bit)
#define   CLEAR_BIT64(reg, bit)                                 reg &= (~((uint64_t) 1 << bit))
#define   INV_BIT64(reg, bit)                                   reg ^= ((uint64_t) 1 << bit)
#define   BIT_IS_SET64(reg, bit)                                ((reg & ((uint64_t) 1 << bit)) != 0)
#define   BIT_IS_CLEAR64(reg, bit)                              ((reg & ((uint64_t) 1 << bit)) == 0)

#define HIGH 0
#define LOW 16
#define DIGITAL_WRITE(port, pin, state)                         port->BSRR |= (1 << (pin + state))

//memory protection unit define section

#define MPU_TYPE                                                ((uint32_t) 0xE000ED90)  

//flash defines

#define PAGE_SIZE                                               ((uint32_t) 0x0000003FF)

#define RDPRT_KEY						((uint16_t) 0x00A5)
#define KEY1							((uint32_t) 0x45670123)
#define KEY2							((uint32_t) 0xCDEF89AB)


#define OPT_USER						((uint32_t) 0x1FFFF800)
#define OPT_RDP							((uint32_t) 0x1FFFF802)
#define OPT_USER_DATA1                                          ((uint32_t) 0x1FFFF804)
#define OPT_USER_DATA0                                          ((uint32_t) 0x1FFFF806)
#define OPT_WRP1						((uint32_t) 0x1FFFF809)
#define OPT_WRP0						((uint32_t) 0x1FFFF80B)
#define OPT_WRP3						((uint32_t) 0x1FFFF80D)
#define OPT_WRP2						((uint32_t) 0x1FFFF80F)

#define PAGE254                                                 ((uint32_t) 0x0807F400)


//SPI1 control
//#define CS_LOW GPIOG->BSRR = GPIO_BSRR_BR7;
//#define CS_HIGH GPIOG->BSRR = GPIO_BSRR_BS7;
//#define CE_LOW GPIOG->BSRR = GPIO_BSRR_BR6;
//#define CE_HIGH GPIOG->BSRR = GPIO_BSRR_BS6;


//SD CARD
#define START_BYTE                                             ((uint8_t) 0x40)
#define CS_HIGH                                                GPIOD->BSRR |= (1 << (2 + HIGH))
#define CS_LOW                                                 GPIOD->BSRR |= (1 << (2 + LOW))

#define DATA_TOKEN                                             ((uint8_t) 0xFE)
#define EMPTY                                                  ((uint8_t) 0xFF)

#define WRITE_OK                                               ((uint8_t) 0x05)
#define CRC_ERROR                                              ((uint8_t) 0x0B)
#define WRITE_ERROR                                            ((uint8_t) 0x0D)
    

#endif