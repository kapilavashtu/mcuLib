#ifndef SD_CARD_H
#define SD_CARD_H

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

#include "__libc.h"

#include "defines.h"
#include "tasks.h"


//SD card
void SD_powerup(void);
void SD_init(void);
void SD_idle_state(void);
void SD_ready_state(void);
void SD_version1_start(void);
void SD_version2_start(void);

void SD_read_CID(void);


void SD_send_command(void); 
void SD_read_block(uint32_t address, uint8_t *buffer);
void SD_write_block(uint32_t address, uint8_t *block);
void SD_erase_block(uint32_t address);

void SD_read_register(void);

#endif