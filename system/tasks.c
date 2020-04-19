#include "tasks.h"
#include <stdio.h>

extern volatile uint32_t delay_counter;

void ActiveLedBlinking(void) {

  DIGITAL_WRITE(GPIOB, 5, HIGH);
  DIGITAL_WRITE(GPIOE, 5, HIGH);
  Delay(800);
  DIGITAL_WRITE(GPIOB, 5, LOW);
  DIGITAL_WRITE(GPIOE, 5, LOW);
  Delay(800);
}

void Delay(uint32_t val) 
{

  delay_counter = val;
  while (delay_counter != 0);
  
}

void Init_Complete(void) {

  DIGITAL_WRITE(GPIOB, 5, HIGH);
  Delay(200);
  DIGITAL_WRITE(GPIOB, 5, LOW);
  Delay(200);
  DIGITAL_WRITE(GPIOB, 5, HIGH);
  Delay(200);
  DIGITAL_WRITE(GPIOB, 5, LOW);
  Delay(200);
  DIGITAL_WRITE(GPIOB, 5, HIGH);
  Delay(200);
  DIGITAL_WRITE(GPIOB, 5, LOW);
  Delay(200);
  DIGITAL_WRITE(GPIOB, 5, HIGH);
  Delay(200);
  DIGITAL_WRITE(GPIOB, 5, LOW);
}