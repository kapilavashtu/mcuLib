#include "timers.h"


extern volatile uint16_t tim6_count_val;
extern volatile uint32_t c_val;

void TIM6_IRQHandler(void)
{
	//printf("TIMER6_HANDLER \n");
        //TIM6 -> SR = ~TIM_SR_UIF;
	TIM6->SR = ~(TIM_SR_UIF);
	//__disable_irq();
	tim6_count_val++;
	c_val++;

	//TIM6->EGR = TIM_EGR_UG;
	//__enable_irq();
}


void TIMER6_Init(void)
{
	// TIM6 clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	//set clock source
	//TIM6->SMCR &= ~(TIM_SMCR_SMS);

	// setup prescaler
	//TIM6->PSC = 7200 - 1;
        TIM6->PSC = 0xFF;
        TIM6->ARR = 10000;
        //TIM6->EGR = TIM_EGR_UG;
        //        TIM6->SR = 0;

	// enable interrupt event
	//TIM6->DIER |= TIM_DIER_UIE;
        TIM6->DIER = TIM_DIER_UIE;

	//set only overflow even eneble
	TIM6->CR1 = TIM_CR1_CEN;

	//TIM6->PSC = 7200 - 1;
	//TIM6->ARR = 10000;

	//TIM6->EGR = TIM_EGR_UG;

	//TIM6->SR = 0;
	//TIM6->DIER = TIM_DIER_UIE;
	//TIM6->CR1 = TIM_CR1_CEN;
}