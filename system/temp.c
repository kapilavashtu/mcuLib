 //Init_Complete();
    uint16_t spi_val = 0;
	

SEGGER_RTT_WriteString(0, "111 complete___ \n");
        USART1_SendStringEnd("rx_buffer content");
        GPIOB->ODR |= GPIO_ODR_ODR5;
        GPIOE->ODR |= GPIO_ODR_ODR5;
        SEGGER_RTT_WriteString(0, "  PB5 + PE5 is HIGH \n");
        goto point;
        while(1)
        {
          DIGITAL_WRITE(GPIOE, 5, HIGH);
          DIGITAL_WRITE(GPIOB, 5, LOW);
          Delay(1000);
          DIGITAL_WRITE(GPIOE, 5, LOW);
          DIGITAL_WRITE(GPIOB, 5, HIGH);
          Delay(1000);
        }
        //ActiveLedBlinking();
        /*
        while(1) 
        {
            spi1_rx_buffer[spi1_rx_counter] = SPI1_Send_Data(spi_val++);
            spi1_rx_counter++;
        }
        */
        ActiveLedBlinking();
        Delay(5000);
        USART1_SendString("count  = ");
        USART1_SendInt(count1);
        USART1_SendNewLine();
        USART1_SendStringEnd("rx_buffer content");
        SEGGER_RTT_WriteString(0, rx_buffer1);
        SEGGER_RTT_WriteString(0, "\r \n");
        USART1_SendStringEnd(rx_buffer1);
		
		
		
		
		
		
		
		
		
		
		
		if ((USART2->SR & USART_SR_RXNE) != (u16)RESET) {
        SEGGER_RTT_WriteString(0, "USART2_IRQ_HANDLER\r\n");
        SEGGER_RTT_WriteString(0, "Count === ");
        RTT_SendInt(count2);
        rx_buffer2[count2] = USART2->DR;
        count2++;
        uart_irq_counter2++;
    }
    if ((USART2->SR & USART_SR_ORE) != (u16)RESET) {
        USART2_SendStringEnd("Overrun error!!!");
    }
    if ((USART2->SR & USART_SR_FE) != (u16)RESET) {
        USART2_SendStringEnd("Frame error!!!");
    }
    if ((USART2->SR & USART_SR_PE) != (u16)RESET) {
        USART2_SendStringEnd("Parity error!!!");
    }
		
		
		
		
		
		
		
		
		
		
		
