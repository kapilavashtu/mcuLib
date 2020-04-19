#include "commands.h"
#include "bash.h"
#include "system.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <unistd.h>

int val = 64;
int boo = 128;
const char *test_string = "abracadabra";

char *com_list[] = {"sample", "help", "exit", "clear", "read_mem", "read_string", "dev_stat"};

int (*com_func[])(int, char **) = {&bash_sample, &bash_help, &bash_exit, &bash_clear, &bash_read_mem, &bash_read_string, &bash_dev_status};

int bash_com_nums(void)
{
	return (sizeof(com_list) / sizeof(char *));
}

//command define
int bash_sample(int argc, char **args)
{
	xprintf("its sample procedure\n");
	xprintf("address of val = %x\n", &val);
	xprintf("address of boo = %x\n", &boo);
	xprintf("address of test_string = %x\n", test_string);
	return 0;
}

int bash_help(int argc, char **args)
{
	xprintf("help get %d arguments:\r\n", argc);
	for (int i = 0; i < argc; i++)
		xprintf("%d arg = %s\r\n", i, args[i]);
	return 0;
}

int bash_exit(int argc, char **args)
{
	xprintf("its exit procedure\r\n");
}

int bash_clist(int argc, char **args)
{
}

int bash_uptime(int argc, char **args)
{
}

int bash_read_mem(int argc, char **args)
{
	if (strcmp(args[1], "m") == 0) {
		uint32_t address = atoi(args[2]);
		xprintf("address %d = %d\n", address, *(uint8_t *)(address));
		return 0;
	}
}

int bash_read_string(int argc, char **args)
{
	if (strcmp(args[1], "s") == 0) {
		uint32_t address = atoi(args[2]);
		uint32_t size = atoi(args[3]);
		xprintf("string at address %d: ", address);

		for (int i = 0; i < size; i++) {
			if (*(uint8_t *)(address + i) == NULL) {
				USART1_SendChar('_');
				continue;
			}
			USART1_SendChar(*(uint8_t *)(address + i));
		}

		xprintf(" ### end string\n");
		return 0;
	}
}

int bash_write_mem(int argc, char **args)
{
}

int bash_usart_status(int argc, char **args)
{
}

int bash_dev_status(int argc, char **args)
{
	xprintf("get device status:\n");
	int dev_num = atoi(args[2]);
	if (args[1] == "spi") {

	} else if ((strcmp(args[1], "uart")) == 0) {
		usart_status(dev_num);
	} else if ((strcmp(args[1], "spi")) == 0) {
		spi_status(dev_num);

	} else if ((strcmp(args[1], "adc")) == 0) {
		xprintf("adc:\n");

	} else if ((strcmp(args[1], "gpio")) == 0) {
		xprintf("gpio:\n");

	} else if ((strcmp(args[1], "dac")) == 0) {
		xprintf("dac:\n");

	} else if ((strcmp(args[1], "tim")) == 0) {
		xprintf("tim:\n");

	} else if ((strcmp(args[1], "rtc")) == 0) {
		xprintf("rtc:\n");

	} else if ((strcmp(args[1], "iwdg")) == 0) {
		xprintf("iwdg:\n");

	} else if ((strcmp(args[1], "wwdg")) == 0) {
		xprintf("wwdg:\n");

	} else if ((strcmp(args[1], "fsmc")) == 0) {
		xprintf("fsmc:\n");

	} else if ((strcmp(args[1], "sdio")) == 0) {
		xprintf("sdio:\n");

	} else if ((strcmp(args[1], "usb")) == 0) {
		xprintf("usb:\n");
	} else {
		xprintf("summary device status:\n");
	}
}

int bash_systemctl(int argc, char **args)
{
}

int bash_system(int argc, char **args)
{
}

int bash_set(int argc, char **args)
{
}

int bash_set_gpio(int argc, char **args)
{
}

int bash_set_pwm(int argc, char **args)
{
}

int bash_clear(int argc, char **args)
{
	xprintf("\e[1;1H\e[2J");
	return 0;
}