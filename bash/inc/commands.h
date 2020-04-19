#ifndef COMMANDS_H
#define COMMANDS_S

int bash_com_nums(void);

int bash_sample(int argc, char **args);
int bash_help(int argc, char **args);
int bash_exit(int argc, char **args);

int bash_clist(int argc, char **args);
int bash_uptime(int argc, char **args);

int bash_read_mem(int argc, char **args);
int bash_write_mem(int argc, char **args);

int bash_read_string(int argc, char **args);

int bash_usart_status(int argc, char **args);
int bash_dev_status(int argc, char **args);
int bash_systemctl(int argc, char **args);
int bash_system(int argc, char **args);
int bash_set(int argc, char **args);

int bash_clear(int argc, char **args);

int bash_set_gpio(int argc, char **args);
int bash_set_pwm(int argc, char **args);

#endif