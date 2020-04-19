#ifndef BASH_H
#define BASH_H

#include <inttypes.h>

//basic func
void bash_loop(void);

char *bash_read_line(void);
char **bash_split_line(char *line, int *argc);
int bash_exec(int argc, char **argv);

#endif