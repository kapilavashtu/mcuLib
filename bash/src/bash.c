#include "bash.h"
#include "commands.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <errno.h>

#define BASH_RL_BUFSIZE 512
#define BASH_TOK_BUFSIZE 32
#define BASH_TOK_DELIM " \t\r\n\a"

//static buffer for input string
char input_string[BASH_RL_BUFSIZE];
//static buffer for array of pointers
char *tokens[BASH_TOK_BUFSIZE];

void bash_loop(void)
{
	int argc = 0;
	int status = 0;

	char *line = NULL;
	char **argv = NULL;

	do {
		xprintf("> ");
		//reading command line from stdin to string
		line = bash_read_line();
		//devide string on tokens
		argv = bash_split_line(line, &argc);
		//executing command
		status = bash_exec(argc, argv);

	} while (status);
}

char *bash_read_line(void)
{
	int pos = 0;
	int c;

	while (1) {
		c = xgetchar();

		if (c == EOF)
			continue;

		if (c == '\n') {
			input_string[pos] = '\0';
			return input_string;
		} else {

			input_string[pos] = (char)c;
		}
		pos++;
	}
}

char **bash_split_line(char *line, int *argc)
{
	int pos = 0;
	char *token = NULL;

	token = strtok(line, BASH_TOK_DELIM);
	while (token != NULL) {
		tokens[pos] = token;
		pos++;
		token = strtok(NULL, BASH_TOK_DELIM);
	}

	tokens[pos] = NULL;
	*argc = pos;
	return tokens;
}

extern char *com_list[];
extern int (*com_func[])(int, char **);
//get are string list, read it and do it commands
int bash_exec(int argc, char **argv)
{
	int i;
	//if no tokens - exit
	if (argv[0] == NULL)
		return 0;

	uint8_t b = 0;
	for (i = 0; i < bash_com_nums(); i++) {
		if (strcmp(argv[0], com_list[i]) == 0) {
			//xprintf("gotcha\r\n");
			b = 1;
			return (*com_func[i])(argc, argv);
		}
	}

	if (b == 0) {
		xprintf("command %s nod found\n", argv[0]);
	}

	return 0;
}