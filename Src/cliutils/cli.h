#ifndef CLIUTILS_CLI
#define CLIUTILS_CLI

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MAX_TOK_LEN (32) // maximal token length
#define TERMINAL_LEAD (">> ") // terminal lead

typedef char CliToken_Type[MAX_TOK_LEN];

#define MAX_TOKEN_N (8) // maximal token count for a single command
#define MAX_HELP_LEN (192) // maximal help line length for a single command

typedef int (*fnCliCallback)(const CliToken_Type * ppArgs, uint8_t argc); // function prototype for a callbacks

void cli_init(); // initialize CLI

void process_cli_line(char *pLine); // sor feldolgozása
int cli_register_command(char *pCmdParsHelp, uint8_t cmdTokCnt, uint8_t minArgCnt, fnCliCallback pCB); // register a new command
void cli_remove_command(int cmdIdx); // remove an existing command
void cli_remove_command_array(int * pCmdHandle); // remove bunch of commands, terminated by -1

void cli_print_hist_stk(); // print CLI history stack

bool get_param_value(const CliToken_Type *ppArgs, uint8_t argc, const char * pKey, char * pVal); // get parameter value from a list of tokens

#define CMD_FUNCTION(name) int (name) (const CliToken_Type * ppArgs, uint8_t argc)

#define ONOFF(str) ((!strcmp(str, "on")) ? 1 : ((!strcmp(str, "off")) ? 0 : -1))

#endif /* CLIUTILS_CLI */
