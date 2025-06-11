#include "cli.h"

#include <cmsis_os2.h>

#include <stdio.h>
#include <string.h>

#include <blocking_io/blocking_fifo.h>
#include <standard_output/standard_output.h>

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

// ----- TASK PROPERTIES -----
#define CLI_STACK_SIZE (2048)
#define CLI_TASK_NAME ("cli")
static osThreadId_t sTH;     // task handle
void task_cli(void *pParam); // task routine function
// ---------------------------

// structure for defining cli commands
struct CliCommand {
    CliToken_Type ppTok[MAX_TOKEN_N]; // tokens
    uint8_t tokCnt;                   // number of tokens in command without arguments
    uint8_t minArgCnt;                // minimal number of parameters
    char pHelp[MAX_HELP_LEN];         // help line
    char *pHint;                      // pointer to the hint part of the command (allocated on pHelp)
    uint8_t cmdLen;                   // length of the command string
    char *pPadding;                   // padding between command and hint parts (allocated on pHelp)
    fnCliCallback pCB;                // processing callback function
};

#define CLI_MAX_CMD_CNT (48) // limit on number of separate commands
static struct CliCommand spCliCmds[CLI_MAX_CMD_CNT];
static uint8_t sCliCmdCnt;
static bool sCmdsTidy = false;

#define CLI_INPUT_FIFO_BUFSIZE (512)
static BFifo sInputFifo;                         // input FIFO
static uint8_t sFifoMem[CLI_INPUT_FIFO_BUFSIZE]; // memory allocated for input FIFO

static bool sInitialized = false; // CLI is initialized

static osMutexId_t modifyMtx = NULL; // mutex protecting the command database

// ---------------------------

// register and initialize task
void cli_init() {
    // in the beginning there are no registered commands
    sCliCmdCnt = 0;

    // initialize FIFO
    bfifo_create(&sInputFifo, sFifoMem, CLI_INPUT_FIFO_BUFSIZE);

    // initialize modification mutex
    modifyMtx = osMutexNew(NULL);

    osThreadAttr_t attr;
    memset(&attr, 0, sizeof(osThreadAttr_t));
    attr.stack_size = CLI_STACK_SIZE;
    attr.name = CLI_TASK_NAME;
    sTH = osThreadNew(task_cli, NULL, &attr);
    if (sTH == NULL) { // taszk létrehozása
        MSG("Failed to create CLI task!\n");
    }
}

// ---------------------------

#define CLI_LINEBUF_LENGTH (191)
#define TOK_ARR_LEN (16)

static void tokenize_cli_line(char *pLine, CliToken_Type ppTok[], uint32_t tokMaxLen, uint32_t tokMaxCnt, uint32_t *pTokCnt) {
    uint32_t len = strlen(pLine);

    // copy to prevent modifying original one
    static char pLineCpy[CLI_LINEBUF_LENGTH];
    strcpy(pLineCpy, pLine);

    *pTokCnt = 0;

    // prevent processing if input is empty
    if (len == 0 || tokMaxCnt == 0) {
        return;
    }

    // first token
    char *pTok = strtok(pLineCpy, " ");
    strncpy(&ppTok[0][0], pTok, tokMaxLen);
    (*pTokCnt)++;

    // further tokens
    while ((*pTokCnt < tokMaxCnt) && (pTok != NULL)) {
        pTok = strtok(NULL, " ");

        if (pTok != NULL) {
            strncpy(&ppTok[*pTokCnt][0], pTok, tokMaxLen); // store token
            (*pTokCnt)++;                                  // increment processed token count
        }
    }
}

#define HINT_DELIMITER ('\t')
#define MIN_GAP_IN_SPACES (3)
static uint32_t sHelpPadding = 0;

static void tidy_format_commands() {
    if (sCmdsTidy) {
        return;
    }

    // get maximal line length
    uint32_t i, max_line_length = 0;
    for (i = 0; i < sCliCmdCnt; i++) {
        // fetch command descriptor
        struct CliCommand *pCmd = &spCliCmds[i];

        // if this command is unprocessed
        if (pCmd->pHint == NULL) {
            // search for hint text
            char *pDelim = strchr(pCmd->pHelp, HINT_DELIMITER);

            if (pDelim == NULL) {
                pCmd->cmdLen = strlen(pCmd->pHelp);       // the whole line only contains the command, no hint
                pCmd->pHint = pCmd->pHelp + pCmd->cmdLen; // make the hint point to the terminating zero
            } else {
                // calculate the length of the command part
                pCmd->cmdLen = pDelim - pCmd->pHelp;

                // split help-line
                *pDelim = '\0';           // get command part
                pCmd->pHint = pDelim + 1; // get hint part

                // trim hint
                while (*(pCmd->pHint) <= ' ') {
                    pCmd->pHint++;
                }

                // allocate padding
                pCmd->pPadding = pCmd->pHint + strlen(pCmd->pHint) + 1;
            }
        }

        // update max line length
        max_line_length = MAX(max_line_length, pCmd->cmdLen);
    }

    // fill-in paddings
    for (i = 0; i < sCliCmdCnt; i++) {
        // fetch command descriptor
        struct CliCommand *pCmd = &spCliCmds[i];

        // calculate padding length
        uint8_t padLen = max_line_length - pCmd->cmdLen + MIN_GAP_IN_SPACES;

        // MSG("pHelp:    %x\npHint:    %d\npPadding: %d\n\n", pCmd->pHelp, pCmd->pHint - pCmd->pHelp, pCmd->pPadding - pCmd->pHint);

        // fill-in padding
        for (uint32_t k = 0; k < padLen; k++) {
            pCmd->pPadding[k] = ' ';
        }

        // terminating zero
        pCmd->pPadding[padLen] = '\0';
    }

    sCmdsTidy = true;

    // refresh help line padding
    sHelpPadding = max_line_length - 1 + MIN_GAP_IN_SPACES;
}

#define CLI_HISTORY_CMD "hist"

void process_cli_line(char *pLine) {
    CliToken_Type ppTok[TOK_ARR_LEN];
    uint32_t tokCnt = 0;

    // tokenize line received from user input
    tokenize_cli_line(pLine, ppTok, MAX_TOK_LEN, TOK_ARR_LEN, &tokCnt);

    if (tokCnt == 0) {
        return;
    }

    int ret = -1;

    // print help
    if (!strcmp(ppTok[0], "?") || !strcmp(ppTok[0], "help")) {
        // tidy-up help if not formatted yet
        tidy_format_commands();

        // ---- BUILT-IN commands ----

        // print "?"
        MSG("\n\n" ANSI_COLOR_BYELLOW "?" ANSI_COLOR_CYAN "%*cPrint this help (%d/%d)" ANSI_COLOR_RESET "\n", sHelpPadding, ' ', sCliCmdCnt, CLI_MAX_CMD_CNT);

        // print "hist"
        uint32_t histPadding = MAX(((int32_t)sHelpPadding) - (strlen(CLI_HISTORY_CMD) - 1), 1);
        MSG(ANSI_COLOR_BYELLOW CLI_HISTORY_CMD ANSI_COLOR_CYAN "%*cPrint command history" ANSI_COLOR_RESET "\n", histPadding, ' ');

        // --------------

        uint8_t i;
        for (i = 0; i < sCliCmdCnt; i++) {
            MSG(ANSI_COLOR_BYELLOW "%s" ANSI_COLOR_RESET "%s" ANSI_COLOR_CYAN "%s" ANSI_COLOR_RESET "\n",
                spCliCmds[i].pHelp, spCliCmds[i].pPadding, spCliCmds[i].pHint);
        }

        MSG("\n\n");

        ret = 0;
    } else {
        // lookup command
        uint8_t i, k = 0, matchCnt = 0;
        int8_t n = -1;
        for (i = 0; i < sCliCmdCnt; i++) {
            matchCnt = 0;

            for (k = 0; k < spCliCmds[i].tokCnt && k < tokCnt; k++) {
                if (strcmp(ppTok[k], spCliCmds[i].ppTok[k])) {
                    break;
                } else {
                    matchCnt++;
                    if (matchCnt == spCliCmds[i].tokCnt) {
                        n = i;
                        break;
                    }
                }
            }

            if (n != -1) {
                break;
            }
        }

        // call command callback function
        if (n < 0) {
            ret = -1;
        } else {
            struct CliCommand *pCmd = &spCliCmds[n];
            uint8_t argc = tokCnt - pCmd->tokCnt;

            if (argc < pCmd->minArgCnt) {
                MSG(ANSI_COLOR_BRED "Insufficient parameters, see help! (" ANSI_COLOR_BYELLOW "?" ANSI_COLOR_BRED ")\n" ANSI_COLOR_RESET);
            } else {
                ret = pCmd->pCB(&ppTok[pCmd->tokCnt], argc);
            }
        }
    }

    if (ret < 0) {
        MSG(ANSI_COLOR_BRED "Unknown command or bad parameter: '" ANSI_COLOR_RESET "%s" ANSI_COLOR_BRED
                            "', see help! (" ANSI_COLOR_BYELLOW "?" ANSI_COLOR_BRED ")\n" ANSI_COLOR_RESET,
            pLine);
    }
}

#define HISTORY_STACK_DEPTH (8)
static char sppCmdHistStk[HISTORY_STACK_DEPTH][CLI_LINEBUF_LENGTH + 1]; // 0: newest element
static int16_t sHistStkLevel = 0;

static void put_onto_cmd_hist_stk(const char *pLine) {
    // if input line is empty, then stop
    if (*pLine == '\0') {
        return;
    }

    // if newly issued command is the last one, then don't push the stack
    uint16_t newLen = strlen(pLine);
    uint16_t firstLen = strlen(sppCmdHistStk[0]);
    if (!strncmp(pLine, sppCmdHistStk[0], MAX(firstLen, newLen))) {
        return;
    }

    // push elements one slot deeper
    int16_t i = 0;
    for (i = HISTORY_STACK_DEPTH - 2; i >= 0; i--) {
        strcpy(sppCmdHistStk[i + 1], sppCmdHistStk[i]);
    }

    // copy the new data
    strcpy(sppCmdHistStk[0], pLine);

    // increase stack level
    sHistStkLevel = MIN(sHistStkLevel + 1, HISTORY_STACK_DEPTH);

    //MSG("Pushed: %s\nLevel: %d\n", pLine, sHistStkLevel);
}

void cli_print_hist_stk() {
    MSG(ANSI_COLOR_BYELLOW "History stack level: " ANSI_COLOR_BGREEN "%d\n\n" ANSI_COLOR_RESET, sHistStkLevel);

    for (int16_t i = 0; i < sHistStkLevel; i++) {
        MSG(ANSI_COLOR_BYELLOW "%2d" ANSI_COLOR_RESET ":" ANSI_COLOR_CYAN " %s\n" ANSI_COLOR_RESET, i, sppCmdHistStk[i]);
    }
}

// special keycodes
#define KC_BS (8)
#define KC_ESC (27)
#define KC_HOME (91)
#define KC_DEL (127)

static void get_line(char *pStr, size_t *pLen) {
    getline_start:

    (*pLen) = 0;

    char c = '\0';          // currect character
    bool escString = false; // mark that we are processing an escape sequence
    uint8_t escPos = 0;     // escape sequence position
    char escParam[4];       // escape sequence paramters
    bool noMod = true;      // no modification happened to the input line so far
    int8_t histIdx = -1;    // history index

    while (c != '\r' && c != '\n' && (*pLen) < CLI_LINEBUF_LENGTH) {
        bfifo_wait_avail(&sInputFifo);                                // wait for availbale data
        uint32_t readLen = bfifo_read(&sInputFifo, (uint8_t *)&c, 1); // read characters
        if (readLen > 0) {
            bfifo_pop(&sInputFifo, readLen, osWaitForever); // pop the character read
        } else {
            continue;
        }

        // ESC received
        if (c == KC_ESC) {
            escString = true;
            escPos = 0;
            continue; // read next char
        }

        // if not in the middle of an escape sequence
        if (!escString) {
            MSGchar(c); // echo the character just read
        }

        if (!escString) { // normal character received
            if ((c >= ' ') && (c != KC_DEL)) {
                pStr[(*pLen)++] = c;
            } else if ((c == KC_BS) || (c == KC_DEL)) {
                noMod = false;
                if ((*pLen > 0)) {
                    (*pLen)--;
                    MSGchar(' ');
                    MSGchar(KC_BS);
                }
            } else if (c == '\t') {
            }
        } else {                                   // if escape string has been received
            escPos++;                              // increment ESC sequence position
            if (escPos == 1) {                     // examine first character
                if (!((c == '[') || (c == 'O'))) { // first character must be either '[' or 'O'
                    escString = false;
                }
            } else if (escPos > 1) {           // we are scanning the parameter
                uint8_t paramIdx = escPos - 2; // calculate parameter character index

                // a number might be followed by another alphanumeric character, but no letter precedes further sections of the parameter
                // tilde marks the end of the escape sequence
                if ((c == '~') || ((c >= 'A') && (c <= 'Z'))) {
                    escString = false;

                    // process escape sequence
                    switch (escParam[0]) {
                    case 'A':        // UP
                    case 'B': {      // DOWN
                        if (noMod) { // replace only if no modification has made to the input buffer
                            // increment/decrement history index
                            bool up = escParam[0] == 'A';
                            if (up) {
                                histIdx = MIN(sHistStkLevel - 1, histIdx + 1);
                            } else {
                                histIdx = MAX(-1, histIdx - 1);
                            }

                            // replace input line
                            MSGchar('\r');          // return carriage
                            MSG("%*c", *pLen, ' '); // fill line with spaces
                            MSGchar('\r');          // return carriage again

                            if (histIdx > -1) {
                                char *histLine = sppCmdHistStk[histIdx];
                                strcpy(pStr, histLine); // replace the buffer with the line from the history
                                MSGraw(histLine); // fetch the line from the history stack
                                *pLen = strlen(histLine);
                            }
                        }
                    } break;
                    }
                }

                // receiving tilde means that we reached the end of the escape sequence, any other character is part of the paramter
                if (c != '~') {
                    escParam[paramIdx] = c;
                }
            }
        }
    }

    // insert NULL-termination
    pStr[*pLen] = '\0';

    // print a CRLF sequence
    MSGraw("\r\n");

    // trim the line

    // left trim
    while ((*pStr) <= ' ' && (*pStr != '\0')) {
        pStr++;
    }

    // right trim
    for (int16_t i = ((int16_t)strlen(pStr)) - 1; (i > -1) && (pStr[i] <= ' '); i--) {
        pStr[i] = '\0';
    }
    
    // put line onto history stack if command was not "history"
    if (strcmp(pStr, CLI_HISTORY_CMD)) {
        put_onto_cmd_hist_stk(pStr);
    } else {
        cli_print_hist_stk(); // print stack and restart processing (don't return from the function)
        goto getline_start;
    }
}

static char sLineBuf[CLI_LINEBUF_LENGTH + 1];

// task routine function
void task_cli(void *pParam) {
    size_t len;

    // CLI became initialized now!
    sInitialized = true;

    // notify the world, that CLI is ON!
    MSG("Type '" ANSI_COLOR_BYELLOW "?" ANSI_COLOR_RESET "' to display help!\n");

    while (1) {
        get_line(sLineBuf, &len);
        process_cli_line(sLineBuf);
    }
}

void serial_io_rx_callback(const uint8_t *data, uint32_t size) {
    if (sInitialized) { // push only, if the CLI had been initialized before
        bfifo_push(&sInputFifo, data, size);
    }
}

int cli_register_command(char *pCmdParsHelp, uint8_t cmdTokCnt,
                         uint8_t minArgCnt, fnCliCallback pCB) {
    // if command storage is full, then return -1;
    if (sCliCmdCnt == CLI_MAX_CMD_CNT) {
        return -1;
    }

    // acquire mutex
    osMutexAcquire(modifyMtx, osWaitForever);

    // obtain pointer to first unused command space
    struct CliCommand *pCmd = &spCliCmds[sCliCmdCnt];

    // tokenize the first part of the line (run until cmkTokCnt tokens have been fetched)
    uint32_t tokCnt = 0;
    tokenize_cli_line(pCmdParsHelp, pCmd->ppTok, MAX_TOK_LEN,
                      (cmdTokCnt > TOK_ARR_LEN ? TOK_ARR_LEN : cmdTokCnt), &tokCnt);
    pCmd->tokCnt = (uint8_t)tokCnt;

    // store minimal argument count parameter
    pCmd->minArgCnt = minArgCnt;

    // copy help line
    strncpy(pCmd->pHelp, pCmdParsHelp, MAX_HELP_LEN);

    // zero out hint part (tidy() will fill it)
    pCmd->pHint = NULL;

    // store callback function pointer
    pCmd->pCB = pCB;

    // increase the amount of commands stored
    sCliCmdCnt++;

    // clean up if the same command registered before
    uint8_t i, t;
    int duplicate_idx = -1;
    for (i = 0; i < (sCliCmdCnt - 1); i++) {
        if (spCliCmds[i].tokCnt == cmdTokCnt) {
            for (t = 0; t < cmdTokCnt; t++) {
                if (strcmp(spCliCmds[i].ppTok[t], pCmd->ppTok[t])) {
                    break;
                }
            }

            if (t == cmdTokCnt) {
                duplicate_idx = i;
                break;
            }
        }
    }

    if (duplicate_idx > -1) {
        cli_remove_command(duplicate_idx);
    }

    sCmdsTidy = false; // commands are untidy

    // release mutex
    osMutexRelease(modifyMtx);

    return sCliCmdCnt - 1;
}

void cli_remove_command(int cmdIdx) {
    // acquire mutex
    osMutexAcquire(modifyMtx, osWaitForever);

    if (cmdIdx + 1 > sCliCmdCnt || cmdIdx < 0) {
        return;
    }

    uint8_t i;
    for (i = cmdIdx; i < sCliCmdCnt - 1; i++) {
        memcpy(&spCliCmds[i], &spCliCmds[i + 1], sizeof(struct CliCommand));
    }

    sCliCmdCnt--;

    // release mutex
    osMutexRelease(modifyMtx);
}

// removes bunch of commands, terminated by -1
void cli_remove_command_array(int *pCmdHandle) {
    int *pIter = pCmdHandle;
    while (*pIter != -1) {
        cli_remove_command(*pIter);
        pIter++;
    }
}

// -----------------------

bool get_param_value(const CliToken_Type *ppArgs, uint8_t argc,
                     const char *pKey, char *pVal) {
    size_t i;
    for (i = 0; i < argc; i++) {
        if (!strncmp(ppArgs[i], pKey, strlen(pKey))) {
            strcpy(pVal, ppArgs[i] + strlen(pKey));
            return true;
        }
    }
    return false;
}