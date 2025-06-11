#include "standard_output.h"
#include "cmsis_os2.h"
#include "standard_output/serial_io.h"

#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include <embfmt/embformat.h>

#define STDIO_OUTPUT_LINEBUF_LEN (2048)
static char lineBuf[STDIO_OUTPUT_LINEBUF_LEN + 1];

static uint32_t insert_carriage_return(char *str, uint32_t maxLen) {
    uint32_t len = strlen(str);
    for (uint32_t i = 0; (i < len) && (len <= maxLen); i++) {
        // if a \n is found and no \r precedes it or it is the first character os the string
        if ((str[i] == '\n') && ((i == 0) || (str[i - 1] != '\r'))) {
            len++;                               // increase length
            for (uint32_t k = len; k > i; k--) { // copy each character one position right
                str[k] = str[k - 1];
            }
            str[i] = '\r'; // insert '\r'
            i++;           // skip the current \r\n block
        }
    }
    str[len] = '\0';
    return len;
}

static char last_char = '\n';

void MSG(const char *format, ...) {
    va_list vaArgP;
    va_start(vaArgP, format);
    uint32_t lineLen = vembfmt(lineBuf, STDIO_OUTPUT_LINEBUF_LEN, format, vaArgP);
    va_end(vaArgP);
    lineLen = insert_carriage_return(lineBuf, STDIO_OUTPUT_LINEBUF_LEN); // insert carriage returns
    if (lineLen > 0) {
        serial_io_write(lineBuf, lineLen); // send line to the CDC
        last_char = lineBuf[lineLen - 1];
    }
}

void MSGchar(int c) {
    serial_io_write((const char *)&c, 1);
}

void MSGraw(const char *str) {
    serial_io_write(str, strlen(str));
}
