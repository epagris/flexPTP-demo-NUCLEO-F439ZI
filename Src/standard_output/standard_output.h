#ifndef STANDARD_OUTPUT_STANDRAD_OUTPUT
#define STANDARD_OUTPUT_STANDRAD_OUTPUT

#include "term_colors.h"

void MSG(const char *format, ...); // custom printf() replacement
void MSGchar(int c); // print a single character
void MSGraw(const char * str); // print a raw string

#endif /* STANDARD_OUTPUT_STANDRAD_OUTPUT */
