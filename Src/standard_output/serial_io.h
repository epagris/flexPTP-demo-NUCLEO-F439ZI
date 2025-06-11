#ifndef STANDARD_OUTPUT_SERIAL_OUTPUT
#define STANDARD_OUTPUT_SERIAL_OUTPUT

#include <stdint.h>

void serial_io_init();
void serial_io_write(const char * str, uint32_t len);

#endif /* STANDARD_OUTPUT_SERIAL_OUTPUT */
