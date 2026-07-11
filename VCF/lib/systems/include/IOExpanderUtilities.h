#ifndef IO_EXPANDER_UTILITIES_H
#define IO_EXPANDER_UTILITIES_H

/* Standard Library Includes */
#include <stdint.h>
#include <stdbool.h>


namespace IOExpanderUtilities
{
    /**
    IOExpander's read() only reads.
    getBit() only get specified bit from previously read dataframe and does not read()
    @param data data from which to get the specified bit
    @param port port from which to get the bit from (A=0, B=1)
    @param bit  bit number of port to get bit from
    */
    bool getBit(uint16_t data, bool port, uint8_t bit);
}

#endif