
/* This file was automatically generated by nrfutil on 2017-07-07 (YY-MM-DD) at 07:50:19 */

#include "stdint.h"
#include "compiler_abstraction.h"

/* This file was generated with a throwaway private key, that is only inteded for a debug version of the DFU project.
  Please see https://github.com/NordicSemiconductor/pc-nrfutil/blob/master/README.md to generate a valid public key. */

#ifdef NRF_DFU_DEBUG_VERSION 

/** @brief Public key used to verify DFU images */
__ALIGN(4) const uint8_t pk[64] =
{
    0xaf, 0x93, 0xbc, 0x65, 0x39, 0xac, 0x65, 0x42, 0xaa, 0x0d, 0xa9, 0x05, 0xf7, 0xca, 0x78, 0x90, 0xfa, 0x2b, 0x55, 0x57, 0xb0, 0xa3, 0xf2, 0x9c, 0x09, 0x2b, 0x61, 0x3c, 0xc7, 0x67, 0x39, 0x09, 
    0xd3, 0x86, 0x02, 0xda, 0xc7, 0xdd, 0xf4, 0x05, 0x46, 0x0e, 0xbb, 0x55, 0x7d, 0xf7, 0xe1, 0x5c, 0x7c, 0x9c, 0x4e, 0x4f, 0x36, 0xb7, 0xab, 0x9c, 0xe7, 0xd1, 0x13, 0x33, 0x87, 0xc0, 0x4b, 0xb8
};

#else
#error "Debug public key not valid for production. Please see https://github.com/NordicSemiconductor/pc-nrfutil/blob/master/README.md to generate it"
#endif