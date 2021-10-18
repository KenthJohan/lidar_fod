#pragma once
#include <stdint.h>

#define ARG_HELP             UINT32_C(0x00000001)
#define ARG_VERBOSE          UINT32_C(0x00000002)
#define ARG_STDIN            UINT32_C(0x00000010)
#define ARG_LEGACY_FILENAME  UINT32_C(0x00000100)
#define ARG_CTRLMODE         UINT32_C(0x00001000)

struct
{
	char const * address; //Grahpic remote address
	char const * filename; //Load filename
	uint32_t flags; //Misc flags
	uint32_t usleep; //Microseconds sleep each frame
	uint32_t frame; //Start from this frame when loading a file.
} mainarg;
