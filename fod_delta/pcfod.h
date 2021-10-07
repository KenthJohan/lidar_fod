#pragma once
#include "csc/csc_xlog.h"
#include "csc/csc_crossos.h"
#include "csc/csc_malloc_file.h"
#include "csc/csc_filecopy.h"
#include "csc/csc_argv.h"
#include "csc/csc_assert.h"
#include "csc/csc_xlog.h"
#include "csc/csc_math.h"
#include "misc.h"
#include "../shared/ce30.h"

#define FODFLAG_ZERO 0x01

struct fodcontext
{
	v4f32 pc_src[CE30_WH];
	v3f32 pc_x0[CE30_WH];
	v3f32 pc_x1[CE30_WH];
	v3f32 pc_xd[CE30_WH];
	float pc_amplitude1[CE30_WH];
	uint8_t pc_flags[CE30_WH];
	float pc_alpha[CE30_WH];
};

static void fodcontext_read (struct fodcontext * fod, FILE * f)
{
	for (uint32_t i = 0; i < 5; ++i)
	{
		int r = fread (fod->pc_src, sizeof(v4f32)*CE30_WH, 1, f);
		ASSERTF (r == 1, "fread %i", r);
	}
	for (uint32_t i = 0; i < CE30_WH; ++i)
	{
		fod->pc_x1[i].x = fod->pc_src[i].x;
		fod->pc_x1[i].y = fod->pc_src[i].y;
		fod->pc_x1[i].z = fod->pc_src[i].z;
		fod->pc_amplitude1[i] = fod->pc_src[i].w;
	}
}
