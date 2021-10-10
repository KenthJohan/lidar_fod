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


#define FODPOINT_GOOD 0x10

struct fodcontext
{
	v4f32 pc_src[CE30_WH];
	v3f32 pc_x0[CE30_WH];
	v3f32 pc_x1[CE30_WH];
	v3f32 pc_xd[CE30_WH];
	float pc_amplitude1[CE30_WH];
	uint8_t cid[CE30_WH];
	float pc_alpha[CE30_WH];
};

static void fodcontext_read (struct fodcontext * fod, FILE * f)
{
	ASSERT_PARAM_NOTNULL(fod);
	ASSERT_PARAM_NOTNULL(f);
	v3f32 * x1 = fod->pc_x1;
	uint8_t * cid = fod->cid;
	float * a1 = fod->pc_amplitude1;
	for (uint32_t i = 0; i < 5; ++i)
	{
		int r = fread (fod->pc_src, sizeof(v4f32)*CE30_WH, 1, f);
		ASSERTF (r == 1, "fread %i", r);
	}
	memset (cid, 0, sizeof(uint8_t)*CE30_WH);
	for (uint32_t i = 0; i < CE30_WH; ++i)
	{
		v4f32 * x = fod->pc_src + i;
		v3f32 y = {{x->x, x->y, x->z}};
		float l = v3f32_norm2 (&y);
		if (l > (1.0f * 1.0f))
		{
			x1[i] = y;
			a1[i] = fod->pc_src[i].w;
			cid[i] |= FODPOINT_GOOD;
		}
	}


	for(int32_t x = 1; x < CE30_W-1; ++x)
	for(int32_t y = 1; y < CE30_H-1; ++y)
	{
		// 00 01 02   nw nn ne
		// 10 11 12 = ww xy ee
		// 20 21 22   sw ss se
		int32_t i11 = CE30_XY_INDEX(x ,  y );
		// 00 01 02:
		int32_t i00 = CE30_XY_INDEX(x-1, y-1);
		int32_t i01 = CE30_XY_INDEX(x  , y-1);
		int32_t i02 = CE30_XY_INDEX(x+1, y-1);
		// 20 21 22:
		int32_t i20 = CE30_XY_INDEX(x-1, y+1);
		int32_t i21 = CE30_XY_INDEX(x  , y+1);
		int32_t i22 = CE30_XY_INDEX(x+1, y+1);
		// 10 12:
		int32_t i10 = CE30_XY_INDEX(x-1, y  );
		int32_t i12 = CE30_XY_INDEX(x+1, y  );
		if (v3f32_norm2(x1 + i11) < 1.0f)
		{
			if (v3f32_norm2 (x1 + i00) > 1.0f){cid[i00] |= POINTLABEL_EDGE;}
			if (v3f32_norm2 (x1 + i01) > 1.0f){cid[i01] |= POINTLABEL_EDGE;}
			if (v3f32_norm2 (x1 + i02) > 1.0f){cid[i02] |= POINTLABEL_EDGE;}
			if (v3f32_norm2 (x1 + i20) > 1.0f){cid[i20] |= POINTLABEL_EDGE;}
			if (v3f32_norm2 (x1 + i21) > 1.0f){cid[i21] |= POINTLABEL_EDGE;}
			if (v3f32_norm2 (x1 + i22) > 1.0f){cid[i22] |= POINTLABEL_EDGE;}
			if (v3f32_norm2 (x1 + i10) > 1.0f){cid[i10] |= POINTLABEL_EDGE;}
			if (v3f32_norm2 (x1 + i12) > 1.0f){cid[i12] |= POINTLABEL_EDGE;}
		}
	}

}


