#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>

#include "csc/csc_assert.h"
#include "csc/csc_f32.h"
#include "csc/csc_m3f32.h"
#include "csc/csc_m3f32_print.h"
#include "csc/csc_v3f32.h"
#include "csc/csc_v3f32_print.h"
#include "csc/csc_v4f32.h"
#include "csc/csc_xlog.h"

#include "../shared/ce30.h"

#include "misc.h"
#include "graphics.h"
#include "pointcloud.h"
#include "pcfod.h"


static void detection_input (struct graphics * g, struct fodcontext * fod, float k, int keybtn)
{
	// Rename:
	v3f32 * x0 = fod->pc_x0;
	v3f32 const * x1 = fod->pc_x1;
	v3f32 * xd = fod->pc_xd;
	float * alpha = fod->pc_alpha;
	uint8_t * flags = fod->pc_flags;
	// Reset:
	memset (flags, 0, sizeof(uint8_t)*CE30_WH);

	// Label points near zero as zero:
	for(int32_t i = 0; i < CE30_WH; ++i)
	{
		float l0 = v3f32_norm2(x0 + i);
		float l1 = v3f32_norm2(x1 + i);
		if (l0 < (1.0f*1.0f))
		{
			flags[i] |= FODFLAG_ZERO;
		}
		if (l1 < (1.0f*1.0f))
		{
			flags[i] |= FODFLAG_ZERO;
		}
	}

	for(int32_t x = 1; x < CE30_W-1; ++x)
	for(int32_t y = 1; y < CE30_H-1; ++y)
	{
		// 00 01 02
		// 10 11 12
		// 20 21 22
		int32_t i = CE30_XY_INDEX(x, y);
		int32_t i00 = CE30_XY_INDEX(x-1, y-1);
		int32_t i01 = CE30_XY_INDEX(x  , y-1);
		int32_t i02 = CE30_XY_INDEX(x+1, y-1);

		int32_t i20 = CE30_XY_INDEX(x-1, y+1);
		int32_t i21 = CE30_XY_INDEX(x  , y+1);
		int32_t i22 = CE30_XY_INDEX(x+1, y+1);

		int32_t i10 = CE30_XY_INDEX(x-1, y);
		int32_t i12 = CE30_XY_INDEX(x+1, y);

		if (v3f32_norm2(x1 + i) < 1.0f)
		{
			if (v3f32_norm2 (x1 + i00) > 1.0f){flags[i00] |= POINTLABEL_OBJ;}
			if (v3f32_norm2 (x1 + i01) > 1.0f){flags[i01] |= POINTLABEL_OBJ;}
			if (v3f32_norm2 (x1 + i02) > 1.0f){flags[i02] |= POINTLABEL_OBJ;}
			if (v3f32_norm2 (x1 + i20) > 1.0f){flags[i20] |= POINTLABEL_OBJ;}
			if (v3f32_norm2 (x1 + i21) > 1.0f){flags[i21] |= POINTLABEL_OBJ;}
			if (v3f32_norm2 (x1 + i22) > 1.0f){flags[i22] |= POINTLABEL_OBJ;}
			if (v3f32_norm2 (x1 + i10) > 1.0f){flags[i10] |= POINTLABEL_OBJ;}
			if (v3f32_norm2 (x1 + i12) > 1.0f){flags[i12] |= POINTLABEL_OBJ;}
		}


	}



	// Calculate position delta (xd = x1 - x0):
	v3f32_subv (xd, x1, x0, 1, 1, 1, CE30_WH);
	memcpy (x0, x1, sizeof(v3f32) * CE30_WH);

	for(int32_t i = 0; i < CE30_WH; ++i)
	{
		if (flags[i] & FODFLAG_ZERO)
		{
			alpha[i] = 0.0f;
		}
		else
		{
			float l = v3f32_norm (xd + i);
			//  alpha[i] = (alpha[i] * (1.0f - k)) + (l * k);
			alpha[i] = l;
		}
	}

	if (keybtn == 'p')
	{
		float amin = FLT_MAX;
		float amax = FLT_MIN;
		float aavg = 0.0f;
		float j = 0.0f;
		for(int32_t i = 0; i < CE30_WH; ++i)
		{
			if ((flags[i] & FODFLAG_ZERO) == 0)
			{
				if (alpha[i] > 0.0f)
				{
					//printf ("%05i: %f\n", i, v3f32_norm(x1 + i));
					//printf ("%05i: %f\n", i, alpha[i]);
					amin = MIN(alpha[i], amin);
					amax = MAX(alpha[i], amax);
					aavg += alpha[i];
					j += 1.0f;
				}
			}
		}
		printf ("(%f, %f), avg: %f\n", amin, amax, aavg/j);
	}
	//graphics_draw_pointcloud_alpha (g, CE30_WH, x1, alpha, 2000.0f);
	graphics_draw_pointcloud_cid (g, CE30_WH, x1, flags);
	graphics_flush (g);
}








