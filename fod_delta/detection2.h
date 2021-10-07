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
	graphics_draw_pointcloud_alpha (g, CE30_WH, x1, alpha, 2000.0f);
	graphics_flush (g);
}








