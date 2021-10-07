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
#include "tracker.h"
#include "pointcloud.h"
#include "pcfod.h"
#include "../shared/ce30.h"


static void detection_input (struct graphics * g, struct fodcontext * fod, float k)
{
	//Rename
	v3f32 * x0 = fod->pc_x0;
	v3f32 * x1 = fod->pc_x1;
	v3f32 * xd = fod->pc_xd;
	float * alpha = fod->pc_alpha;
	uint8_t * cid = fod->cid;

	//Reset
	//memset (cid, 0, sizeof(uint8_t)*CE30_WH);

	//Calculate position delta (xd = x1 - x0)
	v3f32_subv (xd, x1, x0, 1, 1, 1, CE30_WH);
	memcpy (x0, x1, sizeof(v3f32) * CE30_WH);

	for(int32_t i = 0; i < CE30_WH; ++i)
	{
		float v = v3f32_norm2(xd + i) * 40000.0f;
		alpha[i] = (alpha[i] * (1.0f - k)) + (v * k);
	}
	graphics_draw_pointcloud_alpha (g, CE30_WH, x1, alpha);
	graphics_flush (g);
}








