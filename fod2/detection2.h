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


static void detection_input (struct graphics * g, struct fodcontext * fod)
{
	float p = 0.02f;
	float k = 2.5f * 2.5f;

	v3f32 const * x = fod->pc_input;
	v3f32 * u = fod->pc_mean;
	v3f32 * d = fod->pc_delta;
	float * v = fod->pc_variance;
	float  * l2 = fod->pc_length2;


	for(int32_t i = 0; i < CE30_WH; ++i)
	{
		if (CE30_POINT_IS_UNDEFINED(x[i])){continue;}
		if ((l2[i] / v[i]) < k)
		{
			v3f32_add_mul (u + i, x + i, u + i, p, 1.0f - p); // u[t] = kx + (1-k)u[t-1]
		}
		v[i] = f32_lerp2 (v[i], l2[i], p); // v[t] = kd + (1-k)v[t-1]
		v3f32_sub (d + i, x + i, u + i);
		l2[i] = v3f32_norm2 (d + i);
	}
	//v3f32_subv (d, x, u, 1, 1, 1, CE30_WH);
	//v3f32_dotv (l2, d, d, 1, 1, CE30_WH);


	float * alpha = fod->pc_alpha;
	for(int32_t i = 0; i < CE30_WH; ++i)
	{
		alpha[i] = (l2[i] / v[i]) > k ? 255 : 0;
		//alpha[i] = sqrtf (l2[i] / v[i]) * 255.0f;
		//alpha[i] = sqrtf(v[i]) * 255.0f;
		//alpha[i] = sqrtf(u[i]);
	}


}








