#pragma once
#include <stdio.h>
#include "csc/csc_math.h"
#include "csc/csc_v3f32.h"
#include "csc/csc_debug.h"

#define CE30_W 320
#define CE30_H 20
#define CE30_WH 6400
#define CE30_XY_INDEX(x,y) ((x)*CE30_H + (y))
#define CE30_FPS 30
#define CE30_FOV_W 60
#define CE30_FOV_H 4
#define CE30_XYZW_FRAME_SIZE (sizeof(float)*4*CE30_WH)


static uint32_t ce30_filter1 (v4f32 x[], v3f32 y[], float w[], float d2)
{
	ASSERT_PARAM_NOTNULL(x);
	ASSERT_PARAM_NOTNULL(y);
	ASSERT_PARAM_NOTNULL(w);
	uint32_t j = 0;
	for (uint32_t i = 0; i < CE30_WH; ++i)
	{
		v3f32 xi = {{x[i].x, x[i].y, x[i].z}};
		if (v3f32_norm2 (&xi) > d2)
		{
			y[j] = xi;
			w[j] = x[i].w;
			j++;
		}
	}
	return j;
}


static uint32_t ce30_fread (v3f32 y[], float w[], FILE * f)
{
	ASSERT_PARAM_NOTNULL(y);
	ASSERT_PARAM_NOTNULL(w);
	ASSERT_PARAM_NOTNULL(f);
	v4f32 x[CE30_WH];
	//pc->framenr = (float)ftell(f) / (float)(sizeof (float) * LIDAR_WH * POINT_STRIDE);
	//printf ("Framenr: %i\n", pc->framenr);
	//sizeof (float) * POINT_STRIDE * LIDAR_WH
	int r = fread (x, sizeof(x), 1, f);
	ASSERTF (r == 1, "fread %i", r);
	uint32_t n = ce30_filter1 (x, y, w, 1.0f);
	return n;
}


static void ce30_seek_set(FILE * f, long frame)
{
	fseek (f, frame * CE30_XYZW_FRAME_SIZE, SEEK_SET);
}


static uint32_t ce30_ftell(FILE * f)
{
	uint32_t t = (float)ftell(f) / (float)(CE30_XYZW_FRAME_SIZE);
	return t;
}
