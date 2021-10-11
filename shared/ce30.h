#pragma once
#include <stdio.h>
#include <stdint.h>
#include "csc/csc_assert.h"
#include "csc/csc_math.h"


#define CE30_W 320
#define CE30_H 20
#define CE30_WH 6400
#define CE30_XY_INDEX(x,y) ((x)*CE30_H + (y))
#define CE30_FPS 30
#define CE30_FOV_W 60
#define CE30_FOV_H 4
#define CE30_XYZW_FRAME_SIZE (sizeof(float)*4*CE30_WH)


#define CE30_POINT_GOOD 0x10
#define CE30_POINT_EDGE 0x08 //Not used.
#define CE30_POINT_ABOVE 0x04 //Points above ground thus potential objects.
#define CE30_POINT_SECTOR 0x02 //This is a sector from LiDAR origin where objects can be labeled.
#define CE30_POINT_SEARCH 0x01 //This is all points that define PCA.
#define CE30_POINT_IS_REFLECTED(p) ((p).x != 0.0f)
#define CE30_POINT_IS_UNDEFINED(p) ((p).x == 0.0f)

static void ce30_seek_set(FILE * f, long frame)
{
	fseek (f, frame * CE30_XYZW_FRAME_SIZE, SEEK_SET);
}


static uint32_t ce30_ftell(FILE * f)
{
	uint32_t t = (float)ftell(f) / (float)(CE30_XYZW_FRAME_SIZE);
	return t;
}

static void ce30_read (FILE * f, v4f32 xyzqw[CE30_WH], uint32_t n)
{
	for (uint32_t i = 0; i < n; ++i)
	{
		int r = fread (xyzqw, sizeof(v4f32)*CE30_WH, 1, f);
		ASSERTF (r == 1, "fread %i", r);
	}
}


static void ce30_detect_incidence_edges (uint8_t flags[CE30_WH])
{
	// No reflection edge detection:
	// The variable naming (u,v) is used for 2d coordinates:
	for (int32_t u = 1; u < CE30_W-1; ++u)
	for (int32_t v = 1; v < CE30_H-1; ++v)
	{
		// 00 01 02   nw nn ne
		// 10 11 12 = ww xy ee
		// 20 21 22   sw ss se
		int32_t i11 = CE30_XY_INDEX (u ,  v );
		int32_t i00 = CE30_XY_INDEX (u-1, v-1);
		int32_t i01 = CE30_XY_INDEX (u  , v-1);
		int32_t i02 = CE30_XY_INDEX (u+1, v-1);
		int32_t i20 = CE30_XY_INDEX (u-1, v+1);
		int32_t i21 = CE30_XY_INDEX (u  , v+1);
		int32_t i22 = CE30_XY_INDEX (u+1, v+1);
		int32_t i10 = CE30_XY_INDEX (u-1, v  );
		int32_t i12 = CE30_XY_INDEX (u+1, v  );
		if ((flags[i11] & CE30_POINT_GOOD) == 0)
		{
			if (flags[i00] & CE30_POINT_GOOD) {flags[i00] |= CE30_POINT_EDGE;}
			if (flags[i01] & CE30_POINT_GOOD) {flags[i01] |= CE30_POINT_EDGE;}
			if (flags[i02] & CE30_POINT_GOOD) {flags[i02] |= CE30_POINT_EDGE;}
			if (flags[i20] & CE30_POINT_GOOD) {flags[i20] |= CE30_POINT_EDGE;}
			if (flags[i21] & CE30_POINT_GOOD) {flags[i21] |= CE30_POINT_EDGE;}
			if (flags[i22] & CE30_POINT_GOOD) {flags[i22] |= CE30_POINT_EDGE;}
			if (flags[i10] & CE30_POINT_GOOD) {flags[i10] |= CE30_POINT_EDGE;}
			if (flags[i12] & CE30_POINT_GOOD) {flags[i12] |= CE30_POINT_EDGE;}
		}
	}
}


static void ce30_xyzw_to_pos_amp_flags (v4f32 const x[CE30_WH], v3f32 y[CE30_WH], float a[CE30_WH], uint8_t flags[CE30_WH])
{
	// Label good points:
	for (uint32_t i = 0; i < CE30_WH; ++i)
	{
		y[i] = (v3f32){{x[i].x, x[i].y, x[i].z}};
		a[i] = x[i].w;
		if (CE30_POINT_IS_REFLECTED(y[i]))
		{
			flags[i] |= CE30_POINT_GOOD;
		}
	}
}



