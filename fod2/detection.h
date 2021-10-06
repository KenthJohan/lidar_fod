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
#include "../shared/ce30.h"


#define BALL_COUNT 1000
#define BALL_RADIUS 0.1f
#define BALL_RADIUS2 (BALL_RADIUS*BALL_RADIUS)

#define BALL_ACTIVE UINT8_C(0x01)

struct detect_ctx
{
	struct
	{
		uint16_t bmap[CE30_WH];
		uint32_t miss[CE30_WH];
	} pointcloud;
	struct
	{
		float br[BALL_COUNT];
		v3f32 bx[BALL_COUNT];
		uint32_t bh[BALL_COUNT];
		uint8_t bf[BALL_COUNT];
	} balls;
	struct
	{
		uint16_t i[BALL_COUNT];
		uint16_t last;
	} waiting;
};


static uint8_t detect_one (struct detect_ctx * ctx, v3f32 * x, uint32_t xi)
{
	ASSERT_LTU (xi, CE30_WH);
	uint16_t bi = ctx->pointcloud.bmap[xi];
	float l2;
	{
		v3f32 d;
		v3f32 * bx = ctx->balls.bx + bi;
		v3f32_sub (&d, bx, x);
		l2 = v3f32_norm2(&d);
	}
	if (l2 > BALL_RADIUS2)
	{
		// Point is outside ball:
		ctx->pointcloud.miss[xi]++;

		ASSERT_LTU (ctx->waiting.last, BALL_COUNT);
		uint32_t i = ctx->waiting.i[ctx->waiting.last];
		ctx->waiting.last -= ctx->waiting.last ? 1 : 0;
		ctx->balls.bx[i] = *x;
		ctx->pointcloud.bmap[xi] = i;
		return 0;
	}
	else
	{
		// Point is inside ball:
		ctx->pointcloud.miss[xi] >>= 1;
		ctx->balls.bh[bi]++; // Increase ball hit
		return POINTLABEL_OBJ;
	}

	/*
	for (uint32_t i = 0; i < BALL_COUNT; ++i)
	{
		if ((ctx->bh[i] == 0) && )
		{

		}
	}
	*/
	return 0;
}


static void detect_input (struct detect_ctx * ctx, v3f32 x[], uint8_t cid[], uint32_t n)
{
	ASSERT_LTEU (n, CE30_WH);
	for (uint32_t i = 0; i < n; ++i)
	{
		cid[i] = detect_one (ctx, x + i, i);
	}
	for (uint32_t i = 0; i < BALL_COUNT; ++i)
	{
		if ((ctx->balls.bh[i] == 0) && (ctx->waiting.last < BALL_COUNT))
		{
			ctx->balls.bh[i] = 1;
			ASSERT_LTU (ctx->waiting.last, BALL_COUNT);
			ctx->waiting.i[ctx->waiting.last] = i;
			ctx->waiting.last++;
		}
	}
}




//struct detect_ctx dctx = {0};


static v3f32 gpc[CE30_WH];


static void detection_input (struct graphics * g, int32_t n, v3f32 x[], float amp[])
{
	UNUSED (amp);
	ASSERT_LTEU (n, CE30_WH);


	uint8_t cid[CE30_WH];
	memset (cid, 0, sizeof(uint8_t)*CE30_WH);
	//detect_input (&dctx, x, cid, n);

	for(int32_t i = 0; i < CE30_WH; ++i)
	{
		float l2;
		{
			v3f32 d;
			v3f32_sub (&d, gpc + i, x + i);
			l2 = v3f32_norm2(&d);
			v3f32_mul (&d, &d, -0.1f);
			v3f32_add (gpc + i, gpc + i, &d);
			//csc_v3f32_print_rgb(&d);
		}
		if (l2 < BALL_RADIUS2)
		{
			cid[i] |= POINTLABEL_OBJ;
		}
	}

	graphics_draw_pointcloud_cid (g, n, x, cid);
	graphics_flush (g);
}








