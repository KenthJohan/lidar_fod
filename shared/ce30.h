#pragma once
#include <stdio.h>
#include <stdint.h>


#define CE30_W 320
#define CE30_H 20
#define CE30_WH 6400
#define CE30_XY_INDEX(x,y) ((x)*CE30_H + (y))
#define CE30_FPS 30
#define CE30_FOV_W 60
#define CE30_FOV_H 4
#define CE30_XYZW_FRAME_SIZE (sizeof(float)*4*CE30_WH)





static void ce30_seek_set(FILE * f, long frame)
{
	fseek (f, frame * CE30_XYZW_FRAME_SIZE, SEEK_SET);
}


static uint32_t ce30_ftell(FILE * f)
{
	uint32_t t = (float)ftell(f) / (float)(CE30_XYZW_FRAME_SIZE);
	return t;
}
