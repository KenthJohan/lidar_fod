#pragma once

#include <stdio.h>

#include "csc_debug_nng.h"
#include "csc_math.h"
#include "csc_linmat.h"
#include "csc_m3f32.h"
#include "csc_m3f32_print.h"
#include "csc_v3f32_print.h"
#include "csc_vu32.h"
#include "csc_rgb.h"

#include "../shared/shared.h"
#include "../shared/ce30.h"

#include "mathmisc.h"

#include "mg_attr.h"
#include "mg_comp.h"
#include "mg_send.h"
#include "myent.h"

#include "pointcloud.h"

static void graphics_init (nng_socket sock)
{
	mg_send_add (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD);
	mg_send_add (sock, MYENT_DRAW_LINES, MG_LINES);


	{
		component_count c = CE30_WH*1;
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_COUNT, &c, sizeof(component_count));
	}


	{
		//The color of each point. This is only used for visualization.
		uint32_t pointcol[CE30_WH*1];
		vu32_set1 (CE30_WH*1, pointcol+CE30_WH*0, 0xFFFFFF00);
		//vu32_set1 (LIDAR_WH*1, pointcol+LIDAR_WH*1, 0xFFFFFF88);
		//vu32_set1 (LIDAR_WH*1, pointcol+LIDAR_WH*2, 0xFF88FFFF);
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_COL, pointcol, CE30_WH*sizeof(uint32_t)*1);
	}

	{
		//The color of each point. This is only used for visualization.
		v4f32 lines[6];
		uint32_t col[6];
		component_count c = 6;
		vu32_set1 (6, col, 0xFFFFFFFF);
		mg_send_set (sock, MYENT_DRAW_LINES, MG_COUNT, &c, sizeof(c));
		mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_COL, col, sizeof(col));
		mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_POS, lines, sizeof(lines));
	}

}


static void graphics_draw_pointcloud (struct pointcloud * pc, nng_socket sock)
{
	u8rgba pointcol[CE30_WH*1];
	for (uint32_t i = 0; i < CE30_WH; ++i)
	{
		pointcol[i].a = 0x00;
	}

	//Set color
	for (uint32_t i = 0; i < pc->n; ++i)
	{
		float w = CLAMP(pc->a[i] * 4.0f, 0.0f, 255.0f);
		pointcol[i].r = (uint8_t)(w);
		pointcol[i].g = (uint8_t)(w);
		pointcol[i].b = (uint8_t)(w);
		pointcol[i].a = 0xFF;
		//struct csc_u8rgba c = {.r = 0x44, .g = 0x44, .b = 0x44, .a = 0xFF};
		//pointcol[i] = c;
	}




	//mg_send_set (sock, MYENT_DRAW_LINES, MG_POINTCLOUD_POS, x, sizeof(struct v4f32)*LIDAR_WH);
	v4f32 x[CE30_WH];
	//Set point size
	for (uint32_t i = 0; i < CE30_WH; ++i)
	{
		x[i].w = 0.0f;
	}
	for (uint32_t i = 0; i < pc->n; ++i)
	{
		x[i].w = 10.0f;
		x[i].x = pc->x1[i].x;
		x[i].y = pc->x1[i].y;
		x[i].z = pc->x1[i].z;
	}


	mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_POS, x, sizeof(v4f32)*CE30_WH);
	mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_COL, pointcol, sizeof(uint32_t)*CE30_WH);
}




static void graphics_draw_pca (struct pointcloud * pc, nng_socket sock)
{
	v3f32 const * e = pc->e; //Rename
	v4f32 pos[6];
	v4f32_set1     (pos + 0, 0.0f);
	v4f32_set1     (pos + 2, 0.0f);
	v4f32_set1     (pos + 4, 0.0f);

	v4f32_set_xyzw (pos + 1, e[0].x, e[0].y, e[0].z, 0.0f);
	v4f32_set_xyzw (pos + 3, e[1].x, e[1].y, e[1].z, 0.0f);
	v4f32_set_xyzw (pos + 5, e[2].x, e[2].y, e[2].z, 0.0f);

	//https://math.stackexchange.com/questions/1447730/drawing-ellipse-from-eigenvalue-eigenvector
	v4f32_mul (pos + 1, pos + 1, sqrtf(pc->w[0]));
	v4f32_mul (pos + 3, pos + 3, sqrtf(pc->w[1]));
	v4f32_mul (pos + 5, pos + 5, sqrtf(pc->w[2]));

	u8rgba col_x = {0xFF, 0xAA, 0xAA, 0xFF};
	u8rgba col_y = {0xAA, 0xFF, 0xAA, 0xFF};
	u8rgba col_z = {0xAA, 0xAA, 0xFF, 0xFF};
	u8rgba col[6] = {col_x,col_x, col_y,col_y, col_z,col_z};

	mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_POS, pos, sizeof(pos));
	mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_COL, col, sizeof(col));
}































