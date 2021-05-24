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


#define GRAPHICVERTS_MAXITEMS 100000
struct graphicverts
{
	uint32_t count;
	uint32_t last;
	v4f32 * pos;
	u8rgba * col;
};


static void graphicverts_allocate (struct graphicverts * g)
{
	g->last = 0;
	ASSERT_PARAM_NOTNULL (g);
	ASSERT (g->count < GRAPHICVERTS_MAXITEMS);
	g->pos = calloc (sizeof(v4f32) * g->count, 1);
	g->col = calloc (sizeof(u8rgba) * g->count, 1);
	ASSERT_NOTNULL (g->pos);
	ASSERT_NOTNULL (g->col);
	XLOG (XLOG_INF, "Count: %i\n", g->count);
}

static void graphicverts_reserve (struct graphicverts * g, uint32_t n)
{
	ASSERT_PARAM_NOTNULL (g);
	ASSERT (g->count < GRAPHICVERTS_MAXITEMS);
	ASSERTF (g->last + n < GRAPHICVERTS_MAXITEMS, "g->last = %i, n = %i", g->last, n);
	ASSERT (g->last + n < g->count);
	g->last += n;
}

static void graphicverts_push (struct graphicverts * g, uint32_t n, v4f32 p[], u8rgba c[])
{
	ASSERT_PARAM_NOTNULL (g);
	ASSERT_PARAM_NOTNULL (p);
	ASSERT_PARAM_NOTNULL (c);
	ASSERT (g->count < GRAPHICVERTS_MAXITEMS);
	ASSERT (g->last + n < GRAPHICVERTS_MAXITEMS);
	ASSERT (g->last + n < g->count);
	memcpy (g->pos + g->last, p, n * sizeof(v4f32));
	memcpy (g->col + g->last, p, n * sizeof(u8rgba));
	g->last += n;
}



struct graphics
{
	nng_socket sock;
	struct graphicverts lines;
	struct graphicverts points;
};


static void graphics_init (struct graphics * g, char const * address)
{
	ASSERT_PARAM_NOTNULL (g);
	graphicverts_allocate (&g->lines);
	graphicverts_allocate (&g->points);
	XLOG (XLOG_INF, "mg_pairdial remote graphic server %s\n", address);
	mg_pairdial (&g->sock, address);
	nng_socket sock = g->sock;
	mg_send_add (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD);
	mg_send_add (sock, MYENT_DRAW_LINES, MG_LINES);

	{
		component_count points_count = g->points.count;
		component_count lines_count = g->lines.count;
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_CAPACITY, &points_count, sizeof(component_count));
		mg_send_set (sock, MYENT_DRAW_LINES, MG_CAPACITY, &lines_count, sizeof(component_count));
	}

	{
		//The color of each point. This is only used for visualization.
		uint32_t color[CE30_WH*1];
		vu32_set1 (CE30_WH*1, color+CE30_WH*0, 0xFFFFFF00);
		//vu32_set1 (LIDAR_WH*1, pointcol+LIDAR_WH*1, 0xFFFFFF88);
		//vu32_set1 (LIDAR_WH*1, pointcol+LIDAR_WH*2, 0xFF88FFFF);
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_COL, color, CE30_WH*sizeof(uint32_t)*1);
	}

	{
		//The color of each point. This is only used for visualization.
		v4f32 lines[6];
		uint32_t col[6];
		vu32_set1 (6, col, 0xFFFFFFFF);
		mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_COL, col, sizeof(col));
		mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_POS, lines, sizeof(lines));
	}
}



static void graphics_flush (struct graphics * g)
{
	ASSERT_PARAM_NOTNULL (g);
	nng_socket sock = g->sock;
	/*
	mg_send_set (sock, MYENT_DRAW_CLOUD, MG_COUNT, &(component_count){g->points.count}, sizeof(component_count));
	mg_send_set (sock, MYENT_DRAW_LINES, MG_COUNT, &(component_count){g->lines.count}, sizeof(component_count));
	*/
	mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_POS, g->points.pos, sizeof(v4f32)*g->points.last);
	mg_send_set (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD_COL, g->points.col, sizeof(u8rgba)*g->points.last);
	mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_POS, g->lines.pos, sizeof(v4f32)*g->lines.last);
	mg_send_set (sock, MYENT_DRAW_LINES, MG_LINES_COL, g->lines.col, sizeof(u8rgba)*g->lines.last);

	g->points.last = 0;
	g->lines.last = 0;
}









/*
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
*/


static void graphics_draw_pointcloud (struct graphics * g, uint32_t n, v3f32 x[], float a[])
{
	uint32_t last = g->points.last;
	v4f32 * pos = g->points.pos + last;
	u8rgba * col = g->points.col + last;
	graphicverts_reserve (&g->points, n);

	//Set color
	for (uint32_t i = 0; i < n; ++i)
	{
		float w;
		if (a)
		{
			w = CLAMP(a[i] * 4.0f, 0.0f, 255.0f);
		}
		else
		{
			w = 255.0f;
		}
		col[i].r = (uint8_t)(w);
		col[i].g = (uint8_t)(w);
		col[i].b = (uint8_t)(w);
		col[i].a = 0xFF;
		//struct csc_u8rgba c = {.r = 0x44, .g = 0x44, .b = 0x44, .a = 0xFF};
		//pointcol[i] = c;
	}


	for (uint32_t i = 0; i < n; ++i)
	{
		pos[i].w = 10.0f;
		pos[i].x = x[i].x;
		pos[i].y = x[i].y;
		pos[i].z = x[i].z;
	}

}




static void graphics_draw_pca (struct graphics * g, v3f32 e[3], float w[3], v3f32 * c)
{
	uint32_t last = g->lines.last;
	v4f32 * pos = g->lines.pos + last;
	u8rgba * col = g->lines.col + last;
	graphicverts_reserve (&g->lines, 6);

	v4f32_set_xyzw (pos + 0, c->x, c->y, c->z, 0.0f);
	v4f32_set_xyzw (pos + 2, c->x, c->y, c->z, 0.0f);
	v4f32_set_xyzw (pos + 4, c->x, c->y, c->z, 0.0f);
	v3f32_add_mul ((v3f32*)(pos + 1), c, e + 0, 1.0, sqrtf(w[0]));
	v3f32_add_mul ((v3f32*)(pos + 3), c, e + 1, 1.0, sqrtf(w[1]));
	v3f32_add_mul ((v3f32*)(pos + 5), c, e + 2, 1.0, sqrtf(w[2]));

	u8rgba col_x = {{0xFF, 0xAA, 0xAA, 0xFF}};
	u8rgba col_y = {{0xAA, 0xFF, 0xAA, 0xFF}};
	u8rgba col_z = {{0xAA, 0xAA, 0xFF, 0xFF}};
	col[0] = col_x;
	col[1] = col_x;
	col[2] = col_y;
	col[3] = col_y;
	col[4] = col_z;
	col[5] = col_z;
}































