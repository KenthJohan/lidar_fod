#pragma once

#include <stdio.h>

#include "csc/csc_math.h"
#include "csc/csc_linmat.h"
#include "csc/csc_m3f32.h"
#include "csc/csc_m3f32_print.h"
#include "csc/csc_v3f32_print.h"
#include "csc/csc_vu32.h"
#include "csc/csc_rgb.h"
#include "csc/csc_xlog.h"

#include "../shared/ce30.h"
#include "../shared/mg_attr.h"
#include "../shared/mg_comp.h"
#include "../shared/mg_send.h"

#include "misc.h"
#include "myent.h"
#include "fodcontext.h"


#define GRAPHICVERTS_MAXITEMS 100000
struct graphicverts
{
	uint32_t count;
	uint32_t last;
	v4f32 * pos;
	u8rgba * col;
};


static void graphicverts_allocate (struct graphicverts * g, char const * name)
{
	ASSERT_PARAM_NOTNULL (g);
	ASSERT (g->count < GRAPHICVERTS_MAXITEMS);
	g->last = 0;
	g->pos = calloc (sizeof(v4f32) * g->count, 1);
	g->col = calloc (sizeof(u8rgba) * g->count, 1);
	ASSERT_NOTNULL (g->pos);
	ASSERT_NOTNULL (g->col);
	XLOG (XLOG_INF, XLOG_GENERAL, "Allocated %i %s vertices", g->count, name ? name : "");
}

static void graphicverts_reserve (struct graphicverts * g, uint32_t n)
{
	ASSERT_PARAM_NOTNULL (g);
	ASSERT (g->count < GRAPHICVERTS_MAXITEMS);
	ASSERTF (g->last + n < GRAPHICVERTS_MAXITEMS, "g->last = %i, n = %i", g->last, n);
	ASSERTF (g->last + n <= g->count, "%i <= %i", g->last + n, g->count);
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
	ASSERT_PARAM_NOTNULL (address);
	graphicverts_allocate (&g->lines, "Lines");
	graphicverts_allocate (&g->points, "Points");
	XLOG (XLOG_INF, XLOG_GENERAL, "mg_pairdial remote graphic server %s", address);
	mg_pairdial (&g->sock, address);
	nng_socket sock = g->sock;
	mg_send_add (sock, MYENT_DRAW_CLOUD, MG_POINTCLOUD);
	mg_send_add (sock, MYENT_DRAW_LINES, MG_LINES);

	{
		Capacity points_count = g->points.count;
		Capacity lines_count = g->lines.count;
		mg_send_set (sock, MYENT_DRAW_CLOUD, MG_CAPACITY, &points_count, sizeof(Capacity));
		mg_send_set (sock, MYENT_DRAW_LINES, MG_CAPACITY, &lines_count, sizeof(Capacity));
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


static u8rgba graphics_cid (uint8_t tag)
{
	if (tag & CE30_POINT_OBJ)
	{
		return (u8rgba) {.r = 0xEE, .g = 0xEE, .b = 0x66, .a = 0xFF};
	}

	if (tag & CE30_POINT_ABOVE)
	{
		return (u8rgba) {.r = 0x44, .g = 0xCC, .b = 0x44, .a = 0xFF};
	}

	if (tag & CE30_POINT_EDGE)
	{
		return (u8rgba) {.r = 0xDe, .g = 0xDf, .b = 0xB2, .a = 0xFF};
	}

	if (tag & CE30_POINT_GROUND)
	{
		return (u8rgba) {.r = 0x8e, .g = 0x8f, .b = 0x72, .a = 0xFF};
	}

	if (tag & CE30_POINT_SEARCH)
	{
		return (u8rgba) {.r = 0xBB, .g = 0xBB, .b = 0xBB, .a = 0xFF};
	}

	if (tag & CE30_POINT_SECTOR)
	{
		return (u8rgba) {.r = 0x66, .g = 0x66, .b = 0x66, .a = 0xFF};
	}

	return (u8rgba) {.r = 0x00, .g = 0x00, .b = 0x00, .a = 0xFF};
}


static void graphics_draw_pointcloud_cid (struct graphics * g, uint32_t n, v3f32 const x[], uint8_t const cid[])
{
	uint32_t last = g->points.last;
	v4f32 * pos = g->points.pos + last;
	u8rgba * col = g->points.col + last;
	graphicverts_reserve (&g->points, n);
	for (uint32_t i = 0; i < n; ++i)
	{
		col[i] = graphics_cid (cid[i]);
		col[i].a = 0xFF;
		pos[i].w = 10.0f;
		pos[i].x = x[i].x;
		pos[i].y = x[i].y;
		pos[i].z = x[i].z;
	}
}


static void graphics_draw_pointcloud_alpha (struct graphics * g, uint32_t n, v3f32 const x[], float const a[], float k)
{
	uint32_t last = g->points.last;
	v4f32 * pos = g->points.pos + last;
	u8rgba * col = g->points.col + last;
	graphicverts_reserve (&g->points, n);
	for (uint32_t i = 0; i < n; ++i)
	{
		float w;
		w = CLAMP(a[i] * k, 0.0f, 255.0f);
		col[i].r = (uint8_t)(w);
		col[i].g = (uint8_t)(w);
		col[i].b = (uint8_t)(w);
		col[i].a = 0xFF;
		pos[i].w = 10.0f;
		pos[i].x = x[i].x;
		pos[i].y = x[i].y;
		pos[i].z = x[i].z;
	}
}


static void graphics_draw_pointcloud_pn (struct graphics * g, uint32_t n, v3f32 const x[], float const a[], float k)
{
	uint32_t last = g->points.last;
	v4f32 * pos = g->points.pos + last;
	u8rgba * col = g->points.col + last;
	graphicverts_reserve (&g->points, n);
	for (uint32_t i = 0; i < n; ++i)
	{
		uint8_t f = 50;
		float w = a[i] * k;
		w = CLAMP(w, -127.0f, 127.0f) + 127.0f;
		if(w < 0.0f)
		{
			col[i].r = (uint8_t)(w);
			col[i].g = f;
			col[i].b = f;
		}
		else if(w > 0.0f)
		{
			col[i].r = f;
			col[i].g = (uint8_t)(w);
			col[i].b = f;
		}
		col[i].a = 0xFF;
		pos[i].w = 10.0f;
		pos[i].x = x[i].x;
		pos[i].y = x[i].y;
		pos[i].z = x[i].z;
	}
}




static void graphics_draw_pca (struct graphics * g, v3f32 const e[3], float const w[3], v3f32 const * c, float kx, float ky, float kz)
{
	uint32_t last = g->lines.last;
	v4f32 * pos = g->lines.pos + last;
	u8rgba * col = g->lines.col + last;
	graphicverts_reserve (&g->lines, 12);
	u8rgba col_x = {{0xFF, 0x55, 0x55, 0x99}};
	u8rgba col_y = {{0x55, 0xFF, 0x55, 0x99}};
	u8rgba col_z = {{0x55, 0x55, 0xFF, 0x99}};

	v4f32_set_xyzw (pos + 0, c->x, c->y, c->z, 0.0f);
	v4f32_set_xyzw (pos + 2, c->x, c->y, c->z, 0.0f);
	v4f32_set_xyzw (pos + 4, c->x, c->y, c->z, 0.0f);
	v3f32_add_mul ((v3f32*)(pos + 1), c, e + 0, 1.0, sqrtf(w[0])*kx);
	v3f32_add_mul ((v3f32*)(pos + 3), c, e + 1, 1.0, sqrtf(w[1])*ky);
	v3f32_add_mul ((v3f32*)(pos + 5), c, e + 2, 1.0, sqrtf(w[2])*kz);
	col[0] = col_x;
	col[1] = col_x;
	col[2] = col_y;
	col[3] = col_y;
	col[4] = col_z;
	col[5] = col_z;



	v4f32_set_xyzw (pos + 6, c->x, c->y, c->z, 0.0f);
	v4f32_set_xyzw (pos + 8, c->x, c->y, c->z, 0.0f);
	v4f32_set_xyzw (pos + 10, c->x, c->y, c->z, 0.0f);
	v3f32_add_mul ((v3f32*)(pos + 7), c, e + 0, 1.0, -sqrtf(w[0])*kx);
	v3f32_add_mul ((v3f32*)(pos + 9), c, e + 1, 1.0, -sqrtf(w[1])*ky);
	v3f32_add_mul ((v3f32*)(pos + 11), c, e + 2, 1.0, -sqrtf(w[2])*kz);
	col[6] = col_x;
	col[7] = col_x;
	col[8] = col_y;
	col[9] = col_y;
	col[10] = col_z;
	col[11] = col_z;


}





static void graphics_draw_obj (struct graphics * g, v3f32 const * x, float r, u8rgba color)
{
	uint32_t last = g->lines.last;
	v4f32 * pos = g->lines.pos + last;
	u8rgba * col = g->lines.col + last;
	graphicverts_reserve (&g->lines, 6);
	v4f32_set_xyzw (pos + 0, x->x-r, x->y, x->z, 0.0f);
	v4f32_set_xyzw (pos + 1, x->x+r, x->y, x->z, 0.0f);
	v4f32_set_xyzw (pos + 2, x->x, x->y-r, x->z, 0.0f);
	v4f32_set_xyzw (pos + 3, x->x, x->y+r, x->z, 0.0f);
	v4f32_set_xyzw (pos + 4, x->x, x->y, x->z-r, 0.0f);
	v4f32_set_xyzw (pos + 5, x->x, x->y, x->z+r, 0.0f);
	col[0] = color;
	col[1] = color;
	col[2] = color;
	col[3] = color;
	col[4] = color;
	col[5] = color;
}

static void graphics_draw_line (struct graphics * g, v3f32 * a, v3f32 * b, u8rgba color)
{
	uint32_t last = g->lines.last;
	v4f32 * pos = g->lines.pos + last;
	u8rgba * col = g->lines.col + last;
	graphicverts_reserve (&g->lines, 2);
	v4f32_set_xyzw (pos + 0, a->x, a->y, a->z, 0.0f);
	v4f32_set_xyzw (pos + 1, b->x, b->y, b->z, 0.0f);
	col[0] = color;
	col[1] = color;
}




static void graphics_draw_text (struct graphics * g, int i, Position3 * p, char const * text)
{
	Scale2 s = {{0.005f, 0.005f}};
	ASSERT (i < MYENT_TEXT_LAST);
	mg_send_set (g->sock, MYENT_TEXT0 + i, MG_SCALE2, &s, sizeof (Scale2));
	mg_send_set (g->sock, MYENT_TEXT0 + i, MG_POSITION3, p, sizeof (Position3));
	mg_send_set (g->sock, MYENT_TEXT0 + i, MG_TEXT, text, strlen (text));
}



























