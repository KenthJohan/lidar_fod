#pragma once

#include <flecs.h>

#include <nng/nng.h>

#include "csc_m3f32.h"
#include "csc_m3f32_print.h"
#include "csc_m4f32.h"
#include "csc_v3f32.h"
#include "csc_v3f32_print.h"
#include "csc_v4f32.h"
#include "csc_v4f32_print.h"
#include "csc_qf32.h"
#include "csc_rgb.h"

#include "mg_send.h"
#include "mg_attr.h"
#include "myent.h"

static void sys_draw (ecs_iter_t *it)
{
	v4f32 pos_buf[128];
	v4f32 * pos = pos_buf;
	u8rgba col_buf[128];
	u8rgba * col = col_buf;

	int32_t count = 0;
	m3f32 * e = ecs_term (it, m3f32, 1); //Rotation
	v3f32 * p = ecs_term (it, v3f32, 2);  //Position
	float * l = ecs_term (it, float, 3);  //length
	nng_socket * sock = ecs_term (it, nng_socket, 4);

	u8rgba col_x = {{0xFF, 0xAA, 0xAA, 0xFF}};
	u8rgba col_y = {{0xAA, 0xFF, 0xAA, 0xFF}};
	u8rgba col_z = {{0xAA, 0xAA, 0xFF, 0xFF}};

	printf ("Rotation:\n");
	csc_m3f32_print_rgb (stdout, e);

	for(int32_t i = 0; i < it->count; ++i)
	{
		pos[0] = V4F32_V3 (p[i]);
		pos[2] = V4F32_V3 (p[i]);
		pos[4] = V4F32_V3 (p[i]);
		v3f32_add_mul ((v3f32*)(pos + 1), p + i, e[i].c + 0, 1.0f, l[i]);
		v3f32_add_mul ((v3f32*)(pos + 3), p + i, e[i].c + 1, 1.0f, l[i]);
		v3f32_add_mul ((v3f32*)(pos + 5), p + i, e[i].c + 2, 1.0f, l[i]);
		col[0] = col_x;
		col[1] = col_x;
		col[2] = col_y;
		col[3] = col_y;
		col[4] = col_z;
		col[5] = col_z;
		pos += 6;
		col += 6;
		count += 6;
	}

	/*
	printf ("sys_draw Count: %i\n", count);
	printf ("Length: %i\n", l[0]);
	printf ("Pos:\n");
	csc_v4f32_print_rgb (stdout, pos_buf + 0);
	csc_v4f32_print_rgb (stdout, pos_buf + 1);
	csc_v4f32_print_rgb (stdout, pos_buf + 2);
	csc_v4f32_print_rgb (stdout, pos_buf + 3);
	*/


	mg_send_set (sock[0], MYENT_DRAW_LINES, MG_LINES_POS, pos_buf, sizeof(v4f32)*count);
	mg_send_set (sock[0], MYENT_DRAW_LINES, MG_LINES_COL, col_buf, sizeof(u8rgba)*count);

}
