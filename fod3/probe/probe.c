#include "probe.h"
#include "graphics.h"
#include "csc/csc_f32.h"

static struct graphics probe_graphics;

void probe_init (char const * address)
{
	probe_graphics.lines.count = 200;
	probe_graphics.points.count = CE30_WH*2;
	graphics_init (&probe_graphics, address);
}



void probe_pca (struct fodpca * pca)
{
	graphics_draw_pca (&probe_graphics, pca->e, pca->w, &(pca->o), 2.0f, 2.0f, 2.0f);
}

void probe_pointcloud (v3f32 x[], uint8_t tags[], uint32_t xn)
{
	graphics_draw_pointcloud_cid (&probe_graphics, xn, x, tags);
}

void probe_pointcloud_alpha (v3f32 x[], float const a[], uint32_t xn, float k)
{
	graphics_draw_pointcloud_alpha (&probe_graphics, xn, x, a, k);
}

void probe_pointcloud_pn (v3f32 x[], float const a[], uint32_t xn, float k)
{
	graphics_draw_pointcloud_pn (&probe_graphics, xn, x, a, k);
}

void probe_obj (v3f32 * x, uint32_t type)
{
	u8rgba color;
	switch (type)
	{
	case PROBE_OBJ:
		color = (u8rgba){{0xFF, 0xFF, 0x99, 0xAA}};
		break;
	case PROBE_OBJ1:
		color = (u8rgba){{0xFF, 0x99, 0x99, 0xAA}};
		break;
	}
	graphics_draw_obj (&probe_graphics, x, 0.05f, color);
}





void probe_fodcontext (struct fodcontext * fod)
{
	//graphics_draw_pca (&probe_graphics, fod->pca.e, fod->pca.w, &(fod->pca.o), 2.0f, 2.0f, 2.0f);
	graphics_draw_pointcloud_cid (&probe_graphics, CE30_WH, fod->x1, fod->tags);
	//graphics_draw_pointcloud_cid (&probe_graphics, CE30_WH, fod->x2, fod->tags);
}





void probe_tracker (struct poitracker tracker[], v3f32 x[CE30_WH])
{
	for (uint32_t i = 0; i < TRACKER_CAPACITY; ++i)
	{
		char buf[10];
		//snprintf(buf, 10, "%i:%4.2f", i, tracker->h[i]);
		snprintf (buf, 10, "%i", i);
		graphics_draw_obj (&probe_graphics, tracker->x + i, tracker->r[i], (u8rgba){{0xFF, 0xEE, 0x66, MIN(0xFF * tracker->h[i] * 2.0f, 0xFF)}});
		//graphics_draw_text (g, i, tracker->x + i, buf);
	}
	for (uint32_t i = 0; i < TRACKER_CAPACITY; ++i)
	{
		if (tracker->r[i] != FLT_MAX)
		{
			graphics_draw_line (&probe_graphics, x + tracker->i[i], tracker->x + i, (u8rgba){{0x44, 0xAA, 0xAA, 0xFF}});
			//graphics_draw_obj (g, x + tracker->i[i], tracker->r[i], (u8rgba){{0x00, 0x00, 0x66, 0xFF}});
		}
	}
}

void probe_flush()
{
	graphics_flush (&probe_graphics);
}

void probe_quit()
{
	nng_close (probe_graphics.sock);
}
