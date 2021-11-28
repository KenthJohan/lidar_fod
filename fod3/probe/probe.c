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

void probe_obj (v3f32 * x, uint32_t type, int i)
{
	u8rgba color;
	switch (type)
	{
	case PROBE_OBJ:
		color = (u8rgba){{0xCC, 0xCC, 0x55, 0xFF}};
		break;
	case PROBE_OBJ1:
		color = (u8rgba){{0xCC, 0x55, 0x55, 0xFF}};
		break;
	}
	graphics_draw_obj (&probe_graphics, x, 0.05f, color);
	if(i >= 0 && i < MYENT_TEXT_LAST)
	{
		//char buf[10];
		//snprintf (buf, 10, "%i", i);
		//graphics_draw_text (&probe_graphics, i, x, buf);
	}
}





void probe_fodcontext (struct fodcontext * fod)
{
	//graphics_draw_pca (&probe_graphics, fod->pca.e, fod->pca.w, &(fod->pca.o), 2.0f, 2.0f, 2.0f);
	graphics_draw_pointcloud_cid (&probe_graphics, CE30_WH, fod->x1, fod->tags);
	//graphics_draw_pointcloud_cid (&probe_graphics, CE30_WH, fod->x2, fod->tags);
}



void probe_flush()
{
	graphics_flush (&probe_graphics);
}

void probe_quit()
{
	nng_close (probe_graphics.sock);
}
