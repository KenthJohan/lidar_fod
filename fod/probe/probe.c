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


void probe_fodcontext (struct fodcontext * fod, v3f32 x[CE30_WH])
{
	//csc_v3f32_print_rgb(e);
	printf ("sample_delta:         %+5.5f (%+5.2f deg)\n", fod->sample_delta, f32_rad_to_deg(acos(fod->sample_delta)));
	printf ("sample_mean_variance: %+5.5f (%+5.2f deg)\n", fod->sample_mean_variance, f32_rad_to_deg(acos(fod->sample_mean_variance)));
	printf ("sample_mean_roll      %+5.5f\n", f32_rad_to_deg(fod->sample_mean_roll));
	printf ("sample_mean_elevation %+5.5f\n", f32_rad_to_deg(fod->sample_mean_elevation));
	printf ("sample  eigenval:  %+5.5f %+5.5f %+5.5f\n", fod->pca_sample.w[0], fod->pca_sample.w[1], fod->pca_sample.w[2]);
	printf ("cluster eigenval:  %+5.5f %+5.5f %+5.5f\n", fod->pca_cluster.w[0], fod->pca_cluster.w[1], fod->pca_cluster.w[2]);
	printf ("proximity: ");
	for (uint32_t i = 0; i < PROX_SECTOR_COUNT; ++i) {printf ("%i ", fod->proximity[i]);}
	printf ("\n");

	//graphics_draw_pointcloud_alpha (g, n, x1, amp);
	graphics_draw_pca (&probe_graphics, fod->pca_sample.e, fod->pca_sample.w, &(fod->pca_sample.o), DETECT_MIN_EIGEN_RATIO, DETECT_MIN_EIGEN_RATIO, DETECT_MIN_EIGEN_RATIO);
	//graphics_draw_pca (&probe_graphics, fod->pca_sample.e, fod->pca_sample.w, &(fod->pca_sample.o), 20.0f, 20.0f, 20.0f);
	graphics_draw_pca (&probe_graphics, fod->pca_cluster.e, fod->pca_cluster.w, &(fod->pca_cluster.o), 2.0f, 2.0f, 2.0f);
	graphics_draw_pca (&probe_graphics, fod->pca_ground.e, fod->pca_ground.w, &(fod->pca_ground.o), 20.0f, 20.0f, 20.0f);
	//graphics_draw_obj (g, x + fod->clusteri, 0.1f, (u8rgba){{0xCC, 0xEE, 0xFF, 0xFF}});
	graphics_draw_obj (&probe_graphics, x + fod->sample_index, 0.05f, (u8rgba){{0x99, 0x33, 0xFF, 0xAA}});
	graphics_draw_pointcloud_cid (&probe_graphics, CE30_WH, x, fod->pc_tags);
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
