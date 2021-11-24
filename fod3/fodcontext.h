#pragma once

#include "csc/csc_math.h"
#include "../shared/ce30.h"
#include "probe/probe.h"
#include "types.h"

/*
struct ransac
{
	uint32_t capacity;
	uint32_t count;
	v3f32 * x;
	v3f32 * y;
};


static void ransac_init(struct ransac * r)
{
	r->x = calloc (r->capacity, sizeof (v3f32));
	r->y = calloc (r->capacity, sizeof (v3f32));
	r->count = r->capacity;
}
*/

static void ransac_input(v3f32 x1[], v3f32 x2[], uint8_t tags[], uint32_t xn, uint32_t n, struct fodpca * pca, struct fodpca * pca1)
{
	uint32_t j = 0;
	for (uint32_t i = 0; i < xn; ++i)
	{
		if ((tags[i] & CE30_POINT_GOOD) == 0){continue;}
		x2[j] = x1[i];
		j++;
	}

	//float l = 6.0f;
	//float lv[10] = {6.0f, 3.0f, 1.0f, 0.5f, 0.8f, 0.9f, 1.0f, 1.5f, 2.0f, 3.0f};
	float lv[19] = {0.25f, 0.5f, 1.0f, 2.0f, 1.0f, 0.5f, 1.0f, 0.5f, 1.0f, 0.5f, 0.8f, 0.4f, 0.7f, 0.4f, 0.7f, 0.3f, 0.6f, 0.2f, 0.5f};
	n = 19;
	*pca1 = *pca;
	for (uint32_t k = 0; k < n; ++k)
	{
		calculate_pca (pca1, x2, j, 1.0f);
		pointcloud_conditional_basis_flip (pca1->e); // Flip eigen vectors if neccecery:
		v3f32_subv (x1, x1, &pca1->o, 1, 1, 0, CE30_WH);
		pointcloud_rotate ((m3f32 *)pca1->e, x1, x2, xn);
		v3f32_addv (x1, x1, &pca1->o, 1, 1, 0, CE30_WH);
		j = 0;
		for (uint32_t i = 0; i < xn; ++i)
		{
			if ((tags[i] & CE30_POINT_GOOD) == 0){continue;}
			float h = x2[i].x;
			if (fabs(h) < 0.1f*lv[k])
			{
				x2[j] = x1[i];
				j++;
			}
		}
		printf("j=%i, l=%f w0=%f\n", j, lv[k], pca1->w[0]);
	}




	calculate_pca (pca, x2, j, 1.0f);
	v3f32_subv (x1, x1, &pca->o, 1, 1, 0, CE30_WH);
	pointcloud_rotate ((m3f32 *)pca->e, x1, x2, xn);
	v3f32_addv (x1, x1, &pca->o, 1, 1, 0, CE30_WH);
	for (uint32_t i = 0; i < xn; ++i)
	{
		if ((tags[i] & CE30_POINT_GOOD) == 0){continue;}
		float h = x2[i].x;
		if (h > (sqrtf(pca->w[0])*2.0f))
		{
			tags[i] |= CE30_POINT_ABOVE;
		}
	}


	/*
	for (uint32_t i = 0; i < xn; ++i)
	{
		tags[i] &= ~CE30_POINT_ABOVE;
	}

	for (uint32_t i = 0; i < xn; ++i)
	{
		float h = x2[i].x;
		if (h > 0.1f)
		{
			tags[i] |= CE30_POINT_ABOVE;
		}
	}
	*/
}


static struct fodcontext * fodcontext_create()
{
	struct fodcontext * fodctx = calloc (1, sizeof (struct fodcontext));
	return fodctx;
}

static void fodcontext_input (struct fodcontext * fod, v4f32 xyzw[CE30_WH])
{
	memset(fod->tags, 0, sizeof(uint8_t)*CE30_WH);
	ce30_xyzw_to_pos_amp_flags (xyzw, fod->x1, fod->a1, fod->tags);
	ce30_detect_incidence_edges (fod->tags);
	struct fodpca pca1;
	ransac_input (fod->x1, fod->x2, fod->tags, CE30_WH, 14, &(fod->pca), &pca1);
	probe_fodcontext (fod);
	probe_flush();
}

