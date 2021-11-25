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

static void ransac_input(v3f32 x[], uint8_t tags[], uint32_t xn, v3f32 y[], uint32_t n, struct fodpca * pca)
{
	//float l = 6.0f;
	//float lv[10] = {6.0f, 3.0f, 1.0f, 0.5f, 0.8f, 0.9f, 1.0f, 1.5f, 2.0f, 3.0f};
	//float lv[20] = {FLT_MAX, 0.25f, 0.5f, 1.0f, 2.0f, 1.0f, 0.5f, 1.0f, 0.5f, 1.0f, 0.5f, 0.8f, 0.4f, 0.7f, 0.4f, 0.7f, 0.3f, 0.6f, 0.2f, 0.5f};
	//n = 20;
	//float lv[4] = {0.175f, 0.25, 0.5f, 1.0f};
	//float lv[4] = {2.0f, 1.0f, 0.5f, 0.25f};
	//float lv[10] = {4.0f, 2.0f, 1.0f, 0.5f, 0.9f, 0.5f, 0.9f, 0.4f, 0.8f, 0.4f};
	//float lv[10] = {4.0f, 3.0f, 2.0f, 1.0f, 0.9f, 0.8f, 0.7f, 0.6f, 0.5f, 0.4f};
	float lp[4] = {+FLT_MAX, +0.00f  , +0.05f, +0.02f};
	float ln[4] = {-FLT_MAX, -FLT_MAX, -0.1f, -0.1f};
	float kv[4] = {0.1f, 1.0f, 1.0f, 1.0f};
	float ov[4] = {0.0f, 1.0f, 1.0f, 1.0f};
	n = 4;
	for (uint32_t k = 0; k < n; ++k)
	{
		uint32_t yn = 0;
		for (uint32_t i = 0; i < xn; ++i)
		{
			tags[i] &= ~CE30_POINT_GROUND;
			if ((tags[i] & CE30_POINT_GOOD) == 0){continue;}
			float h = y[i].x;
			if(h > lp[k]) {continue;}
			if(h < ln[k]) {continue;}
			y[yn] = x[i];
			yn++;
			tags[i] |= CE30_POINT_GROUND;
		}
		calculate_pca (pca, y, yn, kv[k], ov[k]);
		pointcloud_conditional_basis_flip (pca->e); // Flip eigen vectors if neccecery:
		v3f32_subv (x, x, &pca->o, 1, 1, 0, xn);
		pointcloud_rotate ((m3f32 *)pca->e, x, y, xn);
		v3f32_addv (x, x, &pca->o, 1, 1, 0, xn);
		printf("n=%i, l=(%f,%f) w0=%f\n", yn, lp[k], ln[k], pca->w[0]);
		probe_pointcloud (x, tags, xn);
		probe_pca(pca);
		probe_flush();
		getchar();
	}
}


static void ransac2(v3f32 x1[], v3f32 x2[], uint8_t tags[], uint32_t xn, struct fodpca * pca)
{
	struct fodpca pca1 = *pca;
	memset(x2, 0, sizeof(v3f32)*xn);
	ransac_input (x1, tags, xn, x2, 1, &pca1);

	pca->e[0] = pca1.e[0];
	pca->e[1] = pca1.e[1];
	pca->e[2] = pca1.e[2];
	pca->w[0] = pca1.w[0];
	pca->w[1] = pca1.w[1];
	pca->w[2] = pca1.w[2];
	float k = 0.01f;
	v3f32_add_mul (&(pca->o), &(pca->o), &(pca1.o), k, 1.0f - k);
	m3f32_add_mul (&(pca->c), &(pca->c), &(pca1.c), k, 1.0f - k);

	for (uint32_t i = 0; i < xn; ++i)
	{
		if ((tags[i] & CE30_POINT_GOOD) == 0){continue;}
		float h = x2[i].x;
		tags[i] &= ~CE30_POINT_ABOVE;
		if (h > sqrtf(pca->w[0])*5.0f)
		{
			tags[i] |= CE30_POINT_ABOVE;
		}
	}
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
	ransac2 (fod->x1, fod->x2, fod->tags, CE30_WH, &(fod->pca));
	probe_fodcontext (fod);
	probe_flush();
}

