#pragma once

#include "types.h"
#include "tracker.h"


#define PROBE_OBJ 1
#define PROBE_OBJ1 2


void probe_init (char const * address);
void probe_fodcontext (struct fodcontext * fod);
void probe_obj (v3f32 * x, uint32_t type);
void probe_pca (struct fodpca * pca);
void probe_pointcloud (v3f32 x[], uint8_t tags[], uint32_t xn);
void probe_pointcloud_alpha (v3f32 x[], float const a[], uint32_t xn, float k);
void probe_pointcloud_pn (v3f32 x[], float const a[], uint32_t xn, float k);
void probe_tracker (struct poitracker tracker[], v3f32 x[CE30_WH]);
void probe_flush ();
void probe_quit();

