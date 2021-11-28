#pragma once

#include "types.h"


#define PROBE_OBJ 1
#define PROBE_OBJ1 2


void probe_init (char const * address);
void probe_fodcontext (struct fodcontext * fod);
void probe_obj (v3f32 * x, uint32_t type, int i);
void probe_pca (struct fodpca * pca);
void probe_pointcloud (v3f32 x[], uint8_t tags[], uint32_t xn);
void probe_pointcloud_alpha (v3f32 x[], float const a[], uint32_t xn, float k);
void probe_pointcloud_pn (v3f32 x[], float const a[], uint32_t xn, float k);
void probe_flush ();
void probe_quit();

