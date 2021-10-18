#pragma once

#include "pcfod.h"
#include "tracker.h"



void probe_init (char const * address);
void probe_fodcontext (struct fodcontext * fod, v3f32 x[CE30_WH], uint32_t randomi);
void probe_tracker (struct poitracker tracker[], v3f32 x[CE30_WH]);
void probe_quit();
