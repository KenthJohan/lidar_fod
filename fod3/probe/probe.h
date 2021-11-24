#pragma once

#include "types.h"
#include "tracker.h"



void probe_init (char const * address);
void probe_fodcontext (struct fodcontext * fod);
void probe_tracker (struct poitracker tracker[], v3f32 x[CE30_WH]);
void probe_flush ();
void probe_quit();

