#pragma once
#include "csc/csc_xlog.h"
#include "misc.h"

/**
 * @brief tracker_update1
 * @param h Intersection hits
 * @param r Radiuses in meter^2
 * @param y Tracker positions
 * @param count
 * @param x Detection coordinate
 * @return The index of tracker that got updated
 */
static uint32_t poitracker_update1 (float h[], float r[], v3f32 y[], uint32_t count, v3f32 const * x)
{
	uint32_t i;
	for (i = 0; i < count; ++i)
	{
		v3f32 d;
		v3f32_sub (&d, x, y + i);
		float r2 = r[i] * r[i];
		//Check if (x) is inside a existing tracker sphere(i):
		if (v3f32_norm2 (&d) < r2)
		{
			// Detection coordinate (x) is inside tracker sphere(i)
			// Set tracker position y[i] to detection coordinate (x)
			y[i] = (*x);
			// Move tracker smootly to the detection coordinate if tracker is already tracking:
			v3f32_mul (&d, &d, -0.5f * (r[i] != FLT_MAX));
			v3f32_add (y + i, y + i, &d); // (y := y + d) where (y) is point, (d) is vector
			// Update tracker state:
			h[i] = MIN (h[i] + TRACKER_HITINCREMENT, 1.0f); // Increase intersect hits
			r[i] = TRACKER_RADIUS;
			break;
		}
	}

	if (i >= count)
	{
		return count;
	}

	//Merge trackers if their sphere intersects:
	//i is old tracker
	//Compare old tracker (i) and with every tracker (j)
	//If i and j intersects remove (j)
	for (uint32_t j = 0; j < count; ++j)
	{
		v3f32 d;
		v3f32_sub (&d, y + i, y + j); //d := y[i] - y[j]
		float l2 = v3f32_norm2 (&d);
		if (j == i) {continue;}
		if (r[j] == FLT_MAX) {continue;}
		// Check if two ball with different radius intersects:
		// (a+b)^2 = a^2 + b^2 + 2ab
		float r2 = r[j]*r[j] + r[i]*r[i] + 2.0f*r[i]*r[j];
		if (l2 > r2) {continue;}

		r[i] = FLT_MAX;
		y[i] = (v3f32){{0.0f, 0.0f, 0.0f}};
		XLOG (XLOG_INF, XLOG_GENERAL, "Merging object tracker %i %i", i, j);
	}

	return i;
}












struct poitracker
{
	uint32_t count;//Not used currently
	float r[TRACKER_CAPACITY];//Radius
	v3f32 x[TRACKER_CAPACITY];//Position
	float h[TRACKER_CAPACITY];//History
	uint32_t i[TRACKER_CAPACITY];//Pointcloud Index
};

static void poitracker_init (struct poitracker * tracker)
{
	memset (tracker, 0, sizeof (struct poitracker));
	vf32_set1 (TRACKER_CAPACITY, tracker->r, FLT_MAX);
	vf32_set1 (TRACKER_CAPACITY, tracker->h, 0.0f);
}


static void poitracker_update (struct poitracker * tracker, v3f32 const * x, int32_t randomi)
{
	uint32_t i = poitracker_update1 (tracker->h, tracker->r, tracker->x, TRACKER_CAPACITY, x);
	if (i < TRACKER_CAPACITY)
	{
		tracker->i[i] = randomi;
		XLOG (XLOG_INF, XLOG_GENERAL, "tracker %i got updated\n", i);
	}
}


static void tracker_update2 (struct poitracker * tracker)
{
	float * h = tracker->h;
	float * r = tracker->r;
	for (uint32_t i = 0; i < TRACKER_CAPACITY; ++i)
	{
		h[i] = MAX (h[i] - TRACKER_DISSIPATION, 0.0f);
		r[i] = (h[i] <= 0.0f) ? FLT_MAX : r[i];
	}
}
