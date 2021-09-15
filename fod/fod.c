#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>

//git clone https://github.com/nanomsg/nng
//cd nng && mkdir build && cd build
//cmake -G"MSYS Makefiles" .. -DCMAKE_INSTALL_PREFIX="C:\msys64\mingw64"
//pacman -R cmake
//pacman -S mingw-w64-x86_64-cmake
//mingw32-make -j4
//mingw32-make test
//mingw32-make install
//-lnng
#include <nng/nng.h>
#include <nng/protocol/pair0/pair.h>
#include <nng/supplemental/util/platform.h>

#include "csc/csc_debug_nng.h"
#include "csc/csc_crossos.h"
#include "csc/csc_malloc_file.h"
#include "csc/csc_math.h"
#include "csc/csc_linmat.h"
#include "csc/csc_m3f32.h"
#include "csc/csc_m3f32_print.h"
#include "csc/csc_m4f32.h"
#include "csc/csc_v3f32.h"
#include "csc/csc_v3f32_print.h"
#include "csc/csc_v4f32.h"
#include "csc/csc_qf32.h"
#include "csc/csc_filecopy.h"
#include "csc/csc_argv.h"
#include "csc/csc_assert.h"
#include "csc/csc_xlog.h"

#include "../shared/shared.h"
#include "../shared/ce30.h"

#include "mg_send.h"

#include "pointcloud.h"
#include "physobjects.h"
#include "graphics.h"



#define ARG_HELP             UINT32_C(0x00000001)
#define ARG_VERBOSE          UINT32_C(0x00000002)
#define ARG_STDIN            UINT32_C(0x00000010)
#define ARG_LEGACY_FILENAME  UINT32_C(0x00000100)
#define ARG_CTRLMODE         UINT32_C(0x00001000)

struct
{
	char const * address; //Grahpic remote address
	char const * filename; //Load filename
	uint32_t flags; //Misc flags
	uint32_t usleep; //Microseconds sleep each frame
	uint32_t frame; //Start from this frame when loading a file.
} mainarg;


static uint32_t ce30_filter1 (v4f32 x[], v3f32 y[], float w[], float d2)
{
	ASSERT_PARAM_NOTNULL(x);
	ASSERT_PARAM_NOTNULL(y);
	ASSERT_PARAM_NOTNULL(w);
	uint32_t j = 0;
	for (uint32_t i = 0; i < CE30_WH; ++i)
	{
		v3f32 xi = {{x[i].x, x[i].y, x[i].z}};
		if (v3f32_norm2 (&xi) > d2)
		{
			y[j] = xi;
			w[j] = x[i].w;
			j++;
		}
	}
	return j;
}


static uint32_t ce30_fread (v3f32 y[], float w[], FILE * f)
{
	ASSERT_PARAM_NOTNULL(y);
	ASSERT_PARAM_NOTNULL(w);
	ASSERT_PARAM_NOTNULL(f);
	v4f32 x[CE30_WH];
	//pc->framenr = (float)ftell(f) / (float)(sizeof (float) * LIDAR_WH * POINT_STRIDE);
	//printf ("Framenr: %i\n", pc->framenr);
	//sizeof (float) * POINT_STRIDE * LIDAR_WH
	int r = fread (x, sizeof(x), 1, f);
	ASSERTF (r == 1, "fread %i", r);
	uint32_t n = ce30_filter1 (x, y, w, 1.0f);
	return n;
}


void loop_stdin (struct poitracker * pc, struct graphics * g, FILE * f)
{
	v3f32 x[CE30_WH]; //Pointcloud points position
	float a[CE30_WH]; //Pointcloud points amplitude
	uint32_t n;
	while (1)
	{
		n = ce30_fread (x, a, f);
		//XLOG (XLOG_INF, "Number of points: %i\n", n);
		pointcloud_process (g, pc, n, x, a);
	}
}



void loop_file (struct poitracker * pc, struct graphics * g, FILE * f)
{
	v3f32 x[CE30_WH]; //Pointcloud points position
	float a[CE30_WH]; //Pointcloud points amplitude
	uint32_t n;
	int c = '\n';
	while (1)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Frame %i", ce30_ftell(f));
		n = ce30_fread (x, a, f);
		pointcloud_process (g, pc, n, x, a);
		if (mainarg.flags & ARG_CTRLMODE){c = getchar();}
		if (mainarg.usleep){usleep (mainarg.usleep);}
		if (c == 'q'){return;}
	}
}









int main (int argc, char const * argv[])
{
	setbuf(stdout, NULL);
	UNUSED (argc);
	csc_crossos_enable_ansi_color();

	mainarg.address = "tcp://localhost:9002";
	mainarg.filename = NULL;
	mainarg.flags = 0;
	mainarg.usleep = 0;
	mainarg.frame = 0;

	struct csc_argv_option option[] =
	{
	{'h', "help",            CSC_TYPE_U32,    &mainarg.flags,      ARG_HELP,            "Show help"},
	{'v', "verbose",         CSC_TYPE_U32,    &mainarg.flags,      ARG_VERBOSE,         "Show verbose"},
	{'i', "input",           CSC_TYPE_U32,    &mainarg.flags,      ARG_STDIN,           "Get pointcloud from stdin"},
	{'c', "ctrlmode",        CSC_TYPE_U32,    &mainarg.flags,      ARG_CTRLMODE,        "Step forward foreach keypress"},
	{'a', "address",         CSC_TYPE_STRING, &mainarg.address,    0,                   "The MQTT address to send to"},
	{'f', "legacy_filename", CSC_TYPE_STRING, &mainarg.filename,   0,                   "Filename to f32 xyzw 320x20 file"},
	{'F', "frame",           CSC_TYPE_U32,    &mainarg.frame,      0,                   "The starting frame"},
	{'d', "duration",        CSC_TYPE_U32,    &mainarg.usleep,     0,                   "Duration for each frame"},
	{CSC_ARGV_END}};

	csc_argv_parseall (argv+1, option);

	if (mainarg.flags & ARG_HELP)
	{
		csc_argv_description0 (option, stdout);
		csc_argv_description1 (option, stdout);
		return 0;
	}




	struct graphics g;
	g.lines.count = 100;
	g.points.count = CE30_WH*2;
	graphics_init (&g, mainarg.address);

	struct poitracker tracker;
	poitracker_init (&tracker);

	FILE * f = NULL;
	if (mainarg.filename)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Opening binary file %s to read LiDAR frames.", mainarg.filename);
		f = fopen (mainarg.filename, "rb");
		ce30_seek_set (f, mainarg.frame);
	}
	else
	{
		f = stdin;
	}

	ASSERT_NOTNULL (f);

	if (f == stdin)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Reading from STDIN");
		//printf ("[INFO] Reading from STDIN\n");
		loop_stdin (&tracker, &g, f);
	}
	else if (f != NULL)
	{
		XLOG (XLOG_INF, XLOG_GENERAL, "Reading from file %s", mainarg.filename);
		loop_file (&tracker, &g, f);
	}
	else
	{
		return -1;
	}

	nng_close (g.sock);
	return 0;
}
