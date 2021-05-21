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

#include "csc_debug_nng.h"
#include "csc_crossos.h"
#include "csc_malloc_file.h"
#include "csc_math.h"
#include "csc_linmat.h"
#include "csc_m3f32.h"
#include "csc_m3f32_print.h"
#include "csc_m4f32.h"
#include "csc_v3f32.h"
#include "csc_v3f32_print.h"
#include "csc_v4f32.h"
#include "csc_qf32.h"
#include "csc_filecopy.h"
#include "csc_argv.h"
#include "csc_debug.h"

#include "../shared/shared.h"
#include "../shared/log.h"
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
	char const * address;
	char const * filename;
	uint32_t flags;
	uint32_t visualmode;
	uint32_t usleep;
	uint32_t frame;
} mainarg;


//../txtpoints/4/14_17_18_225279.txt -m1

/*
Frame: 206    Bad PCA plane
Frame: 6000    Bad PCA plane
Frame: 1500
*/

void loop1 (struct pointcloud * pc, struct graphics * g)
{
	pointcloud_process (pc);
	graphics_draw_pca (g, pc->e, pc->w, &pc->o);
	graphics_draw_pointcloud (g, pc->n, pc->a, pc->x);
	graphics_draw_pointcloud (g, pc->n, pc->a, pc->x1);
	//graphics_draw_pca (pc, sock);
	//csc_v3f32_print_rgb (stdout, &pc->o);
	graphics_flush (g);
}



void loop_stdin (struct pointcloud * pc, struct graphics * g, FILE * f)
{
	while (1)
	{
		pc->n = ce30_fread (pc->x, pc->a, f);
		XLOG (XLOG_INF, "Number of points: %i\n", pc->n);
		//pointcloud_readfile (pc, f);
		loop1 (pc, g);
	}
}



void loop_file (struct pointcloud * pc, struct graphics * g, FILE * f)
{
	int a = '\n';
	while (1)
	{
		XLOG (XLOG_INF, "Frame %i\n", ce30_ftell(f));
		pc->n = ce30_fread (pc->x, pc->a, f);
		XLOG (XLOG_INF, "Number of points: %i\n", pc->n);
		loop1 (pc, g);
		if (mainarg.flags & ARG_CTRLMODE)
		{
			a = getchar();
		}
		if (mainarg.usleep)
		{
			usleep (mainarg.usleep);
		}
		if (a == 'q')
		{
			return;
		}
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
	mainarg.visualmode = 1;
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
	{'m', "mode",            CSC_TYPE_U32,    &mainarg.visualmode, 0,                   "The visual mode"},
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



	XLOG (XLOG_INF, "Init remote graphic server %s\n", mainarg.address);
	struct graphics g;
	g.lines.count = 12;
	g.points.count = CE30_WH*2;
	graphics_init (&g, mainarg.address);

	struct pointcloud pc;
	memset (&pc, 0, sizeof(pc));
	pc.capacity = CE30_WH;
	pointcloud_allocate (&pc);

	FILE * f = NULL;
	if (mainarg.filename)
	{
		XLOG (XLOG_INF, "Opening binary file %s to read LiDAR frames.\n", mainarg.filename);
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
		XLOG (XLOG_INF, "Reading from STDIN");
		//printf ("[INFO] Reading from STDIN\n");
		loop_stdin (&pc, &g, f);
	}
	else if (f != NULL)
	{
		XLOG (XLOG_INF, "Reading from file %s\n", mainarg.filename);
		loop_file (&pc, &g, f);
	}
	else
	{
		return -1;
	}

	nng_close (g.sock);
	return 0;
}
