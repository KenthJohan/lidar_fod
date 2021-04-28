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
#include "csc/csc_v4f32.h"
#include "csc/csc_qf32.h"
#include "csc/csc_filecopy.h"
#include "csc/csc_argv.h"
#include "csc/csc_debug.h"

#include "../shared/shared.h"
#include "../shared/log.h"

#include "mg_send.h"

#include "pointcloud.h"
#include "physobjects.h"
#include "graphics.h"


#define ARG_HELP             UINT32_C(0x00000001)
#define ARG_VERBOSE          UINT32_C(0x00000002)
#define ARG_STDIN            UINT32_C(0x00000010)
#define ARG_LEGACY_FILENAME  UINT32_C(0x00000100)
#define ARG_CTRLMODE         UINT32_C(0x00001000)


//../txtpoints/4/14_17_18_225279.txt -m1

/*
Frame: 206    Bad PCA plane
Frame: 6000    Bad PCA plane
Frame: 1500
*/




void from_stdin (struct pointcloud * pc, nng_socket sock, FILE * f)
{
	while (1)
	{
		int r = fread (pc->x, 1, sizeof (float) * POINT_STRIDE * LIDAR_WH, f);
		printf ("fread %i\n", r);
		//ASSERTF (r == 1, "fread %i", r);
		pc->n = LIDAR_WH;

		//pointcloud_readfile (pc, f);
		//pointcloud_filter1 (pc, 1);
		graphics_draw_pointcloud (pc, sock);
	}
}



void from_file (struct pointcloud * pc, nng_socket sock, FILE * f, uint32_t arg_flags, uint32_t arg_usleep)
{
	int a = '\n';
	while (1)
	{
		pointcloud_readfile (pc, f);
		pointcloud_filter1 (pc, 1);
		graphics_draw_pointcloud (pc, sock);

		if ((arg_flags & ARG_CTRLMODE) && a == '\n')
		{
			a = getchar();
		}
		if (arg_usleep)
		{
			usleep (arg_usleep);
		}
	}
}










int main (int argc, char const * argv[])
{
	setbuf(stdout, NULL);
	UNUSED (argc);
	csc_crossos_enable_ansi_color();
	//char const * arg_filename = "../txtpoints/4/14_17_18_225279.txt";
	char const * arg_address = "tcp://localhost:9002";
	char const * arg_filename = NULL;
	//char const * arg_address = NULL;
	uint32_t arg_flags = 0;
	uint32_t arg_visualmode = 1;
	uint32_t arg_usleep = 0;
	uint32_t arg_frame = 0;
	struct csc_argv_option option[] =
	{
	{'h', "help",            CSC_TYPE_U32,    &arg_flags,      ARG_HELP,            "Show help"},
	{'v', "verbose",         CSC_TYPE_U32,    &arg_flags,      ARG_VERBOSE,         "Show verbose"},
	{'a', "address",         CSC_TYPE_STRING, &arg_address,    0,                   "The MQTT address to send to"},
	{'f', "legacy_filename", CSC_TYPE_STRING, &arg_filename,   0,                   "Filename to f32 xyzw 320x20 file"},
	{'i', "input",           CSC_TYPE_U32,    &arg_flags,      ARG_STDIN,           "Get pointcloud from stdin"},
	{'m', "mode",            CSC_TYPE_U32,    &arg_visualmode, 0,                   "The visual mode"},
	{'F', "frame",           CSC_TYPE_U32,    &arg_frame,      0,                   "The starting frame"},
	{'c', "ctrlmode",        CSC_TYPE_U32,    &arg_flags,      ARG_CTRLMODE,        "Step forward foreach keypress"},
	{'d', "duration",        CSC_TYPE_U32,    &arg_usleep,     0,                   "Duration for each frame"},
	{CSC_ARGV_END}};

	csc_argv_parseall (argv+1, option);

	if (arg_flags & ARG_HELP)
	{
		csc_argv_description0 (option, stdout);
		csc_argv_description1 (option, stdout);
		return 0;
	}

	nng_socket sock;
	mg_pairdial (&sock, arg_address);
	graphics_init (sock);


	struct pointcloud pc;
	memset (&pc, 0, sizeof(pc));
	pc.n = LIDAR_WH;
	pointcloud_allocate (&pc);

	FILE * f = NULL;
	if (arg_filename)
	{
		printf ("[INFO] Opening binary file %s to read LiDAR frames.\n", arg_filename);
		f = fopen (arg_filename, "rb");
		fseek (f, arg_frame * sizeof (float) * LIDAR_WH * POINT_STRIDE, SEEK_SET);
	}
	else
	{
		f = stdin;
	}

	ASSERT_NOTNULL (f);

	if (f == stdin)
	{
		while (1)
		{
			from_stdin (&pc, sock, f);
		}
	}
	else if (f != NULL)
	{
		while (1)
		{
			from_file (&pc, sock, f, arg_flags, arg_usleep);
		}
	}
	else
	{
		return -1;
	}




	nng_close (sock);
	return 0;
}
