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
#include "sys_draw.h"

#include <flecs.h>


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

ecs_world_t * world = NULL;

void loop1(struct pointcloud * pc, nng_socket sock)
{
	pointcloud_process (pc);
	graphics_draw_pointcloud (pc, sock);
	//graphics_draw_pca (pc, sock);
	ecs_progress (world, 0);
	printf ("Number of points: %i\n", pc->n);
	//csc_v3f32_print_rgb (stdout, &pc->o);
}



void loop_stdin (struct pointcloud * pc, nng_socket sock, FILE * f)
{
	while (1)
	{
		pc->n = ce30_fread (pc->x, pc->a, f);
		//pointcloud_readfile (pc, f);
		loop1 (pc, sock);
	}
}



void loop_file (struct pointcloud * pc, nng_socket sock, FILE * f, uint32_t arg_flags, uint32_t arg_usleep)
{
	int a = '\n';
	while (1)
	{
		printf ("Frame %i\n", ce30_ftell(f));
		pc->n = ce30_fread (pc->x, pc->a, f);
		loop1 (pc, sock);
		if (arg_flags & ARG_CTRLMODE)
		{
			a = getchar();
		}
		if (arg_usleep)
		{
			usleep (arg_usleep);
		}
		if (a == 'q')
		{
			return;
		}
	}
}



typedef m3f32 Rotation;
typedef v3f32 Position;
typedef v3f32 Velocity;
typedef float Length;
typedef struct GraphicServer
{
	nng_socket socket;
} GraphicServer;






int main (int argc, char const * argv[])
{
	setbuf(stdout, NULL);
	UNUSED (argc);
	csc_crossos_enable_ansi_color();



	world = ecs_init();
	ECS_COMPONENT(world, Rotation);
	ECS_COMPONENT(world, Position);
	ECS_COMPONENT(world, Velocity);
	ECS_COMPONENT(world, Length);
	ECS_COMPONENT(world, GraphicServer);
	ECS_SYSTEM (world, sys_draw, EcsOnUpdate, [in] Rotation, [in] Position, [in] Length, [in] $GraphicServer);




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



	printf ("[INFO] Init remote graphic server %s\n", arg_address);
	nng_socket sock;
	mg_pairdial (&sock, arg_address);
	graphics_init (sock);
	ecs_singleton_set (world, GraphicServer, {sock});

	ecs_entity_t e = ecs_new(world, 0);
	//ecs_add (world, e, Position);
	//ecs_add (world, e, Rotation);
	//ecs_add (world, e, Length);
	ecs_set (world, e, Position, {{1.0f, 1.0f, 1.0f}});
	ecs_set (world, e, Rotation, {{0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}});
	ecs_set (world, e, Length, {0.5f});

	struct pointcloud pc;
	memset (&pc, 0, sizeof(pc));
	pc.capacity = CE30_WH;
	pointcloud_allocate (&pc);

	FILE * f = NULL;
	if (arg_filename)
	{
		printf ("[INFO] Opening binary file %s to read LiDAR frames.\n", arg_filename);
		f = fopen (arg_filename, "rb");
		ce30_seek_set (f, arg_frame);
	}
	else
	{
		f = stdin;
	}

	ASSERT_NOTNULL (f);

	if (f == stdin)
	{
		printf ("[INFO] Reading from STDIN\n");
		loop_stdin (&pc, sock, f);
	}
	else if (f != NULL)
	{
		printf ("[INFO] Reading from file %s\n", arg_filename);
		loop_file (&pc, sock, f, arg_flags, arg_usleep);
	}
	else
	{
		return -1;
	}



	ecs_fini (world);
	nng_close (sock);
	return 0;
}
