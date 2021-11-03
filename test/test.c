#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>
#include "flecs.h"

#if defined(WIN32)
#include <windows.h>
#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif
#endif

/**
 * @brief https://superuser.com/questions/413073/windows-console-with-ansi-colors-handling
 */
static void csc_crossos_enable_ansi_color ()
{
#if defined(WIN32)
	HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
	DWORD dwMode = 0;
	GetConsoleMode (hOut, &dwMode);
	dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
	SetConsoleMode (hOut, dwMode);
#endif
}

int main (int argc, char const * argv[])
{
	csc_crossos_enable_ansi_color();
	ecs_world_t *world = ecs_init_w_args(1, (char*[]){
	"rest_test", NULL // Application name, optional
	});
	ecs_log_set_level(2);
	ecs_trace("ecs_app_run '%s:%s'", "rest_test", "true");
	return ecs_app_run(world, &(ecs_app_desc_t) {
	.target_fps = 60,
	.enable_rest = true
	});
}
