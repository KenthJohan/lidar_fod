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


ECS_STRUCT(Position, {
float x;
float y;
});

ECS_STRUCT(Velocity, {
float x;
float y;
});



void Move(ecs_iter_t *it) {
	Position *p = ecs_term(it, Position, 1);
	const Velocity *v = ecs_term(it, const Velocity, 2);

	for (int i = 0; i < it->count; i ++) {
		p[i].x += v[i].x;
		p[i].y += v[i].y;
	}
}

void PrintPosition(ecs_iter_t *it) {
	const Position *p = ecs_term(it, const Position, 1);

	for (int i = 0; i < it->count; i ++) {
		printf("%s: {%f, %f}\n", ecs_get_name(it->world, it->entities[i]),
		p[i].x, p[i].y);
	}
}



int main (int argc, char const * argv[])
{
	csc_crossos_enable_ansi_color();
	ecs_world_t *world = ecs_init_w_args(1, (char*[]){
	"rest_test", NULL // Application name, optional
	});
	ecs_set(world, EcsWorld, EcsRest, {0});
	ecs_log_set_level(2);
	// This registers the component with reflection data
	ECS_META_COMPONENT(world, Position);
	ECS_META_COMPONENT(world, Velocity);
	//ecs_trace("ecs_app_run '%s:%s'", "rest_test", "true");


	ECS_SYSTEM(world, Move, EcsOnUpdate, Position, [in] Velocity);
	ECS_SYSTEM(world, PrintPosition, EcsPostUpdate, [in] Position);

	Position value = {10, 20};
	char *json = ecs_ptr_to_json(world, ecs_id(Position), &value);
	printf("%s\n", json); // {"x": 10.0, "y": 20.0}
	ecs_os_free(json);


	ecs_entity_t e1 = ecs_new_entity(world, "e1");
	ecs_set(world, e1, Position, {10, 20});
	ecs_set(world, e1, Velocity, {1, 2});

	ecs_entity_t e2 = ecs_new_entity(world, "e2");
	ecs_set(world, e2, Position, {10, 20});
	ecs_set(world, e2, Velocity, {3, 4});


	/*
	return ecs_app_run(world, &(ecs_app_desc_t) {
	.target_fps = 60,
	.enable_rest = true
	});
	*/


	int c = getchar();
	while(c != 'q')
	{
		ecs_progress(world, 0.0f);
		c = getchar();
	}

	return ecs_fini(world);
}
