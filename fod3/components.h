#pragma once
#include "flecs.h"

ECS_STRUCT(Position, {
float x;
float y;
float z;
});

ECS_STRUCT(Velocity, {
float x;
float y;
float z;
});


ECS_STRUCT(Tracker, {
float h;
});

