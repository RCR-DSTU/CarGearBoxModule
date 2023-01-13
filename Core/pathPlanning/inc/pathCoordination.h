#pragma once
#include "main.h"

#define POINTS_STACK_SIZE	3

typedef struct {
	float Point[2];
	uint8_t Direction;
	float Speed[2];
}pathPoint;
pathPoint Points[POINTS_STACK_SIZE];


void AddPoint(pathPoint *point);

void RemovePoint();

void CreatePathPlan();

void ClearPathPlan();

