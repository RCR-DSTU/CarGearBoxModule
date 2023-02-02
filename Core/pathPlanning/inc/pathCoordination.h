#pragma once
#include "main.h"
#include "regulator.h"

#define POINTS_STACK_SIZE	2

typedef struct {
	uint8_t Direction;
	float PointX;
	float PointY;
	float (* PointVelocity);
}pathPoint;

typedef struct {
	bool MoveFlag;
	int8_t TargetTransmissionFlag;
	pathPoint Points[POINTS_STACK_SIZE];
}Path;

extern Path PathPlan;

void AddPointInFront(pathPoint *points, float *newPoint, uint8_t type, uint8_t *lastPoint);

void AddPointInBack(pathPoint *points, float *newPoint, uint8_t type, uint8_t *lastPoint);

void RemovePoint(pathPoint *pointsArray, uint8_t *lastPoint);

void CreatePath(pathPoint *next_point, pathPoint *cur_point, Path *output);

void Tracking(void);

void ZeroPlanning(Path *output);
