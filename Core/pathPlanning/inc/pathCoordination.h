#pragma once
#include "main.h"
#include "regulator.h"

#define POINTS_STACK_SIZE	7

typedef struct {
	uint8_t LocalFlag;
	float PointX;
	float PointY;
	float (* PointVelocity);
}pathPoint;

typedef struct {
	bool PathMoveFlag;
	uint8_t CurPointFlag;
	uint8_t CurDirection;
	uint8_t TargetPointFlag;
	float CurPoint[2];
	float TargPoint[2];
	float LengthTrace;
	float (* TraceVelocity);
	float MiddleTraceError;
	pathPoint Points[POINTS_STACK_SIZE];
}Path;

extern Path PathPlan;

void AddPointInFront(pathPoint *points, float *newPoint, uint8_t type, uint8_t *lastPoint);

void AddPointInBack(pathPoint *points, float *newPoint, uint8_t type, uint8_t *lastPoint);

void RemovePoint(pathPoint *pointsArray, uint8_t *lastPoint);

void CreatePath(pathPoint *next_point, pathPoint *cur_point, Path *output);

void Tracking(void);
