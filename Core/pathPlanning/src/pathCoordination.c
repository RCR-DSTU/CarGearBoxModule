#include "pathCoordination.h"

/*!
 * Measurement table of transmission
 * 0 - N flag
 * 1 - R flag
 * 2 - F1 flag
 */
static float const MEASUREMENT_X[3] = {0, 0.25, -0.25};
static float const MEASUREMENT_Y[3] = {0, 0.25, -0.25};

Path PathPlan;
/*!
 * VelocityMap[3] - { VerySlow, Slow, Middle, Fast, VeryFast }
 * [Value] m/s
 */
static float VelocityMap[5] = {0.1, 0.3, 0.5, 0.7, 1.0};
/*!
 *
 * @param pointsArray
 * @param lastPoint
 */
void RemovePoint(pathPoint *points, uint8_t *lastPoint)
{
	int8_t i, j;
	for(i = 0; i < *lastPoint; i++)
	{
		for(j = 0; j < sizeof(pathPoint); j++)
			*(((char *)(&points[i])) + j) = *(((char *)(&points[i+1])) + j);
	}
	if(*lastPoint > 0)
	{
		for(j = 0; j < sizeof(pathPoint); j++)
			*(((char *)(&points[*lastPoint])) + j) = 0;
		    (*lastPoint)--;
	}
}

/*!
 *
 * @param points
 * @param newPoint
 * @param type
 * @param lastPoint
 */
void AddPointInFront(pathPoint *points, float *newPoint,
		uint8_t type, uint8_t *lastPoint)
{
	int8_t i,j;
	for(i = *lastPoint; i >= 0; i--)
	{
		if(!((i + 1) > POINTS_STACK_SIZE))
		{
			for(j = 0; j < sizeof(pathPoint); j++)
	            *(((char *)(&points[i + 1])) + j) = *(((char *)(&points[i])) + j);
		}
	}
	points[0].PointVelocity = &VelocityMap[type];
	points[0].PointX = *newPoint;
	points[0].PointY = *(newPoint + 1);

	(*lastPoint)++;
}

/*!
 *
 * @param points
 * @param newPoint
 * @param type
 * @param lastPoint
 */
void AddPointInBack(pathPoint *points, float *newPoint,
		uint8_t type, uint8_t *lastPoint)
{
	int8_t i,j;
	for(i = 0; i <= *lastPoint; i++)
	{
		if(!((i - 1) < 0))
		{
			for(j = 0; j < sizeof(pathPoint); j++)
				*(((char *)(&points[i])) + j) = *(((char *)(&points[i + 1])) + j);
		}
	}

	points[POINTS_STACK_SIZE - 1].PointVelocity = &VelocityMap[type];
	points[POINTS_STACK_SIZE - 1].PointX = *newPoint;
	points[POINTS_STACK_SIZE - 1].PointY = *(newPoint + 1);

	(*lastPoint)--;
}

/*!
 *
 * @param next_point
 * @param cur_point
 * @param output
 */
void CreatePath(pathPoint *next_point, pathPoint *cur_point, Path *output)
{




}

/*!
 *
 * @param output
 */
void ZeroPlanning(Path *output)
{
	output->PathMoveFlag = false;
	output->TargPoint[0] = 0.0;
	output->TargPoint[1] = 0.0;

	float delta[2] = { 0.0, 0.0 };

	delta[0] = output->CurPoint[0] - output->TargPoint[0];
	delta[1] = output->CurPoint[1] - output->TargPoint[1];

	/* Clearing points table */
	for(int i = 0; sizeof(output->Points); i++)
		*((char *)(&output->Points[i])) = 0;

	if(delta[1] != 0.0)
	{
		float point1[2] = { 0.0, delta[1] };
		AddPointInFront(&(*output->Points), &(*point1), 3, 0);
	}
	if(delta[0] != 0.0)
	{
		float point2[2] = { delta[0], 0.0 };
		AddPointInFront(&(*output->Points), &(*point1), 3, 1);
	}

	output->PathMoveFlag = true;
}
