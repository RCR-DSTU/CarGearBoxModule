#include "pathCoordination.h"


bool MoveFlag;
Path PathPlan;
/*!
 * VelocityMap[3] - { VerySlow, Slow, Middle, Fast, VeryFast }
 * [Value] m/s
 */
static float VelocityMap[5] = {0.1, 0.3, 0.5, 0.7, 1.0};
static const float DistMap[5] = {0.15, 0.3, 0.45, 0.6, 0.8};
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
	float delta_x;
	float delta_y;

	delta_x = next_point->PointX - cur_point->PointX;
	delta_y = next_point->PointY - cur_point->PointY;

	output->CurPoint_X = cur_point->PointX;
	output->CurPoint_Y = cur_point->PointY;

	if(delta_x == 0.0 && delta_y == 0.0)
	{
		output->PathMoveFlag = false;
	}
	else if(delta_x > 0.0) output->CurDirection = 1;
	else if(delta_x < 0.0) output->CurDirection = 2;
	else if(delta_y > 0.0) output->CurDirection = 3;
	else if(delta_y < 0.0) output->CurDirection = 4;
	else output->CurDirection = 0;

	output->LengthTrace = (delta_x != 0) ? delta_x :
						  (delta_y != 0) ? delta_y : 0.0;

	if(output->LengthTrace == 0)
	{
		output->TraceVelocity = 0;
	}
	else output->TraceVelocity = &(*next_point->PointVelocity);
}

