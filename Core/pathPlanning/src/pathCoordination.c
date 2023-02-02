#include "pathCoordination.h"

/*!
 * 0 - Core XY
 * 1 - Pinion-rail
 */
#define KINEMATIC	0

/*!
 * Measurement table of transmission
 * 0 - N flag
 * 1 - R flag
 * 2 - F1 flag
 */
//static float const MEASUREMENT_X[3] = {0, 0.25, -0.25};
//static float const MEASUREMENT_Y[3] = {0, 0.25, -0.25};

Path PathPlan;
/*!
 * VelocityMap[3] - { VerySlow, Slow, Middle, Fast, VeryFast }
 * [Value] m/s
 */
static float VelocityMap[5] = {0.1, 0.3, 0.5, 0.7, 1.0};

void Planning_init(void)
{

}

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
 * @param next_point
 * @param cur_point
 * @param output
 */
void CreatePath(pathPoint *next_point, pathPoint *cur_point, Path *output)
{
	if(!output->MoveFlag)
	{
		float delta_X, delta_Y = 0.0;

		delta_X = next_point->PointX - cur_point->PointX;
		delta_Y = next_point->PointY - cur_point->PointY;

		Transmission.Current_Dir = next_point->Direction;

		switch(Transmission.Current_Dir)
		{
		case 1:
		{
			Transmission.Current_Dist[0] = delta_X;
			Transmission.Current_Dist[1] = 0.0;
			Transmission.Transmit_float[0] = *next_point->PointVelocity;
			Transmission.Transmit_float[1] = 0.0;
			break;
		}
		case 2:
		{
			Transmission.Current_Dist[1] = delta_Y;
			Transmission.Current_Dist[0] = 0.0;
			Transmission.Transmit_float[1] = *next_point->PointVelocity;
			Transmission.Transmit_float[0] = 0.0;
			break;
		}
		case 3:
		{
			Transmission.Current_Dist[1] = delta_Y;
			Transmission.Current_Dist[0] = 0.0;
			Transmission.Transmit_float[1] = *next_point->PointVelocity;
			Transmission.Transmit_float[0] = 0.0;
			break;
		}
		case 4:
		{
			Transmission.Current_Dist[0] = delta_X;
			Transmission.Current_Dist[1] = 0.0;
			Transmission.Transmit_float[0] = *next_point->PointVelocity;
			Transmission.Transmit_float[1] = 0.0;
			break;
		}
		default:
		{
			Transmission.Current_Dist[0] = 0.0;
			Transmission.Current_Dist[1] = 0.0;
			Transmission.Transmit_float[0] = 0.0;
			Transmission.Transmit_float[1] = 0.0;
			break;
		}
		}
		Transmission.Finish = false;
	}
}

void ZeroPlanning(void)
{

}

void CustomPlanning(uint8_t Transmission)
{

}

void Tracking(void)
{

#if(KINEMATIC == 0)

	if(Transmission.Current_Dir == 1)
	{
		Transmission.X_pos = (Regulator[0].Dist + Regulator[1].Dist) / 2;
		Regulator[2].Current = Transmission.X_pos;
	} else if(Transmission.Current_Dir == 2)
	{
		Transmission.Y_pos = (Regulator[0].Dist - Regulator[1].Dist) / 2;
		Regulator[2].Current = Transmission.Y_pos;
	} else if(Transmission.Current_Dir == 3)
	{
		Transmission.Y_pos = (-Regulator[0].Dist + Regulator[1].Dist) / 2;
		Regulator[2].Current = Transmission.Y_pos;
	} else if(Transmission.Current_Dir == 4)
	{
		Transmission.X_pos = (-Regulator[0].Dist - Regulator[1].Dist) / 2;
		Regulator[2].Current = Transmission.X_pos;
	}

#endif /* KINEMATIC__0 */

}
