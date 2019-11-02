/*******************************************************************************
* mb_odometry.h
*
* 
*******************************************************************************/
#ifndef MB_ODOMETRY_H
#define MB_ODOMETRY_H

#include "mb_defs.h"
#include "mb_structs.h"

void mb_odometry_init(float x, float y, float psi);
void mb_odometry_update();
float mb_clamp_radians(float angle);
//int mb_in_range(float num, float min, float max);

#endif
