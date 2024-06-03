#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include "math.h"
#include "arm_math.h"
#include "Chassis_Control.h"

#define SAMPLE_PERIOD (2/1000.0f)
#define DIM_N (2) //size(A,1)
#define DIM_M (1) //size(B,2)
#define DIM_Q (2) //size(C,1)
	
typedef struct
{
	float Prev_P;
	float Current_P;
	float Output;
	float K;
	float Q;
	float R;
}Kalman_Filter_t;

void Kalman_Filter_Init(void);
void Kalman_Filter_Update(void);
float First_Order_Kalman_Filter(Kalman_Filter_t *KF, float Measurement);

extern float Xplus[DIM_N][1];

#endif
