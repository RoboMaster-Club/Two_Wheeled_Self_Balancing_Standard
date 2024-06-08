/**
 * @file PID.h
 * @author Leo Liu
 * @brief header file for PID
 * @version 1.0
 * @date 2022-07-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
 
#ifndef __PID_H
#define	__PID_H

#include "User_Defined_Math.h" 
#include "MF9025_Motor.h"
#include "M3508_Motor.h"
#include "GM6020_Motor.h"
#include "M2006_Motor.h"
#include "Super_Capacitor.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define PID_Func_GroundInit		\
		{													\
			&Positional_PID,				\
					&Incremental_PID,		\
					&Clear_PID_Data,		\
		}

typedef struct
{
	float Target_Value;
	float Measured_Value;
	
	float Error;
	float Prev_Error;
	float Prev_Prev_Error;
	
	float Kp;
	float Ki;
	float Kd;
	
	float P_Out;
	float I_Out;
	float D_Out;
	
	float Output;
	float Output_Max;
	float I_Out_Max;
}PID_t;

typedef struct
{
	float (*Positional_PID)(PID_t *PID, float Target_Value, float Measured_Value);
	float (*Incremental_PID)(PID_t *PID, float Target_Value, float Measured_Value);
	void (*Clear_PID_Data)(PID_t *PID);
}PID_Func_t;

#define Fric_Wheel_PIDInit		\
		{													\
				0,										\
					0,									\
					0,									\
					0,									\
					0,									\
					10.0f,							\
					0,									\
					0,									\
					0,									\
					0,									\
					0,									\
					0,									\
					16384.0f,						\
					5000.0f,						\
		}
		
#define Chassis_Angle_PIDInit	\
		{													\
				0,										\
					0,									\
					0,									\
					0,									\
					0,									\
					100.0f,							\
					0,									\
					20000.0f,						\
					0,									\
					0,									\
					0,									\
					0,									\
					10000.0f,						\
					0,									\
		}

#define Chassis_Turning_PIDInit	\
		{													\
				0,										\
					0,									\
					0,									\
					0,									\
					0,									\
					1.0f,								\
					0,									\
					1.0f,								\
					0,									\
					0,									\
					0,									\
					0,									\
					10000.0f,						\
					0,									\
		}

#define Chassis_Speed_PIDInit	\
		{													\
				0,										\
					0,									\
					0,									\
					0,									\
					0,									\
					1000.0f,						\
					30.0f,							\
					0,									\
					0,									\
					0,									\
					0,									\
					0,									\
					10000.0f,						\
					500.0f,							\
		}

#define Chassis_Position_PIDInit	\
		{													\
				0,										\
					0,									\
					0,									\
					0,									\
					0,									\
					6000.0f,						\
					0,									\
					700000.0f,					\
					0,									\
					0,									\
					0,									\
					0,									\
					10000.0f,						\
					0,									\
		}
		
#define Yaw_Angle_PIDInit			\
		{													\
				0,										\
					0,									\
					0,									\
					0,									\
					0,									\
					500.0f,							\
					0,									\
					70000.0f,						\
					0,									\
					0,									\
					0,									\
					0,									\
					28000.0f,				  	\
					0,									\
		}

#define Yaw_Speed_PIDInit			\
		{													\
				0,										\
					0,									\
					0,									\
					0,									\
					0,									\
					5.0f,								\
					0,									\
					2.0f,								\
					0,									\
					0,									\
					0,									\
					0,									\
					25000.0f,						\
					10000.0f,						\
		}
		
#define Pitch_Angle_PIDInit		\
		{													\
				0,										\
					0,									\
					0,									\
					0,									\
					0,									\
					3.0f,								\
					0.01f,							\
					0,									\
					0,									\
					0,									\
					0,									\
					0,									\
					3000.0f,						\
					5.0f,								\
		}
		
#define Pitch_Speed_PIDInit		\
		{													\
				0,										\
					0,									\
					0,									\
					0,									\
					0,									\
					1000.0f,						\
					0,									\
					800.0f,							\
					0,									\
					0,									\
					0,									\
					0,									\
					28000.0f,						\
					6000.0f,						\
		}

#define Trigger_Angle_PIDInit	\
		{													\
				0,										\
					0,									\
					0,									\
					0,									\
					0,									\
					4.0f,								\
					0,									\
					70.0f,							\
					0,									\
					0,									\
					0,									\
					0,									\
					6000.0f,						\
					0,									\
		}

#define Trigger_Speed_PIDInit	\
		{													\
				0,										\
					0,									\
					0,									\
					0,									\
					0,									\
					8.0f,								\
					0,									\
					0,									\
					0,									\
					0,									\
					0,									\
					0,									\
					10000.0f,						\
					0,									\
		}

extern PID_Func_t PID_Func;
extern PID_t Fric_Wheel_PID;
extern PID_t Chassis_Angle_PID;
extern PID_t Yaw_Angle_PID;
extern PID_t Yaw_Speed_PID;
extern PID_t Pitch_Angle_PID;
extern PID_t Pitch_Speed_PID;
extern PID_t Trigger_Angle_PID; 
extern PID_t Trigger_Speed_PID;
extern PID_t Chassis_Position_PID;
extern PID_t Chassis_Speed_PID;
extern PID_t Chassis_Turning_PID;

#endif
