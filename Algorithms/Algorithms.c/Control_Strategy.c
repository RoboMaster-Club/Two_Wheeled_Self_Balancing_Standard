	/**
 * @file Control_Strategy.c
 * @author Leo Liu
 * @brief different control strategy for chassis control of SBR
 * @version 1.0
 * @date 2023-01-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
 
#include "Control_Strategy.h"

void Expert_PID_LQR_Combined(void);

Control_Strategy_Func_t Control_Strategy_Func = Control_Strategy_Func_GroundInit;
#undef Control_Strategy_Func_GroundInit

float LQR_K_Matrix[4] = {9.9196,  21.3417, 395.9124,  54.9115};

void Expert_PID_LQR_Combined(void)
{
	switch(Chassis.Current_State)
	{
		case Balancing:
		{
			 Chassis_Speed_PID.Kp = 3000.0f;
			 Chassis_Speed_PID.Ki = 10.0f;
			 Chassis_Speed_PID.I_Out_Max = 3000.0f;
			 Chassis_Speed_PID.Output_Max = 10000.0f;
			 Chassis.PID_Output.Speed_Loop = PID_Func.Positional_PID(&Chassis_Speed_PID, CHASSIS_TARGET_SPEED, Chassis.Chassis_Coord.Forward_Speed_KF);		
			
			 if(State_Machine.Mode == Spin_Top)
			 {
				 Chassis.PID_Output.Spintop_Compensate_Loop = MF9025_Chassis[0].Actual_Speed + MF9025_Chassis[1].Actual_Speed;
				 Chassis.PID_Output.Speed_Loop = 0;
			 }
			 else
				 Chassis.PID_Output.Spintop_Compensate_Loop = 0;
			 
			 Chassis.PID_Output.Speed_Loop = PID_Func.Positional_PID(&Chassis_Speed_PID, CHASSIS_TARGET_SPEED, Chassis.Chassis_Coord.Forward_Speed_KF);			
			
			 if(fabs(Chassis.Chassis_Coord.Pitch_Angle) < 15.0f)
					Chassis.Chassis_Coord.Target_Pitch_Angle += (-Chassis.Chassis_Coord.Target_Pitch_Angle - Chassis.Chassis_Coord.Pitch_Angle)/500.0f;
			 Chassis.PID_Output.Angle_Loop = 0.75f * (Chassis.Chassis_Coord.Pitch_Angle - Chassis.Chassis_Coord.Target_Pitch_Angle) * LQR_K_Matrix[2] + 3.5f * Chassis.Chassis_Coord.Pitch_Angular_Rate * LQR_K_Matrix[3];
			 
			 Chassis_Turning_PID.Kp = 5.0f;
			 Chassis_Turning_PID.Ki = 0.02f;
			 Chassis_Turning_PID.Output_Max = 10000.0f;
			 Chassis_Turning_PID.I_Out_Max = 1000.0f;
			 Chassis.PID_Output.Turning_Loop = PID_Func.Positional_PID(&Chassis_Turning_PID, Chassis.Chassis_Coord.Wz,-Chassis.Chassis_Coord.Yaw_Angular_Rate);
			
			 Chassis.Target.Left_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop - Chassis.PID_Output.Turning_Loop + Chassis.PID_Output.Spintop_Compensate_Loop; //Left Wheel
			 Chassis.Target.Right_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop + Chassis.PID_Output.Turning_Loop + Chassis.PID_Output.Spintop_Compensate_Loop; //Right Wheel
			
			 break;
		}
		case Moving:
		{
			Chassis_Speed_PID.Kp = 3000.0f;
			Chassis_Speed_PID.Kd = 10000.0f;
			Chassis_Speed_PID.Output_Max = 10000.0f;
			if(State_Machine.Mode == Follow_Wheel)
				Chassis.PID_Output.Speed_Loop = PID_Func.Positional_PID(&Chassis_Speed_PID, -Chassis.Chassis_Coord.Vx, Chassis.Chassis_Coord.Forward_Speed_KF); 
			else
				Chassis.PID_Output.Speed_Loop = PID_Func.Positional_PID(&Chassis_Speed_PID, Chassis.Chassis_Coord.Vy, Chassis.Chassis_Coord.Forward_Speed_KF); 
			
			Chassis.PID_Output.Angle_Loop = 0.75f * (Chassis.Chassis_Coord.Pitch_Angle - Chassis.Chassis_Coord.Target_Pitch_Angle) * LQR_K_Matrix[2] + 3.5f * Chassis.Chassis_Coord.Pitch_Angular_Rate * LQR_K_Matrix[3];

			Chassis_Turning_PID.Kp = 5.0f;
			Chassis_Turning_PID.Ki = 0.02f;
			Chassis_Turning_PID.Output_Max = 10000.0f;
			Chassis_Turning_PID.I_Out_Max = 3000.0f;
			Chassis.PID_Output.Turning_Loop = PID_Func.Positional_PID(&Chassis_Turning_PID, Chassis.Chassis_Coord.Wz,	-Chassis.Chassis_Coord.Yaw_Angular_Rate);

			Chassis.Target.Left_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop - Chassis.PID_Output.Turning_Loop;	//Left Wheel
			Chassis.Target.Right_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop + Chassis.PID_Output.Turning_Loop;	//Right Wheel
			break;
		}
		case Braking:
		{
			Chassis_Speed_PID.Kp = 3000.0f;
			Chassis_Speed_PID.Ki = 60.0;
			Chassis_Speed_PID.I_Out_Max = 5000.0f;
			Chassis_Speed_PID.Output_Max = 10000.0f;
			Chassis.PID_Output.Speed_Loop = PID_Func.Positional_PID(&Chassis_Speed_PID, CHASSIS_TARGET_SPEED, Chassis.Chassis_Coord.Forward_Speed_KF); 
			Chassis.PID_Output.Angle_Loop = 0.75f * (Chassis.Chassis_Coord.Pitch_Angle - Chassis.Chassis_Coord.Target_Pitch_Angle) * LQR_K_Matrix[2] + 3.5f * Chassis.Chassis_Coord.Pitch_Angular_Rate * LQR_K_Matrix[3];
			//Chassis.PID_Output.Angle_Loop = 0.9f * (Chassis.Chassis_Coord.Pitch_Angle - Chassis.Chassis_Coord.Target_Pitch_Angle) * LQR_K_Matrix[2] + 4.5f * Chassis.Chassis_Coord.Pitch_Angular_Rate * LQR_K_Matrix[3];
			
			Chassis_Turning_PID.Kp = 5.0f;
			Chassis_Turning_PID.Ki = 0.02f;
			Chassis_Turning_PID.Output_Max = 10000.0f;
			Chassis_Turning_PID.I_Out_Max = 3000.0f;
			Chassis.PID_Output.Turning_Loop = PID_Func.Positional_PID(&Chassis_Turning_PID, Chassis.Chassis_Coord.Wz, -Chassis.Chassis_Coord.Yaw_Angular_Rate);
			
			Chassis.Target.Left_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop - Chassis.PID_Output.Turning_Loop; //Left Wheel
			Chassis.Target.Right_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop + Chassis.PID_Output.Turning_Loop; //Right Wheel
			break;
		}
	}
}
