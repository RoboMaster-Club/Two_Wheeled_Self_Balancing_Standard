 /**
 * @file Chassis_Control.c
 * @author Leo Liu
 * @brief control chassis
 * @version 1.0
 * @date 2022-07-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Chassis_Control.h"
Chassis_t Chassis;

void Chassis_Get_Data(Chassis_t *Chassis);
void Chassis_State_Update(Chassis_t *Chassis);
void Chassis_Processing(Chassis_t *Chassis);

Chassis_Func_t Chassis_Func = Chassis_Func_GroundInit;
#undef Chassis_Func_GroundInit

void Chassis_Get_Data(Chassis_t *Chassis)
{
	if(State_Machine.Control_Source == Remote_Control)
	{
		Chassis->Gimbal_Coord.Vx = DR16_Export_Data.Remote_Control.Joystick_Left_Vx / 330.0f;
		Chassis->Gimbal_Coord.Vy = DR16_Export_Data.Remote_Control.Joystick_Left_Vy / 330.0f;
		Chassis->Gimbal_Coord.Wz = 0;
	}
	else if(State_Machine.Control_Source == Computer)
	{
		Chassis->Gimbal_Coord.Vx = 2*(DR16_Export_Data.Keyboard.Press_D.Hold_Flag - DR16_Export_Data.Keyboard.Press_A.Hold_Flag);
		Chassis->Gimbal_Coord.Vy = 2*(DR16_Export_Data.Keyboard.Press_W.Hold_Flag - DR16_Export_Data.Keyboard.Press_S.Hold_Flag);
		Chassis->Gimbal_Coord.Wz = 0;
	}
	
	Chassis->Chassis_Coord.Prev_Forward_Speed = Chassis->Chassis_Coord.Forward_Speed;
	Chassis->Chassis_Coord.Forward_Speed = (MF9025_Chassis[0].Actual_Speed - MF9025_Chassis[1].Actual_Speed) / 2.0f  / 360.0f * PI * WHEEL_DIAMETER;
	Chassis->Chassis_Coord.Forward_Distance = (MF9025_Chassis[0].Total_Turn - MF9025_Chassis[1].Total_Turn) / 2.0f * PI * WHEEL_DIAMETER;
	Chassis->Chassis_Coord.Forward_Acceleration = Control_Board_A.Rec.Accel_Y;
	
	Chassis->Chassis_Coord.Prev_Pitch_Angle = Chassis->Chassis_Coord.Pitch_Angle;
	Chassis->Chassis_Coord.Pitch_Angle = Control_Board_A.Rec.Pitch;
	Chassis->Chassis_Coord.Prev_Pitch_Angular_Rate = Chassis->Chassis_Coord.Pitch_Angular_Rate;
	Chassis->Chassis_Coord.Pitch_Angular_Rate = Control_Board_A.Rec.Gyro_Pitch;// * 0.7f + Chassis->Chassis_Coord.Prev_Pitch_Angular_Rate * (1-0.7f);
	
	Chassis->Chassis_Coord.Prev_Yaw_Angle = Chassis->Chassis_Coord.Yaw_Angle;
	Chassis->Chassis_Coord.Yaw_Angle = Control_Board_A.Rec.Yaw;
	Chassis->Chassis_Coord.Prev_Yaw_Angular_Rate = Chassis->Chassis_Coord.Yaw_Angular_Rate;
	Chassis->Chassis_Coord.Yaw_Angular_Rate = Control_Board_A.Rec.Gyro_Yaw; //* 0.7f + Chassis->Chassis_Coord.Prev_Yaw_Angular_Rate * (1-0.7f);
	
	Chassis->Slip_Detection.Friction_Force_Left = ((0.3058f*MF9025_Chassis[0].Calculated_Current+0.1071f) - 0.01f*MF9025_Chassis[0].Angular_Acceleration)/0.11f;
	Chassis->Slip_Detection.Friction_Force_Right = ((0.3058f*MF9025_Chassis[1].Calculated_Current+0.1071f) - 0.01f*MF9025_Chassis[1].Angular_Acceleration)/0.11f;
}

void Chassis_State_Update(Chassis_t *Chassis)
{
	switch(Chassis->Current_State)
	{
		case(Balancing):
		{
			if(fabs(Chassis->Chassis_Coord.Vy) > 0.5f)
			{
				Chassis->Current_State = Moving;
				PID_Func.Clear_PID_Data(&Chassis_Speed_PID);
				PID_Func.Clear_PID_Data(&Chassis_Turning_PID);
				break;
			}
			else if(fabs(Chassis->Chassis_Coord.Forward_Speed_KF) > 0.5f) 
			{
				Chassis->Current_State = Braking;
				PID_Func.Clear_PID_Data(&Chassis_Speed_PID);
				PID_Func.Clear_PID_Data(&Chassis_Turning_PID);
				break;
			}
			else if(Chassis->Gimbal_Coord.Vy == 0 && Chassis->Gimbal_Coord.Wz == 0 && fabs(Chassis->Chassis_Coord.Pitch_Angle - CHASSIS_TARGET_ANGLE) < 10)
			{
				if((abs(MF9025_Chassis[0].Actual_Speed) > 2000 || abs(MF9025_Chassis[1].Actual_Speed) > 2000) && Chassis->Off_Ground_Detection.Off_Ground_Flag == 0)
					Chassis->Off_Ground_Detection.Counter++;
				
				if(Chassis->Off_Ground_Detection.Counter > 20)
					Chassis->Off_Ground_Detection.Off_Ground_Flag = 1;
				break;
			}
      break;
		}
		
		case(Moving):
		{
			if(Chassis->Chassis_Coord.Vy == 0)
			{
				Chassis->Current_State = Braking;
				PID_Func.Clear_PID_Data(&Chassis_Speed_PID);
				PID_Func.Clear_PID_Data(&Chassis_Turning_PID);
				break;
			}
			else
			{
				Chassis->Chassis_Coord.Prev_Vy = Chassis->Chassis_Coord.Vy;
				break;
			}
		}
		
		case(Braking):
		{
			if(fabs(Chassis->Chassis_Coord.Vy) > 0.5f)
			{
				Chassis->Current_State = Moving;
				PID_Func.Clear_PID_Data(&Chassis_Speed_PID);
				PID_Func.Clear_PID_Data(&Chassis_Turning_PID);
				break;
			}
			
			if(fabs(Chassis->Chassis_Coord.Forward_Speed_KF) < 0.2f)
			{
				Chassis->Current_State = Balancing;
				PID_Func.Clear_PID_Data(&Chassis_Speed_PID);
				PID_Func.Clear_PID_Data(&Chassis_Turning_PID);
				Chassis->Chassis_Coord.Prev_Vy = 0;
				Chassis->Chassis_Coord.Forward_Distance = 0;
				break;
			}
			
			if(Chassis->Gimbal_Coord.Vy == 0 && Chassis->Gimbal_Coord.Wz == 0 && fabs(Chassis->Chassis_Coord.Pitch_Angle - CHASSIS_TARGET_ANGLE) < 10)
			{
				if((abs(MF9025_Chassis[0].Actual_Speed) > 2000 || abs(MF9025_Chassis[1].Actual_Speed) > 2000) && Chassis->Off_Ground_Detection.Off_Ground_Flag == 0)
					Chassis->Off_Ground_Detection.Counter++;
				
				if(Chassis->Off_Ground_Detection.Counter > 20)
					Chassis->Off_Ground_Detection.Off_Ground_Flag = 1;
				break;
			}
			break;
		}
	}
}

void Chassis_Processing(Chassis_t *Chassis)
{
	switch(State_Machine.Mode)
	{
		case(Follow_Gimbal):
		{
			Gimbal.Angle_Difference = DEG_TO_RAD(Find_Gimbal_Min_Angle(GM6020_Yaw.Actual_Angle - YAW_MID_MECH_ANGLE) * GM6020_ANGLE_CONVERT);
			Chassis->Chassis_Coord.Vy = Chassis->Gimbal_Coord.Vy;
			Chassis->Chassis_Coord.Wz = PID_Func.Positional_PID(&Chassis_Angle_PID,0,Gimbal.Angle_Difference);
			if(fabs(Chassis->Chassis_Coord.Wz) < 20.0f)
				Chassis->Chassis_Coord.Wz = 0;
			Chassis_Func.Chassis_State_Update(Chassis);
			Control_Strategy_Func.Expert_PID_LQR_Combined();
			
			if(Chassis->Off_Ground_Detection.Off_Ground_Flag)
			{
				MF9025_Chassis[0].Target_Speed = 0;
				MF9025_Chassis[1].Target_Speed = 0;
				
				if(abs(MF9025_Chassis[0].Actual_Speed) < 100 && abs(MF9025_Chassis[1].Actual_Speed) < 100 && Chassis->Off_Ground_Detection.Counter > 0)
					Chassis->Off_Ground_Detection.Counter--;
				
				if(Chassis->Off_Ground_Detection.Counter == 0 && (fabs(Chassis->Slip_Detection.Friction_Force_Right) > 25 || fabs(Chassis->Slip_Detection.Friction_Force_Left) > 25))
					Chassis->Off_Ground_Detection.Off_Ground_Flag = 0;
			}
			else
			{
				MF9025_Chassis[0].Target_Speed = VAL_LIMIT(100*Chassis->Target.Left_Wheel,Chassis->Slip_Detection.Max_Speed_Left,-Chassis->Slip_Detection.Max_Speed_Left);
				MF9025_Chassis[1].Target_Speed = VAL_LIMIT(100*-Chassis->Target.Right_Wheel,Chassis->Slip_Detection.Max_Speed_Right,-Chassis->Slip_Detection.Max_Speed_Right);
				if(abs(MF9025_Chassis[0].Target_Speed) < 3000)
					MF9025_Chassis[0].Target_Speed = 0;
				if(abs(MF9025_Chassis[1].Target_Speed) < 3000)
					MF9025_Chassis[1].Target_Speed = 0;	
				
				if(State_Machine.Swing_Flag && Chassis->Chassis_Coord.Vy == 0)
					State_Machine.Mode = Swing;
				else if(State_Machine.Spin_Top_Flag && Chassis->Chassis_Coord.Vy == 0 && Chassis->Current_State == Balancing)
					State_Machine.Mode = Spin_Top;
				else if(State_Machine.Follow_Wheel_Flag && Chassis->Chassis_Coord.Vy == 0)
					State_Machine.Mode = Follow_Wheel;
			}
			break;
		}	
		
		case(Not_Follow_Gimbal):
		{
			Chassis->Chassis_Coord.Vy = Chassis->Gimbal_Coord.Vy;
			Chassis->Chassis_Coord.Wz = Chassis->Gimbal_Coord.Wz;
			Chassis_Func.Chassis_State_Update(Chassis);
			Control_Strategy_Func.Expert_PID_LQR_Combined();
			MF9025_Chassis[0].Target_Speed = VAL_LIMIT(100*Chassis->Target.Left_Wheel,MF9025_SPEED_MAX,-MF9025_SPEED_MAX);
			MF9025_Chassis[1].Target_Speed = VAL_LIMIT(100*-Chassis->Target.Right_Wheel,MF9025_SPEED_MAX,-MF9025_SPEED_MAX);
			if(abs(MF9025_Chassis[0].Target_Speed) < 4000)
				MF9025_Chassis[0].Target_Speed = 0;
			if(abs(MF9025_Chassis[1].Target_Speed) < 4000)
				MF9025_Chassis[1].Target_Speed = 0;	
			break;
		}
		
		case(Spin_Top):
		{
			Chassis->Chassis_Coord.Vy = Chassis->Gimbal_Coord.Vy;
			Chassis->Chassis_Coord.Wz = -Chassis->Chassis_Coord.Spin_Rate; //This is where you control how fast the spintop spins
			Chassis_Func.Chassis_State_Update(Chassis);
			Control_Strategy_Func.Expert_PID_LQR_Combined();
			MF9025_Chassis[0].Target_Speed = VAL_LIMIT(100*Chassis->Target.Left_Wheel,MF9025_SPEED_MAX,-MF9025_SPEED_MAX);
			MF9025_Chassis[1].Target_Speed = VAL_LIMIT(100*-Chassis->Target.Right_Wheel,MF9025_SPEED_MAX,-MF9025_SPEED_MAX);
			
			if(Chassis->Chassis_Coord.Vy != 0)
				State_Machine.Mode = Follow_Gimbal;
			else if(State_Machine.Swing_Flag && Chassis->Chassis_Coord.Vy == 0)
				State_Machine.Mode = Swing;
			else if(State_Machine.Follow_Wheel_Flag && Chassis->Chassis_Coord.Vy == 0)
				State_Machine.Mode = Follow_Wheel;
			break;
		}
		
		case(Follow_Wheel):
		{
			Gimbal.Angle_Difference = DEG_TO_RAD(Find_Gimbal_Min_Angle(GM6020_Yaw.Actual_Angle - YAW_WHEEL_MID_MECH_ANGLE) * GM6020_ANGLE_CONVERT);
			Chassis->Chassis_Coord.Vy = Chassis->Gimbal_Coord.Vy;
			Chassis->Chassis_Coord.Wz = PID_Func.Positional_PID(&Chassis_Angle_PID,0,Gimbal.Angle_Difference);
			Chassis_Func.Chassis_State_Update(Chassis);
			Control_Strategy_Func.Expert_PID_LQR_Combined();
			MF9025_Chassis[0].Target_Speed = VAL_LIMIT(100*Chassis->Target.Left_Wheel,MF9025_SPEED_MAX,-MF9025_SPEED_MAX);
			MF9025_Chassis[1].Target_Speed = VAL_LIMIT(100*-Chassis->Target.Right_Wheel,MF9025_SPEED_MAX,-MF9025_SPEED_MAX);
			if(abs(MF9025_Chassis[0].Target_Speed) < 3000)
				MF9025_Chassis[0].Target_Speed = 0;
			if(abs(MF9025_Chassis[1].Target_Speed) < 3000)
				MF9025_Chassis[1].Target_Speed = 0;	
			
			if(Chassis->Chassis_Coord.Vy != 0)
				State_Machine.Mode = Follow_Gimbal;
			else if(State_Machine.Spin_Top_Flag && Chassis->Chassis_Coord.Vy == 0)
				State_Machine.Mode = Spin_Top;
			else if(State_Machine.Swing_Flag && Chassis->Chassis_Coord.Vy == 0)
				State_Machine.Mode = Swing;
			break;
		}	
		
		case(Swing):
		{
			if(fabs(Find_Gimbal_Min_Angle(GM6020_Yaw.Actual_Angle - YAW_SWING_LOWER_ANGLE)) < 200)
				Chassis->Chassis_Coord.Swing_Target_Angle = YAW_SWING_UPPER_ANGLE;
			else if(fabs(Find_Gimbal_Min_Angle(GM6020_Yaw.Actual_Angle - YAW_SWING_UPPER_ANGLE)) < 200)
				Chassis->Chassis_Coord.Swing_Target_Angle = YAW_SWING_LOWER_ANGLE;
			Gimbal.Angle_Difference = DEG_TO_RAD(Find_Gimbal_Min_Angle(GM6020_Yaw.Actual_Angle - Chassis->Chassis_Coord.Swing_Target_Angle) * GM6020_ANGLE_CONVERT);
			Chassis->Chassis_Coord.Vy = Chassis->Gimbal_Coord.Vy; 
			Chassis->Chassis_Coord.Wz = PID_Func.Positional_PID(&Chassis_Angle_PID,0,Gimbal.Angle_Difference); //This is where you control how fast the spintop spins
			Chassis_Func.Chassis_State_Update(Chassis);
			Control_Strategy_Func.Expert_PID_LQR_Combined();
			MF9025_Chassis[0].Target_Speed = VAL_LIMIT(100*Chassis->Target.Left_Wheel,MF9025_SPEED_MAX,-MF9025_SPEED_MAX);
			MF9025_Chassis[1].Target_Speed = VAL_LIMIT(100*-Chassis->Target.Right_Wheel,MF9025_SPEED_MAX,-MF9025_SPEED_MAX);
			
			if(Chassis->Gimbal_Coord.Vy != 0)
				State_Machine.Mode = Follow_Gimbal;
			else if(State_Machine.Spin_Top_Flag && Chassis->Chassis_Coord.Vy == 0)
				State_Machine.Mode = Spin_Top;
			else if(State_Machine.Follow_Wheel_Flag && Chassis->Chassis_Coord.Vy == 0)
				State_Machine.Mode = Follow_Wheel;
			break;
		}
		
		case(Escape):
		{
			if(Control_Board_A.Rec.Pitch < -10.0f)
			{
				MF9025_Chassis[0].Target_Speed = 30000.0f;
				MF9025_Chassis[1].Target_Speed = -30000.0f;
			}
			else if(Control_Board_A.Rec.Pitch > 10.0f)
			{
				MF9025_Chassis[0].Target_Speed = -30000.0f;
				MF9025_Chassis[1].Target_Speed = 30000.0f;
			}
			break;
		}
		
		case(Disabled):
		{
			//Disabled so everything is zero
			MF9025_Chassis[0].Target_Speed = 0;
			MF9025_Chassis[1].Target_Speed = 0;
			PID_Func.Clear_PID_Data(&Chassis_Speed_PID);
			PID_Func.Clear_PID_Data(&Chassis_Turning_PID);
			break;
		}
	}
}
