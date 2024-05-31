/**
 * @file Control_Board_A.c
 * @author Leo Liu
 * @brief communication between two boards
 * @version 1.0
 * @date 2023-03-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 
#ifndef __CONTROL_BOARD_A_H
#define __CONTROL_BOARD_A_H

#include "dma.h"
#include "usart.h"
#include "Board_A_IMU.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define Control_Board_A_Func_GroundInit \
{																				\
		&Board_A_Send_Data,									\
			&Board_A_Rec_Data,								\
			&Board_A_Handler,									\
			&Board_A_USART_Receive_DMA,				\
}

typedef struct
{
	float Yaw;
	float Roll;
	float Pitch;
	float Gyro_Yaw;
	float Gyro_Pitch;
	float Gyro_Roll;
	float Accel_Y;
} __attribute__ ((__packed__)) Board_A_Package_t;

typedef struct
{
	struct
	{
		float Prev_Yaw;
		float Yaw;
		float Roll;
		float Pitch;
		float Gyro_Yaw;
		float Gyro_Pitch;
		float Gyro_Roll;
		float Accel_Y;
		float Turn_Count;
		float Total_Yaw;
	}Rec;
}Control_Board_A_t;

typedef struct
{
  void(*Board_A_Send_Data)(void);
  void(*Board_A_Rec_Data)(void);
	void (*Board_A_Handler)(UART_HandleTypeDef *huart);
  void (*Board_A_USART_Receive_DMA)(UART_HandleTypeDef *huartx);
}Control_Board_A_Func_t;

extern Control_Board_A_t Control_Board_A;
extern Control_Board_A_Func_t Control_Board_A_Func;

#endif
