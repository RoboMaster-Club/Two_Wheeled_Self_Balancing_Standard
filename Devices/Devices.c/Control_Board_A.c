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
 
 #include "Control_Board_A.h"
 
 void Board_A_Send_Data(void);
 void Board_A_Rec_Data(void);
 void Board_A_Handler(UART_HandleTypeDef *huart);
 void Board_A_USART_Receive_DMA(UART_HandleTypeDef *huartx);
 static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size);
 
 Control_Board_A_Func_t Control_Board_A_Func = Control_Board_A_Func_GroundInit;
 Control_Board_A_t Control_Board_A;
 Board_A_Package_t Send_Package;
 Board_A_Package_t Rec_Package;
 
 #undef Control_Board_A_Func_GroundInit
 
 void Board_A_Send_Data(void)
 { 
		Send_Package.Yaw = Board_A_IMU.Export_Data.Yaw;
		Send_Package.Roll = Board_A_IMU.Export_Data.Roll;
		Send_Package.Pitch = Board_A_IMU.Export_Data.Pitch;
		Send_Package.Gyro_Yaw = Board_A_IMU.Export_Data.Gyro_Yaw;
		Send_Package.Gyro_Roll = Board_A_IMU.Export_Data.Gyro_Roll;
		Send_Package.Gyro_Pitch = Board_A_IMU.Export_Data.Gyro_Pitch;
		Send_Package.Accel_Y = Board_A_IMU.Export_Data.Ay;

		HAL_UART_Transmit(&huart7, (uint8_t*)&Send_Package, sizeof(Board_A_Package_t),10);
 }
 
 void Board_A_Rec_Data(void)
 {	
		Control_Board_A.Rec.Prev_Yaw = Control_Board_A.Rec.Yaw;
		if (fabs(Rec_Package.Yaw) <= 180)
		Control_Board_A.Rec.Yaw = Rec_Package.Yaw;
		if (fabs(Rec_Package.Pitch) <= 180)
		Control_Board_A.Rec.Roll = Rec_Package.Pitch;
		if (fabs(Rec_Package.Roll) <= 180)
		Control_Board_A.Rec.Pitch = Rec_Package.Roll;
		Control_Board_A.Rec.Gyro_Yaw = Rec_Package.Gyro_Yaw;
		Control_Board_A.Rec.Gyro_Pitch = Rec_Package.Gyro_Roll;
		Control_Board_A.Rec.Gyro_Roll = Rec_Package.Gyro_Pitch;
		Control_Board_A.Rec.Accel_Y = Rec_Package.Accel_Y - sin(Control_Board_A.Rec.Pitch/180*PI)*9.81f;
	 
		if((Control_Board_A.Rec.Yaw - Control_Board_A.Rec.Prev_Yaw) < - 300)
			Control_Board_A.Rec.Turn_Count++;
		else if((Control_Board_A.Rec.Yaw - Control_Board_A.Rec.Prev_Yaw) > 300)
			Control_Board_A.Rec.Turn_Count--;
		Control_Board_A.Rec.Total_Yaw = Control_Board_A.Rec.Yaw + 360.0f * Control_Board_A.Rec.Turn_Count;
 }
 
 static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
 {
	 if(huart->RxState == HAL_UART_STATE_READY)
	 {
		 if((pData == NULL) || (Size == 0))
			 return HAL_ERROR;
		 huart->pRxBuffPtr = pData;
		 huart->RxXferSize = Size;
		 huart->ErrorCode = HAL_UART_ERROR_NONE;
		 
		 HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,(uint32_t)pData,Size);
		 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);
	 }
	 else
		 return HAL_BUSY;
	 return HAL_OK;
 }
 
 //Receive data if pass verification
 void Board_A_Handler(UART_HandleTypeDef *huart)
 {
	 __HAL_DMA_DISABLE(huart->hdmarx);
	 Control_Board_A_Func.Board_A_Rec_Data();
	 __HAL_DMA_ENABLE(huart->hdmarx);
 }
 
 void Board_A_USART_Receive_DMA(UART_HandleTypeDef *huartx)
 {
	 __HAL_UART_CLEAR_IDLEFLAG(huartx);
	 __HAL_UART_ENABLE(huartx);
	 __HAL_UART_ENABLE_IT(huartx,UART_IT_IDLE);
	 USART_Receive_DMA_NO_IT(huartx,(uint8_t*)&Rec_Package,sizeof(Board_A_Package_t));
 }

