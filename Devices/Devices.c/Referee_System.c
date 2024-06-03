/**
 * @file Referee_System.c
 * @author Leo Liu
 * @brief reading data from the referee system
 * @version 2.0
 * @date 2024-05-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "Referee_System.h"

Referee_System_t Referee_System;
Referee_Robot_State_t Referee_Robot_State;

HAL_StatusTypeDef Referee_UART_Receive_DMA_No_Interrupt(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef Referee_UART_Receive_Interrupt(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void Referee_System_Handler(UART_HandleTypeDef *huart);
void Referee_Get_Data(uint16_t Data_Length);
void Referee_Set_Robot_State(void);

Referee_System_Func_t Referee_System_Func = Referee_System_Func_GroundInit;
#undef Referee_System_Func_GroundInit

void Referee_Set_Robot_State(void)
{
    Referee_Robot_State.Game_Type = Referee_System.Game_Status.Type;
    Referee_Robot_State.ID = Referee_System.Robot_State.ID;
    Referee_Robot_State.Level = Referee_System.Robot_State.Level;
		Referee_Robot_State.Manual_Level = Referee_System.Robot_State.Level;

		Referee_Robot_State.Cooling_Rate = Referee_System.Robot_State.Shooter_Cooling_Value;
		Referee_Robot_State.Heat_Max = Referee_System.Robot_State.Shooter_Heat_Max;
		Referee_Robot_State.Launch_Speed_Max = DEFAULT_STANDARD_LAUNCH_SPEED_MAX;
		Referee_Robot_State.Chassis_Power_Max = Referee_System.Robot_State.Chassis_Power_Max;

    Referee_Robot_State.Chassis_Power = Referee_System.Power_Heat.Chassis_Power;
    Referee_Robot_State.Power_Buffer = Referee_System.Power_Heat.Chassis_Power_Buffer;
    Referee_Robot_State.Shooter_Heat_1 = Referee_System.Power_Heat.Shooter_1_17mm_Heat;
    Referee_Robot_State.Shooter_Heat_2 = Referee_System.Power_Heat.Shooter_2_17mm_Heat;
    Referee_Robot_State.Shooting_Frequency = Referee_System.Shooter.Frequency;
    Referee_Robot_State.Shooting_Speed = Referee_System.Shooter.Speed;
}

HAL_StatusTypeDef Referee_UART_Receive_DMA_No_Interrupt(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }

        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        /* Enable the DMA Stream */
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
        /* Enable the DMA transfer for the receiver request by setting the DMAR bit in the UART CR3 register */
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

HAL_StatusTypeDef Referee_UART_Receive_Interrupt(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    Referee_UART_Receive_DMA_No_Interrupt(huart, pData, Size);

    return HAL_OK;
}

void Referee_System_Handler(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_DMA_DISABLE(huart->hdmarx);

    Referee_System.DMA_Counter = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    Referee_System_Func.Referee_Get_Data(REFEREE_BUFFER_LEN - Referee_System.DMA_Counter);

    __HAL_DMA_SET_COUNTER(huart->hdmarx, REFEREE_BUFFER_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

// Get referee system data based on ID
void Referee_Get_Data(uint16_t Data_Length)
{
    for (int n = 0; n < Data_Length;)
    {
        if (Referee_System.Buffer[n] == REFEREE_FRAME_HEADER_START)
        {
            switch (Referee_System.Buffer[n + 5] | Referee_System.Buffer[n + 6] << 8)
            {
            case REFEREE_GAME_STATUS:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_GAME_STATUS_LEN))
                {
                    memcpy(&Referee_System.Game_Status, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_GAME_STATUS_LEN-9]));
                    n += REFEREE_GAME_STATUS_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_GAME_RESULT:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_GAME_RESULT_LEN))
                {
                    memcpy(&Referee_System.Game_Result, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_GAME_RESULT_LEN-9]));
                    n += REFEREE_GAME_RESULT_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_ROBOT_HP:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_ROBOT_HP_LEN))
                {
                    memcpy(&Referee_System.Alive_Robot, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_ROBOT_HP_LEN-9]));
                    n += REFEREE_ROBOT_HP_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_EVENT_DATA:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_EVENT_DATA_LEN))
                {
                    memcpy(&Referee_System.Event, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_EVENT_DATA_LEN-9]));
                    n += REFEREE_EVENT_DATA_LEN;
                }
                else
                    n++;
                break;
						case REFEREE_SUPPLY_STATION:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_SUPPLY_STATION_LEN))
                {
                    memcpy(&Referee_System.Supply_Station, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_SUPPLY_STATION_LEN-9]));
                    n += REFEREE_SUPPLY_STATION_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_REFEREE_WARNING:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_REFEREE_WARNING_LEN))
                {
                    memcpy(&Referee_System.Warning, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_REFEREE_WARNING_LEN-9]));
                    n += REFEREE_REFEREE_WARNING_LEN;
                }
                else
                    n++;
                break;
						case REFEREE_DART_DATA:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_DART_DATA_LEN))
                {
                    memcpy(&Referee_System.Warning, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_DART_DATA_LEN-9]));
                    n += REFEREE_DART_DATA_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_ROBOT_STATE:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_ROBOT_STATE_LEN))
                {
                    memcpy(&Referee_System.Robot_State, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_ROBOT_STATE_LEN-9]));
                    n += REFEREE_ROBOT_STATE_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_POWER_HEAT:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_POWER_HEAT_LEN))
                {
                    memcpy(&Referee_System.Power_Heat, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_POWER_HEAT_LEN-9]));
                    n += REFEREE_POWER_HEAT_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_ROBOT_POSITION:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_ROBOT_POSITION_LEN))
                {
                    memcpy(&Referee_System.Robot_Pos, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_ROBOT_POSITION_LEN-9]));
                    n += REFEREE_ROBOT_POSITION_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_ROBOT_BUFF:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_ROBOT_BUFF_LEN))
                {
                    memcpy(&Referee_System.Robot_Buff, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_ROBOT_BUFF_LEN-9]));
                    n += REFEREE_ROBOT_BUFF_LEN;
                }
                else
                    n++;
                break;
						case REFEREE_AERIAL_DATA:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_AERIAL_DATA_LEN))
                {
                    memcpy(&Referee_System.Aerial_Data, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_AERIAL_DATA_LEN-9]));
                    n += REFEREE_AERIAL_DATA_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_INJURY_STATE:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_INJURY_STATE_LEN))
                {
                    memcpy(&Referee_System.Injury_State, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_INJURY_STATE_LEN-9]));
                    n += REFEREE_INJURY_STATE_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_SHOOTER_STATE:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_SHOOTER_STATE_LEN))
                {
                    memcpy(&Referee_System.Shooter, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_SHOOTER_STATE_LEN-9]));
                    n += REFEREE_SHOOTER_STATE_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_REMAINING_AMMO:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_REMAINING_AMMO_LEN))
                {
                    memcpy(&Referee_System.Remaining_Ammo, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_REMAINING_AMMO_LEN-9]));
                    n += REFEREE_REMAINING_AMMO_LEN;
                }
                else
                    n++;
                break;
            case REFEREE_ROBOT_RFID:
                if (CRC_Verif_Func.Verify_CRC16_Check_Sum(Referee_System.Buffer + n, REFEREE_ROBOT_RFID_LEN))
                {
                    memcpy(&Referee_System.RFID, &Referee_System.Buffer[n + 7], sizeof(uint8_t[REFEREE_ROBOT_RFID_LEN-9]));
                    n += REFEREE_ROBOT_RFID_LEN;
                }
                else
                    n++;
                break;
            default:
                n++;
                break;
            }
        }
        else
            n++;
    }
}
