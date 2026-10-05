/**
 * @file dji_motor.cpp
 * @author ZHY
 * @brief 大疆电机配置与操作
 * @version 
 * @date 2025-8-18
 *
 * @copyright 
 *
 */

#include "M3508.h"
#include <stdio.h>


FDCAN_HandleTypeDef *Class_Motor_3508::Can_Motor = &hfdcan2; //绑定CAN总线

/**
 * @brief 分配CAN发送缓冲区（大疆电机多个使用同一段数据）
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(FDCAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr = nullptr;

    if (hcan == &hfdcan1)
    {
        switch (__CAN_ID)
        {
        case (CAN_Motor_ID_0x201):
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);
            break;
        case (CAN_Motor_ID_0x202):
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);
            break;
        case (CAN_Motor_ID_0x203):
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);
            break;
        case (CAN_Motor_ID_0x204):
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);
            break;
        case (CAN_Motor_ID_0x205):
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);
            break;
        case (CAN_Motor_ID_0x206):
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[2]);
            break;
        case (CAN_Motor_ID_0x207):
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[4]);
            break;
        case (CAN_Motor_ID_0x208):
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[6]);
            break;
        case (CAN_Motor_ID_0x209):
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[0]);
            break;
        case (CAN_Motor_ID_0x20A):
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[2]);
            break;
        case (CAN_Motor_ID_0x20B):
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[4]);
            break;
        default:
            break;
        }
    }
    else if (hcan == &hfdcan2)
    {
        switch (__CAN_ID)
        {
        case (CAN_Motor_ID_0x201):
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[0]);
            break;
        case (CAN_Motor_ID_0x202):
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[2]);
            break;
        case (CAN_Motor_ID_0x203):
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[4]);
            break;
        case (CAN_Motor_ID_0x204):
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[6]);
            break;
        case (CAN_Motor_ID_0x205):
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[0]);
            break;
        case (CAN_Motor_ID_0x206):
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[2]);
            break;
        case (CAN_Motor_ID_0x207):
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[4]);
            break;
        case (CAN_Motor_ID_0x208):
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[6]);
            break;
        case (CAN_Motor_ID_0x209):
            tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[0]);
            break;
        case (CAN_Motor_ID_0x20A):
            tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[2]);
            break;
        case (CAN_Motor_ID_0x20B):
            tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[4]);
            break;
        default:
            break;
        }
    }
    return (tmp_tx_data_ptr);
}

	
/**
 * @brief 3508电机初始化
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @param __Control_Method 电机控制方式, 默认速度
 * @param __Gearbox_Rate 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __Torque_Max 最大扭矩, 需根据不同负载测量后赋值
 */
void Class_Motor_3508::Set_CAN(FDCAN_HandleTypeDef *hcan)
{
    Can_Motor = hcan;
}

void Class_Motor_3508::Init(Enum_CAN_Motor_ID __CAN_ID,
                            Enum_Control_Method __Control_Method)
{
    CAN_ID = __CAN_ID;
    Control_Method = __Control_Method;
    CAN_Tx_Data = allocate_tx_data(Can_Motor, CAN_ID);
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Motor_3508::FDCAN_RxCpltCallback(FDCan_Export_Data_t RxMessage)
{
    int16_t delta_encoder;

    Flag += 1;

    Pre_Encoder = Rx_Encoder;

    Rx_Encoder = (int16_t)((RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]));
    Rx_Omega = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
    Rx_TorqueI = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
    Rx_Temperature = RxMessage.FDCANx_Export_RxMessage[6];

    delta_encoder = Rx_Encoder - Pre_Encoder;
    if (delta_encoder < -(Encoder_Num_Per_Round*0.5))
    {
        //正方向转过了一圈
        Total_Round++;
    }
    else if (delta_encoder > Encoder_Num_Per_Round*0.5)
    {
        //反方向转过了一圈
        Total_Round--;
    }
    Total_Encoder = Total_Round * Encoder_Num_Per_Round + Rx_Encoder;

    Now_Angle = (float)Total_Encoder / (float)Encoder_Num_Per_Round * 2.0f * PI / Gearbox_Rate;
    Now_Omega = (float)Rx_Omega * RPM_TO_RADPS / Gearbox_Rate;
    // 反馈的“实际转矩电流”是转子端 Iq，单位 A
    Now_TorqueI = ((float)Rx_TorqueI / M3508_C620_CMD_MAX) * M3508_C620_IQ_MAX_A;
    Now_Torque =  Now_TorqueI * KA * Gearbox_Rate;
    Now_Temperature = Rx_Temperature;
}
/**
 * @brief 电机数据输出变为协议要求的格式
 *
 */
void Class_Motor_3508::Output()
{
    if (CAN_Tx_Data == nullptr)
    {
        return;
    }
    CAN_Tx_Data[0] = targetCurrent >> 8;
    CAN_Tx_Data[1] = targetCurrent;
}

/**
 * @brief 获取最大输出电流
 *
 * @return uint16_t 最大输出电流
 */
uint16_t Class_Motor_3508::Get_Output_Max()
{
    return (Output_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_CAN_Motor_Status 电机状态
 */
Enum_CAN_Motor_Status Class_Motor_3508::Get_CAN_Motor_Status()
{
    return (CAN_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_Motor_3508::Get_Now_Angle()
{
    return (Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_Motor_3508::Get_Now_Omega()
{
    return (Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return 当前的扭矩, 直接采用反馈值
 */
float Class_Motor_3508::Get_Now_Torque()
{
    return (Now_Torque);
}

/**
 * @brief 获取当前的温度, 摄氏度
 *
 * @return uint8_t 当前的温度, 摄氏度
 */
uint8_t Class_Motor_3508::Get_Now_Temperature()
{
    return (Now_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Control_Method 电机控制方式
 */
Enum_Control_Method Class_Motor_3508::Get_Control_Method()
{
    return (Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_Motor_3508::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_Motor_3508::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, 直接采用反馈值
 *
 * @return float 目标的扭矩, 直接采用反馈值
 */
float Class_Motor_3508::Get_Target_Torque()
{
    return (Target_Torque);
}

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
float Class_Motor_3508::Get_Out()
{
    return (Out);
}


/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_Motor_3508::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_Motor_3508::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 3508下达控制电流的函数
 *
 * @param 
 */
void Class_Motor_3508::Ctrl()
{
    switch (Control_Method)
    {
    case Control_Method_OPENLOOP:
        break;
    case Control_Method_TORQUE:
        Calc_Current();
        Output();
        break;
    case Control_Method_OMEGA:
        break;
    case Control_Method_ANGLE:
        break;
    default:
        break;
    }
}
/**
 * @brief 设定目标的扭矩, 直接采用反馈值
 *
 * @param __Target_Torque 目标的扭矩, 直接采用反馈值
 */
void Class_Motor_3508::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
  * @brief  设置M3508电机电流值（id号为1~4）静态函数，所有电机都通过这个函数来发送CAN数据
  * @param  CtrlDatax (x:1~4) 对应id号电机的电流值，范围-16383~0~16383
  * @retval None
  */
void Class_Motor_3508::Set_Current()
{
    uint8_t *tx_data = nullptr;
    if (Can_Motor == &hfdcan1)
    {
        tx_data = CAN1_0x200_Tx_Data;
    }
    else if (Can_Motor == &hfdcan2)
    {
        tx_data = CAN2_0x200_Tx_Data;
    }
    if (tx_data != nullptr)
    {
        Can_Fun.fdcanx_send_data(Can_Motor, M3508_SENDID_Chassis, tx_data, 8);
    }
}

void Class_Motor_3508::Calc_Current()
{
    targetTorqueI = Target_Torque/(KA*Gearbox_Rate);

    // 将转矩电流限制在 C620 可控范围内（转子端 Iq）
    if (targetTorqueI > M3508_C620_IQ_MAX_A) {
        targetTorqueI = M3508_C620_IQ_MAX_A;
    }
    if (targetTorqueI < -M3508_C620_IQ_MAX_A) {
        targetTorqueI = -M3508_C620_IQ_MAX_A;
    }
    
    //将电流映射到控制电流的范围内 [-16384, 16384]
    targetCurrent = (int16_t)((targetTorqueI / M3508_C620_IQ_MAX_A) * M3508_C620_CMD_MAX);
}
