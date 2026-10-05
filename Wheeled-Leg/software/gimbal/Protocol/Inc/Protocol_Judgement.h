/**
 * @file Protocol_Judgement.h
 * @author Why (1091537104@qq)
 * @brief ����ϵͳ����ϸЭ��,1.6.1�汾
 * @version 0.2
 * @date 2023-02-21
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef _RM_JUDGESYSTEM_H
#define _RM_JUDGESYSTEM_H

#include "main.h"
#include "PowerControl.h"

#pragma anon_unions                         //�������������ṹ�� **��ߵ�__packed��ʾ��ʹ��ϵͳ����**

#define JudgeSystem_FUNGroundInit       \
    {                                   \
        &JudgeSystem_USART_Receive_DMA, \
        &JudgeSystem_Handler,           \
        &Check_Judge,                   \
    }

typedef struct
{
    void (*JudgeSystem_USART_Receive_DMA)(UART_HandleTypeDef *huartx);
    void (*JudgeSystem_Handler)(UART_HandleTypeDef *huart);
    void (*Check_Judge)(void);
} JudgeSystem_FUN_t;

extern JudgeSystem_FUN_t JudgeSystem_FUN;

typedef struct
{
    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
} RM_Judge_t;

extern RM_Judge_t RM_Judge;

float GetVision_CP(void);

#define JUDGE_DATA_ERROR 0
#define JUDGE_DATA_CORRECT 1

//2023 ����ϵͳ�ٷ��ӿ�Э��

//��Ӧͨ��Э���ʽ   frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16,����У��)
#define       LEN_HEADER        5/*frame_header*/
#define       LEN_CMDID         2/*cmd_id*/
#define       LEN_TAIL          2/*frame_tail*/

//��ʼ�ֽڣ�Э��̶�Ϊ0xA5
#define      JUDGE_FRAME_HEADER             0xA5
#define      JUDGESYSTEM_PACKSIZE           389u        //����ϵͳ����С(354+35)

//������
#define BLUE 0
#define RED  1

//��Ϣ����
#define TRUE 1
#define FALSE 0


/***********************************������ID*************************************/
/*

ID: 0x0001          Byte: 11      ����״̬����                   ����Ƶ�� 1Hz
ID: 0x0002          Byte: 1       �����������                   ������������
ID: 0x0003          Byte: 32      ����������Ѫ������             ����Ƶ�� 3Hz
ID: 0x0101          Byte: 4       �����¼�����                   �̶�1HZƵ�ʷ���
ID: 0x0102          Byte: 4       ���ز���վ������ʶ����         �����ı����
ID: 0X0104          Byte: 3       ���о�������                   ���淢������
ID: 0x0105          Byte: 3       ���ڷ���ڵ���ʱ               ����Ƶ�� 1Hz
ID: 0X0201          Byte: 13      ������״̬����                 ����Ƶ�� 10Hz
ID: 0X0202          Byte: 16      ʵʱ������������               ����Ƶ�� 50Hz
ID: 0X0203          Byte: 16      ������λ������                 ����Ƶ�� 1Hz
ID: 0X0204          Byte: 6       ��������������                 �̶�3HZ����
ID: 0X0205          Byte: 2       ���л�����֧Ԯʱ������         ����Ƶ��1Hz��ֻ�п��л��������ط���
ID: 0X0206          Byte: 1       �˺�״̬����                   �˺���������
ID: 0X0207          Byte: 7       ʵʱ�������                   �ӵ��������
ID: 0X0208          Byte: 6       �ӵ�����������                 Ӣ�۲����ڱ��Ϳ��л����˱�����  ����Ƶ�� 10Hz
ID: 0X0209          Byte: 4       ������RFID״̬                 ����Ƶ�� 3Hz
ID: 0x020A          Byte: 6       ���ڻ����˿ͻ���ָ����         ����Ƶ�� 3Hz
ID: 0x020B          Byte: 40      ���������λ������             ����Ƶ�� 1Hz
ID: 0x020C          Byte: 6       �״��ǽ�������               ����Ƶ�� 1Hz
ID: 0x020D          Byte: 4       �ڱ���Ϣ����                   ����Ƶ�� 1Hz
ID: 0x020E          Byte: 1       �״���Ϣ����                   ����Ƶ�� 1Hz
ID: 0X0301          Byte: 128     �����˼佻������               ���ͷ���������    ����Ƶ�� 10Hz
ID: 0x0302          Byte: 30      �Զ���������������ݽӿ�       �ͻ��˴�������    ����Ƶ�� 30Hz
ID: 0x0303          Byte: 15      �ͻ���С��ͼ��������           ѡ�ֶ˴�������
ID: 0x0304          Byte: 12      ���̡������Ϣ                 ͨ��ͼ�����ڷ��ͣ��̶�30HZ
ID: 0x0305          Byte: 10      ��ͼ�����״������             Ƶ������10HZ
ID: 0x0306          Byte: 8       �Զ����������ѡ�ֶ˽�������   Ƶ������30HZ
ID: 0x0307          Byte: 103     ѡ�ֶ�С��ͼ�����ڱ�����       Ƶ������1HZ
ID: 0x0308          Byte: 34      ѡ�ֶ�С��ͼ���ܻ���������     Ƶ������3HZ

*/

#define       Judge_Game_StatusData              0x0001
#define       Judge_Game_ResultData              0x0002
#define       Judge_Robot_HP                     0x0003
#define       Judge_Event_Data                   0x0101
#define       Judge_Supply_Station               0x0102
#define       Judge_Referee_Warning              0x0104
#define       Judge_Dart_Countdown               0x0105
#define       Judge_Robot_State                  0x0201
#define       Judge_Power_Heat                   0x0202
#define       Judge_Robot_Position               0x0203
#define       Judge_Robot_Buff                   0x0204
#define       Judge_Aerial_Time                  0x0205
#define       Judge_Injury_State                 0x0206
#define       Judge_RealTime_Shoot               0x0207
#define       Judge_Remaining_Rounds             0x0208
#define       Judge_Robot_RFID                   0x0209
#define       Judge_Dart_Client                  0x020A
#define       Judge_Ground_Position              0x020B
#define       Judge_Radar_Marking                0x020C
#define       Judge_Sentry_Info                  0x020D
#define       Judge_Radar_Info                   0x020E
#define       Judge_Robot_Communicate            0x0301
#define       Judge_User_Defined                 0x0302
#define       Judge_Map_Interaction              0x0303
#define       Judge_KeyMouse_Message             0x0304
#define       Judge_Client_Map                   0x0305
#define       Judge_User_Interaction             0x0306
#define       Judge_Sentinal_Map                 0x0307
#define       Judge_Custom_Info                  0x0308


/***************************DATA_Length*************************/
/*Calculation:frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16)*/

#define       JudgeLength_Game_StatusData        20
#define       JudgeLength_Game_ResultData        10
#define       JudgeLength_Robot_HP               41
#define       JudgeLength_Event_Data             13
#define       JudgeLength_Supply_Station         13
#define       JudgeLength_Referee_Warning        11
#define       JudgeLength_Dart_Countdown         11
#define       JudgeLength_Robot_State            36
#define       JudgeLength_Power_Heat             25
#define       JudgeLength_Robot_Position         25
#define       JudgeLength_Robot_Buff             15
#define       JudgeLength_Aerial_Time            11
#define       JudgeLength_Injury_State           10
#define       JudgeLength_RealTime_Shoot         16
#define       JudgeLength_Remaining_Rounds       15
#define       JudgeLength_Robot_RFID             13
#define       JudgeLength_Dart_Client            15
#define       JudgeLength_Ground_Position        49
#define       JudgeLength_Radar_Marking          15
#define       JudgeLength_Sentry_Info            13      
#define       JudgeLength_Radar_Info             10
#define       JudgeLength_Robot_Communicate      137
#define       JudgeLength_User_Defined           39
#define       JudgeLength_Map_Interaction        24
#define       JudgeLength_KeyMouse_Message       21
#define       JudgeLength_Client_Map             19
#define       JudgeLength_User_Interaction       17
#define       JudgeLength_Sentinal_Map           112
#define       JudgeLength_Custom_Info            43

/**************************************�Զ���UI������ָʾ��Ϣ����*****************************************/

#define       LED_YELLOWMeaning                 "Fric:Manual"
#define       LED_GREENMeaning                  "Magazine:OFF"
#define       LED_ORANGEMeaning                 "ChassisMode:Normal"
#define       LED_PURPLEMeaning                 "VisionMode:None"
#define       LED_PINKMeaning                   "Pitch:Default"

#define       LED_CYANMeaning                   "Resever"
#define       LED_BLACKMeaning                  "SuperCap"
#define       LED_WHITEMeaning                  "Windmill"
#define       LED_Init                          "         "

extern int YellowFlag;
extern int GreenFlag;
extern int OrangeFlag;
extern int PurpleFlag;
extern int PinkFlag;
/**************************************����ϵͳ�ṹ��*****************************************/
/* �Զ���֡ͷ */
typedef __packed struct
{
    uint8_t  SOF;
    uint16_t DataLength;
    uint8_t  Seq;
    uint8_t  CRC8;

} xFrameHeader;

/* ID: 0x0001    Byte: 11     ����״̬���� */
typedef struct
{
    union
    {
        uint8_t dataBuff[11];
        __packed struct
        {
            uint8_t game_type : 4;             //��������
            uint8_t game_progress : 4;         //��ǰ�����׶�
            uint16_t stage_remain_time;        //��ǰ�׶�ʣ��ʱ��  ��λs
            uint64_t SyncTimeStamp;            //UNIXʱ��
        };
    } data;
    uint8_t infoUpdateFlag;
} ext_game_status_t;

/* ID: 0x0002    Byte: 1       ����������� */
typedef struct
{
    union
    {
        uint8_t dataBuff[1];
        __packed struct
        {
            uint8_t winner;//0ƽ�� 1��ʤ 2��ʤ
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_game_result_t;

/* ID: 0x0003     Byte: 32     ����������Ѫ������ */
typedef struct
{
    union
    {
        uint8_t dataBuff[32];
        __packed struct
        {
            uint16_t red_1_robot_HP;//�� 1 Ӣ�ۻ�����Ѫ����δ�ϳ��Լ�����Ѫ��Ϊ 0
            uint16_t red_2_robot_HP;//�� 2 ���̻�����Ѫ��
            uint16_t red_3_robot_HP;//�� 3 ����������Ѫ��
            uint16_t red_4_robot_HP;//�� 4 ����������Ѫ��
            uint16_t red_5_robot_HP;//�� 5 ����������Ѫ��
            uint16_t red_7_robot_HP;//�� 7 ����������Ѫ��
            uint16_t red_outpost_HP;//�췽ǰ��սѪ��
            uint16_t red_base_HP;   //�췽����Ѫ��
            uint16_t blue_1_robot_HP;
            uint16_t blue_2_robot_HP;
            uint16_t blue_3_robot_HP;
            uint16_t blue_4_robot_HP;
            uint16_t blue_5_robot_HP;
            uint16_t blue_7_robot_HP;
            uint16_t blue_outpost_HP;
            uint16_t blue_base_HP;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_game_robot_HP_t;

/* ID: 0x0101     Byte: 4       �����¼����� */
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        __packed struct
        {
            uint32_t event_type;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_event_data_t;

/* ID: 0x0102     Byte: 4       ���ز���վ������ʶ���� */
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        __packed struct
        {
            uint8_t reserved;              //֮ǰΪ����վ�� ID�����ڱ�����
            uint8_t supply_robot_id;       //���������� ID
            uint8_t supply_projectile_step;//�����ڿ���״̬
            uint8_t supply_projectile_num; //��������
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_supply_projectile_action_t;

/* ID: 0X0104          Byte: 2       ���о������� */
typedef struct
{
    union
    {
        uint8_t dataBuff[3];
        __packed struct
        {
            uint8_t level;              //����ȼ���
            uint8_t offending_robot_id; //��������� ID���и�ʱ�������� ID Ϊ 0 ���ơ�����ʱ�������� ID Ϊ��������� ID
            uint8_t count;              //�������һ���ܵ��з���Υ������˶�Ӧ�з��ȼ���Υ�����
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_referee_warning_t;

/* ID: 0x0105          Byte: 1       ���ڷ���ڵ���ʱ  */
typedef struct
{
    union
    {
        uint8_t dataBuff[3];
        __packed struct
        {
            uint8_t dart_remaining_time;//15s ����ʱ
			uint16_t dart_info;         //�����Ϣ
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_dart_remaining_time_t;


/* �����1.6.1�汾��Э�� */
/* ID: 0X0201          Byte: 13      ������״̬���� */
typedef struct
{
    union
    {
        uint8_t dataBuff[13];
        __packed struct
        {
            uint8_t robot_id;
            uint8_t robot_level;
            uint16_t remain_HP;                     //������ʣ��Ѫ��
            uint16_t max_HP;                        //����������Ѫ��
			uint16_t shooter_barrel_cooling_value;  //ǹ��ÿ����ȴֵ
			uint16_t shooter_barrel_heat_limit;     //ǹ����������
			uint16_t chassis_power_limit; 			//�����˵��̹�����������         
            /*���ص�Դ��������
            0 bit��gimbal ������� 1 Ϊ�� 24V �����0 Ϊ�� 24v �����
            1 bit��chassis �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
            2 bit��shooter �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����*/
            uint8_t mains_power_gimbal_output : 1;
            uint8_t mains_power_chassis_output : 1;
            uint8_t mains_power_shooter_output : 1;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_game_robot_status_t;

/* ID: 0X0201          Byte: 13      ������״̬���� */
//typedef struct
//{
//    union
//    {
//        uint8_t dataBuff[27];
//        __packed struct
//        {
//            uint8_t robot_id;
//            uint8_t robot_level;
//            uint16_t remain_HP;                             //������ʣ��Ѫ��
//            uint16_t max_HP;                                //����������Ѫ��
//			uint16_t shooter_id1_17mm_barrel_cooling_value; //ǹ��ÿ����ȴֵ
//			uint16_t shooter_id1_17mm_barrel_heat_limit;    //ǹ����������
//			//������1��17mm �������������ٶ�����(��λ��m/s)
//			uint16_t shooter_id1_17mm_initial_launching_speed_limit; 
//			uint16_t shooter_id2_17mm_barrel_cooling_valuecooling_rate;
//			uint16_t shooter_id2_17mm_barrel_heatcooling_limit; 
//			uint16_t shooter_id2_17mm_initial_launching_speed_limit;
//			uint16_t shooter_id1_42mm_barrel_cooling_value;
//			uint16_t shooter_id1_42mm_barrel_heat_cooling_limit;
//			uint16_t shooter_id1_42mm_initial_launching_speed_limit;     
//			uint16_t chassis_power_limit; 			//�����˵��̹�����������         
//            /*���ص�Դ��������
//            0 bit��gimbal ������� 1 Ϊ�� 24V �����0 Ϊ�� 24v �����
//            1 bit��chassis �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
//            2 bit��shooter �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����*/
//            uint8_t mains_power_gimbal_output : 1;
//            uint8_t mains_power_chassis_output : 1;
//            uint8_t mains_power_shooter_output : 1;
//        };
//    } data;
//    uint8_t InfoUpdataFlag;
//} ext_game_robot_status_t;

/* ID: 0X0202          Byte: 16      ʵʱ������������ */
typedef struct
{
    union
    {
        uint8_t dataBuff[16];
        __packed struct
        {
            uint16_t chassis_volt;                 //��Դ����ģ����������ѹ ��λ ����
            uint16_t chassis_current;              //��Դ����ģ������������ ��λ ����
            float chassis_power;                   //����������� ��λ W ��
            uint16_t chassis_power_buffer;         //���̹��ʻ��� ��λ J ���� ��ע�����¸��ݹ��������� 250J
            uint16_t shooter_id1_17mm_barrel_heat;//ǹ������
            uint16_t shooter_id2_17mm_barrel_heat;
            uint16_t shooter_id1_42mm_barrel_heat;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_power_heat_data_t;

/* ID: 0X0203          Byte: 16      ������λ������  */
typedef struct
{
    union
    {
        uint8_t dataBuff[12];
        __packed struct
        {
            float x;  //λ�� x ����
            float y;  //λ�� y ����
            float angle;//ǹ�ڲ���λ�ó���
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_game_robot_pos_t;

/* ID: 0X0204          Byte: 5       �������������� */
typedef struct
{
    union
    {
        uint8_t dataBuff[1];
        __packed struct
        {
            uint8_t power_rune_buff;/*bit 0��������Ѫ����Ѫ״̬
                                      bit 1��ǹ��������ȴ����
                                      bit 2�������˷����ӳ�
                                      bit 3�������˹����ӳ� */
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_buff_t;
/* ID: 0X0205          Byte: 2       ���л���������״̬���� */
typedef struct
{
    union
    {
        uint8_t dataBuff[2];
        __packed struct
        {
            uint8_t airforce_status;
            uint8_t time_remain;     //�ɹ���ʱ�� ��λ s��30s �ݼ��� 0
        };
    } data;
    uint8_t InfoUpdataFlag;
} aerial_robot_energy_t;

/* ID: 0X0206          Byte: 1       �˺�״̬���� */
typedef struct
{
    union
    {
        uint8_t dataBuff[1];
        __packed struct
        {
            uint8_t armor_id : 4;   /*bit 0-3����Ѫ���仯����Ϊװ���˺������ģ�飬����װ�׻���� ID��
                                            ����Ѫ���仯���ͣ��ñ�����ֵΪ 0��*/
            uint8_t hurt_type : 4;  /*bit 4-7��Ѫ���仯����,0x0 װ���˺���Ѫ��
                                            0x1 ģ����߿�Ѫ��
                                            0x2 �����ٿ�Ѫ��
                                            0x3 ��ǹ��������Ѫ��
                                            0x4 �����̹��ʿ�Ѫ��
                                            0x5 װ��ײ����Ѫ */
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_robot_hurt_t;

/* ID: 0X0207          Byte: 7       ʵʱ������� */
typedef struct
{
    union
    {
        uint8_t dataBuff[7];
        __packed struct
        {
            uint8_t bullet_type; //�ӵ�����: 1��17mm ���� 2��42mm ����
            uint8_t shooter_id;  //������� ID
            uint8_t bullet_freq; //�ӵ���Ƶ ��λ Hz
            float bullet_speed;  //�ӵ����� ��λ m/s
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_shoot_data_t;

/* ID: 0X0208          Byte: 6       �ӵ�ʣ�෢���� */
typedef struct
{
    union
    {
        uint8_t dataBuff[6];
        __packed struct
        {
            uint16_t bullet_remaining_num_17mm; //17mm �ӵ�ʣ�෢����Ŀ
            uint16_t bullet_remaining_num_42mm; //42mm �ӵ�ʣ�෢����Ŀ
            uint16_t coin_remaining_num;        //ʣ��������
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_bullet_remaining_t;

/* ID: 0X0209          Byte: 4       ������RFID״̬ */
/*  bit λֵΪ 1/0 �ĺ��壺 �Ƿ��Ѽ�⵽������� RFID
    bit0�� �������������
    bit1�� �������θߵ������
    bit2�� �Է����θߵ������
    bit3�� ���� R3/B3 ���θߵ������
    bit4�� �Է� R3/B3 ���θߵ������
    bit5�� ���� R4/B4 ���θߵ������
    bit6�� �Է� R4/B4 ���θߵ������
    bit7�� �����������ؼ����
    bit8�� ������������㣨��������һ�����ǰ��
    bit9�� ������������㣨��������һ����º�
    bit10�� �Է���������㣨�����Է�һ�����ǰ��
    bit11�� �Է���������㣨�����Է�һ����º�
    bit12�� ����ǰ��վ�����
    bit13�� ������Ѫ�㣨��⵽��һ����Ϊ���
    bit14�� �����ڱ�Ѳ����
    bit15�� �Է��ڱ�Ѳ����
    bit16�� ��������Դ�������
    bit17�� �Է�����Դ�������
    bit18�� ����������
    bit19�� �Է�������
    bit20-31������
ע�� ��������㣬�ߵ�����㣬��������㣬ǰ��վ����㣬��Դ������㣬
��Ѫ�㣬 ���������ڱ�Ѳ������ RFID �������ڲ���Ч�� �������⣬��ʹ��
�⵽��Ӧ�� RFID ������ӦֵҲ��Ϊ 0��*/
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        __packed struct
        {
            uint32_t rfid_status;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_rfid_status_t;

/* ID: 0x020A          Byte: 6      ���ڻ����˿ͻ���ָ���� */
typedef struct
{
    union
    {
        uint8_t dataBuff[6];
        __packed struct
        {
            uint8_t dart_launch_opening_status;//��ǰ���ڷ���ڵ�״̬
            uint8_t reserved;                  //����
            uint16_t target_change_time;       //�л����Ŀ��ʱ�ı���ʣ��ʱ�䣬��λ�룬��δ�л�Ĭ��Ϊ 0��
            uint16_t last_dart_launch_time;    //���һ�εķ�����ڵı���ʣ��ʱ�䣬��λ�룬��ʼֵΪ 0��
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_dart_client_cmd_t;

/* ID: 0x020B          Byte: 40      ���������λ������ */
typedef struct
{
    union
    {
        uint8_t dataBuff[40];
        __packed struct
        {
            float hero_x;        //���¾��ֱ��ʾ��Ӧ�����˵�xy������
            float hero_y;
            float engineer_x;
            float engineer_y;
            float standard_3_x;
            float standard_3_y;
            float standard_4_x;
            float standard_4_y;
            float standard_5_x;
            float standard_5_y;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_ground_robot_position_t;

/* ID: 0x020C          Byte: 6      �״��ǽ������� */
typedef struct
{
    union
    {
        uint8_t dataBuff[40];
        __packed struct
        {
            uint8_t mark_hero_progress;          //���¾���ʾ��Ӧ�����˵ı�ǽ��ȣ�ֵΪ0-120
            uint8_t mark_engineer_progress;
            uint8_t mark_standard_3_progress;
            uint8_t mark_standard_4_progress;
            uint8_t mark_standard_5_progress;
            uint8_t mark_sentry_progress;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_radar_mark_data_t;


/* �ڱ�������Ϣ�� 0x020D */
typedef struct
{
    uint32_t sentry_info;
    uint8_t InfoUpdataFlag;
} ext_sentry_info_t;

/* �״�������Ϣ�� 0x020E */
typedef struct
{
    uint32_t radar_info;
    uint8_t InfoUpdataFlag;
} ext_radar_info_t;


/*

�������ݰ���һ��ͳһ�����ݶ�ͷ�ṹ��
���ݶΰ��������� ID���������Լ������ߵ� ID ���������ݶΣ�
�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
��ȥ frame_header,cmd_id �� frame_tail �� 9 ���ֽ��Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
�ʶ����͵��������ݶ����Ϊ 113��
ÿ�������˽����������Զ�����������������кϼƴ��������� 5000 Byte�������з���Ƶ�ʷֱ𲻳���
30Hz��

������ ID��
1��Ӣ��(��)��
2������(��)��
3/4/5������(��)��
6������(��)��
7���ڱ�(��)��
9���״�վ���죩��

101��Ӣ��(��)��
102������(��)��
103/104/105������(��)��
106������(��)��
107���ڱ�(��)��
109���״�վ��������

�ͻ��� ID��
0x0101 ΪӢ�۲����ֿͻ���(��)��
0x0102�����̲����ֿͻ���((��)��
0x0103/0x0104/0x0105�����������ֿͻ���(��)��
0x0106�����в����ֿͻ���((��)��

0x0165��Ӣ�۲����ֿͻ���(��)��
0x0166�����̲����ֿͻ���(��)��
0x0167/0x0168/0x0169�����������ֿͻ��˲���(��)��
0x016A�����в����ֿͻ���(��)

*/

/*
  ���ڴ��ڶ������ ID�������� cmd_id ����Ƶ�����Ϊ 10Hz��
  ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
    �������� �����˼�ͨ�ţ�0x0301��
    ����Ƶ�ʣ����� 10Hz

    �ֽ�ƫ����  ��С    ˵��                       ��ע
        0        2      ���ݵ����� ID           0x0200~0x02FF
                                                ���������� ID ��ѡȡ������ ID �����ɲ������Զ���

        2        2      �����ߵ� ID             ��ҪУ�鷢���ߵ� ID ��ȷ�ԣ�

        4        2      �����ߵ� ID             ��ҪУ������ߵ� ID ��ȷ�ԣ�
                                                ���粻�ܷ��͵��жԻ����˵�ID

        6        n      ���ݶ�                  n ��ҪС�� 113
*/

/* ID: 0X0301          Byte: n       �����˼佻������ */
typedef __packed struct
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/* data */
typedef __packed struct
{
    uint8_t data[112];//���ݶ�nС��113
} robot_interactive_data_t;

/*�����˽�����Ϣ��0x0301*/
typedef __packed struct
{
    xFrameHeader                            txFrameHeader;//֡ͷ
    uint16_t                                CmdID;//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    robot_interactive_data_t                interactData;//���ݶ�
    uint16_t                                FrameTail;//֡β
} ext_CommunatianData_t;

/*�����˽�����Ϣ��0x0301*/
typedef struct
{
    union
    {
        uint8_t dataBuff[128];
        __packed struct
        {
            ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
			robot_interactive_data_t                interactData;//���ݶ�
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_user_data_t;

/*
    �ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0x0100   0x0101  0x0102  0x0103  0x0110  0x0104
    ����Ƶ�ʣ����� 10Hz
*/

/* �ͻ���ɾ��ͼ�� �����˼�ͨ�ţ�0x0100 */
typedef __packed struct
{
    uint8_t operate_tpye;
    uint8_t layer;
} ext_client_custom_graphic_delete_t;


/* ͼ������  ID 0x0101*/
typedef __packed struct
{
    uint8_t figure_name[3];
    uint32_t operate_tpye: 3;
    uint32_t figure_tpye: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t details_a: 9;
    uint32_t details_b: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t details_c: 10;
    uint32_t details_d: 11;
    uint32_t details_e: 11;
} interaction_figure_t;

/* ͼ������  ID 0x0102*/
typedef __packed struct
{
    interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

/* ͼ������  ID 0x0103*/
typedef __packed struct
{
    interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

/* ͼ������  ID 0x0104*/
typedef __packed struct
{
    interaction_figure_t interaction_figure[7];
} interaction_figure_4_t;

/* ͼ������  ID 0x0110*/
typedef __packed struct
{
    ext_student_interactive_header_data_t grapic_head;
    interaction_figure_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;


/* �Զ��������������˽������� Byte������30  ID 0x0302*/
typedef __packed struct
{
    uint8_t data[30];
    uint8_t InfoUpdataFlag;
} custom_robot_data_t;

/* ѡ�ֶ�С��ͼ�������ݣ�0x0303 (Byte 15)*/
/*typedef struct
{
    union
    {
        uint8_t dataBuff[15];
        __packed struct
        {
            float target_position_x;     //Ŀ��λ�õ�x����
			float target_position_y;     //Ŀ��λ�õ�y����
			uint8_t cmd_keyboard;        //���µļ���ͨ��ֵ��û�а�����Ϊ0
			uint8_t target_robot_id;     //�Է�������ID��û����Ϊ0
			uint16_t cmd_source;         //��Ϣ��Դ��ID���ٷ��ļ������д�
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_robot_command_t;
*/
/* ͼ��ң�ؿͻ����·�����ң����Ϣ:0x0304  Byte 12*/
#ifndef _EXT_ROBOT_KEYCOMMAND_T_DEFINED
#define _EXT_ROBOT_KEYCOMMAND_T_DEFINED
typedef struct
{
    union
    {
        uint8_t dataBuff[12];
        __packed struct
        {
            int16_t mouse_x;
            int16_t mouse_y;
            int16_t mouse_z;
            int8_t left_button_down;
            int8_t right_button_down;
            uint16_t keyboard_value;
            uint16_t reserved;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_robot_keycommand_t;
#endif

/* ѡ�ֶ˽���:0x0305 Byte 10*/
typedef struct
{
    union
    {
        uint8_t dataBuff[10];
        __packed struct
        {
            uint16_t target_robot_ID;
            float target_position_x;
            float target_position_y;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_client_map_command_t;

typedef struct
{
    xFrameHeader                            txFrameHeader;//֡ͷ
    uint16_t                                CmdID;//������
    //ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ext_client_map_command_t                interactData;//���ݶ�
    uint16_t                                FrameTail;//֡β
} ext_MapCommunate_t;

/* �Զ����������ѡ�ֶ˽�������:0x0306  Byte 8*/
typedef struct
{
    union
    {
        uint8_t dataBuff[8];
        __packed struct
        {
            uint16_t key_value;
            uint16_t x_position: 12;
            uint16_t mouse_left: 4;
            uint16_t y_position: 12;
            uint16_t mouse_right: 4;
            uint16_t reserved;
        };
    } data;
    uint8_t InfoUpdataFlag;
} custom_client_data_t;


/* ѡ�ֶ�С��ͼ�����ڱ�����:0x0307 Byte 103*/
typedef struct
{
    union
    {
        uint8_t dataBuff[105];
        __packed struct
        {
            uint8_t intention;
            uint16_t start_position_x;
            uint16_t start_position_y;
            int8_t delta_x[49];
            int8_t delta_y[49];
			uint16_t sender_id;
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_map_sentry_data_t;

/* ���������˿�ͨ��������·�򼺷�����ѡ�ֶ˷����Զ������Ϣ
*����Ϣ���ڼ���ѡ�ֶ��ض�λ����ʾ:0x0308 Byte 34         */
typedef struct
{
    union
    {
        uint8_t dataBuff[34];
        __packed struct
        {
            uint16_t sender_id;
			uint16_t receiver_id;
			uint8_t user_data[30];
        };
    } data;
    uint8_t InfoUpdataFlag;
} ext_custom_info_t;


void Judge_GetMessage(uint16_t Data_Length);

extern RM_Judge_t JudgeSystem;

extern ext_game_status_t                ext_game_status;
extern ext_game_result_t                ext_game_result;
extern ext_game_robot_HP_t              ext_game_robot_HP;
extern ext_event_data_t ext_even_data;
extern ext_supply_projectile_action_t   ext_supply_projectile_action;
extern ext_referee_warning_t            ext_referee_warning;
extern ext_dart_remaining_time_t        ext_dart_remaining_time;
extern ext_game_robot_status_t          ext_game_robot_state;
extern ext_power_heat_data_t            ext_power_heat_data;
extern ext_game_robot_pos_t             ext_game_robot_pos;
extern ext_buff_t Buff;
extern aerial_robot_energy_t            aerial_robot_energy;
extern ext_robot_hurt_t                 ext_robot_hurt;
extern ext_shoot_data_t                 ext_shoot_data;
extern ext_bullet_remaining_t           ext_bullet_remaining;
extern ext_rfid_status_t                ext_rfid_status;
extern ext_dart_client_cmd_t            ext_dart_client_cmd;
extern ext_ground_robot_position_t      ext_ground_robot_position;
extern ext_radar_mark_data_t            ext_radar_mark_data;
extern ext_user_data_t                  ext_user_data;
extern ext_custom_info_t                ext_custom_info;
extern ext_sentry_info_t                ext_sentry_info;
extern ext_radar_info_t                 ext_radar_info;

extern uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE];

#endif //ͷ�ļ�
