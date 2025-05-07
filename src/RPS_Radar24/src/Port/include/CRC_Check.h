#ifndef _CRC_CHECK_H_
#define _CRC_CHECK_H_

#include <iostream>
#include <stdint.h>

#define SOF                               0xA5
#define FRAME_HEADER_LEN                  sizeof(FRAME_HEADER)
#define CMD_LEN                           2    //cmd_id bytes
#define CRC8_LEN                          1    //CRC8 bytes
#define CRC16_LEN                         2    //CRC16 bytes

////---------------output------------------------------------------------------------------------
#define CMD_ID_MAP_ROBOT_DATA_T           0x0305   // 选手端小地图接收雷达数据，频率上限为 10Hz
#define CMD_ID_ROBOT_INTERACTION          0x0301   // 车间通信
#define CMD_INTERACTION_RADAR_DECISION    0x0121   // 雷达自主决策指令
////---------------input------------------------------------------------------------------------
#define CMD_ID_RADAR_MARK_DATA_T          0x020C   // 雷达标记进度数据，固定以 1Hz频率发送
#define CMD_ID_RADAR_INFO_T               0x020E   // 雷达自主决策信息同步，固定以1Hz 频率发送


typedef enum
{
    GAME_STATE_ID                      =0x0001,// 比赛状态数据：0x0001。发送频率：1Hz
    GAME_RESULT_ID                     =0x0002,//比赛结果数据：0x0002。发送频率：比赛结束后发送
    GAME_ROBOT_SURVIVORS_ID            =0x0003,//机器人存活数据：0x0003。发送频率：1Hz
    GAME_ROBOT_HP_ID                   =0x0003,//机器人存活数据：0x0003。发送频率：1Hz
    EVENT_DADA_ID                      =0x0101,//场地事件数据：0x0101。发送频率：事件改变后发送
    SUPPLY_PROJECTILE_ACTION_ID        =0x0102,//补给站动作标识：0x0102。发送频率：动作改变后发送
    SUPPLY_PROJECTILE_BOOKING_ID       =0x0103,//请求补给站补弹子弹：cmd_id (0x0103)。发送频率：上限 10Hz
    REFEREE_WARNING_ID                 =0x0104,//裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送
    DART_INFO_DATA_ID                  =0x0105,//飞镖发射相关数据: 0x0105。发送频率：1Hz
    GAME_ROBOT_STATE_ID                =0x0201,// 比赛机器人状态：0x0201。发送频率：10Hz
    POWER_HEAT_DATA_ID                 =0x0202,//实时功率热量数据：0x0202。发送频率：50Hz
    GAME_ROBOT_POS_ID                  =0x0203,//机器人位置：0x0203。发送频率：10Hz
    BUFF_MUSK_ID                       =0x0204,// 机器人增益：0x0204。发送频率：状态改变后发送
    AERIAL_ROBOT_ENERGY_ID             =0x0205,// 空中机器人能量状态：0x0205。发送频率：10Hz
    ROBOT_HURT_ID                      =0x0206,//伤害状态：0x0206。发送频率：伤害发生后发送
    SHOOT_DATA_ID                      =0x0207,//实时射击信息：0x0207。发送频率：射击后发送
    BULLET_REMAINING_ID                =0x0208,//子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人以及哨兵机器人主控发送
    STUDENT_INTERACTIVE_HEADER_DATA_ID =0x0301,//交互数据接收信息：0x0301。发送频率：上限 10Hz
    CLIENT_CUSTOM_DATA_ID              =0x0301,//客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz
    ROBOT_INTERACTIVE_DATA_ID          =0x0301,//交互数据 机器人间通信：0x0301。发送频率：上限10Hz
    CLIENT_GRAPHIC_DRAW_ID             =0x0301,//客户端自定义图形 机器人间通信：0x0301。发送频率：上限 10Hz
    ROBOT_COMMAND_ID                   =0x0303,

////RADAR  output(write)
    CMD_MAP_ROBOT_DATA_T               =0x0305,// 选手端小地图接收雷达数据，频率上限为 10Hz
    CMD_ROBOT_INTERACTION              =0x0301,// 车间通信
    INTERACTION_RADAR_DECISION         =0x0121,// 雷达自主决策指令
////RADAR input(read)
    CMD_RADAR_MARK_DATA_T              =0x020C,// 雷达标记进度数据，固定以 1Hz频率发送
    CMD_RADAR_INFO_T                   =0x020E, // 雷达自主决策信息同步，固定以1Hz 频率发送
    CMD_SENTRY_RADAR_T                 =0x02FD //哨兵2雷达的子内容cmd_id
} judge_data_cmd_id_e;



//typedef struct
//{
//    uint8_t figure_name[3];
//    uint32_t operate_tpye:3;
//    uint32_t figure_tpye:3;
//    uint32_t layer:4;
//    uint32_t color:4;
//    uint32_t details_a:9;
//    uint32_t details_b:9;
//    uint32_t width:10;
//    uint32_t start_x:11;
//    uint32_t start_y:11;
//    uint32_t details_c:10;
//    uint32_t details_d:11;
//    uint32_t details_e:11;
//}interaction_figure_t;

//union InteractionFigureUnion {
//    interaction_figure_t data;
//    uint8_t bytes[sizeof(interaction_figure_t)];
//};




#pragma pack(1) // #pragma pack(1)指令确保结构体成员之间紧凑地排列，没有填充字节。


typedef union
{
    uint16_t data;
    unsigned char u_char8[2];
} uint16_t_uchar;


typedef union {
    struct {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
    }data;
    unsigned char u_char8[6];

}CHLID_FRAME_HEADER;


typedef union {
    struct {
        unsigned char sof;
        uint16_t data_length;
        unsigned char seq;
        unsigned char CRC8;
    }data;
    unsigned char u_char8[5];

}FRAME_HEADER;

//-----------------------------------------------

typedef union{
    struct {
        uint16_t target_robot_ID;
        float target_position_x;
        float target_position_y;
    }data;
    unsigned char u_char8[10];
}MAP_ROBOT_DATA_T_OLD;


typedef union{
    struct{
        uint16_t hero_position_x;
        uint16_t hero_position_y;
        uint16_t engineer_position_x;
        uint16_t engineer_position_y;
        uint16_t infantry_3_position_x;
        uint16_t infantry_3_position_y;
        uint16_t infantry_4_position_x;
        uint16_t infantry_4_position_y;
        uint16_t infantry_5_position_x;
        uint16_t infantry_5_position_y;
        uint16_t sentry_position_x;
        uint16_t sentry_position_y;
    }data;
    unsigned char u_char8[24];
}MAP_ROBOT_DATA_T;

typedef union{
    struct {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
        uint8_t user_data[1];
    }data;
    unsigned char  u_char8[sizeof(data)];
}RADAR_DECISION_DATA_T;


typedef struct
{
    uint8_t figure_name[3];
    uint32_t operate_tpye:3;
    uint32_t figure_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t details_a:9;
    uint32_t details_b:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t details_c:10;
    uint32_t details_d:11;
    uint32_t details_e:11;
}interaction_figure_t;

typedef union {
    struct {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
//        uint8_t user_data[45];
        interaction_figure_t interactionFigure;
        uint8_t char_data[30];
    } data;
    unsigned char  u_char8[sizeof(data)];
}RADAR_DRAW_CAHR_DATA_T;


typedef union {
    struct {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
//        uint8_t user_data[45];
        uint8_t char_data[13];
    } data;
    unsigned char  u_char8[sizeof(data)];
}RADAR_SENF_TO_PLANE_DATA_T;

typedef union {
    struct {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
        //        uint8_t user_data[45];
        uint16_t position[10];
        int16_t speed[10];
    } data;
    unsigned char  u_char8[sizeof(data)];
}RADAR_SEND_TO_SENTRY_DATA_T;

typedef union {
    struct {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
        //        uint8_t user_data[45];
        float char_data[10];
    } data;
    unsigned char  u_char8[sizeof(data)];
}RADAR_RECIEVE_SENTRY_DATA_T;

//typedef union{
//    struct
//    {
//        interaction_figure_t interactionFigure;
//        uint8_t char_data[30];
//    } data;
//    uint8_t bytes[sizeof(data)]; // 45
//}RADAR_DRAW_CAHR_DATA_T;
//------------read------------------------------

//cmd_id 0x0105
typedef union {
    struct
    {
        uint8_t dart_remaining_time;
        uint16_t new_hit_target:3;
        uint16_t cumulative_hit_time:3;
        uint16_t target:2;
        uint16_t nothing:8;
    }data;
    unsigned char u_char8[3];
}DART_INFO_T;

// cmd_id 0x020C
typedef union {
    struct
    {
        uint8_t mark_hero_progress:1;
        uint8_t mark_engineer_progress:1;
        uint8_t mark_standard_3_progress:1;
        uint8_t mark_standard_4_progress:1;
        uint8_t mark_sentry_progress:1;
        uint8_t else_:3;
    }data;
    unsigned char u_char8[1];
}RADAR_MARK_DATA_T;


// 0x020E
typedef union {
    struct
    {
        uint8_t radar_info:2;
        uint8_t dacideing:1;
        uint8_t else_:5;
    }data;
    unsigned char u_char8[1];
}RADAR_INFO_T;

typedef union{
    struct {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
        uint8_t user_data[1];
    }data;
    unsigned char u_char8[7];
}VULNERABILITY_DATA_T;       //TODO:


typedef union {
    struct
    {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
    }data;
    unsigned char u_char8[11];
}GAME_STATUS_T;


typedef union {
    struct
    {
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t red_5_robot_HP;
        uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP;
        uint16_t else1;
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t else2;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP;
        uint16_t blue_base_HP;
    }data;
    unsigned char u_char8[32];
}GAME_ROBOT_HP_T;

typedef union {
    struct
    {
        uint32_t supply_not_overlap:1;
        uint32_t supply_overlap:1;
        uint32_t supply_area:1;

        uint32_t small_buff:1;
        uint32_t big_buff:1;

        uint32_t circular_heights:2;
        uint32_t trapezoidal_heights:2;

        uint32_t dart_hit_time:9;
        uint32_t dart_hit_target:3;
        uint32_t center_gain:2;

        uint32_t else_:9;

    }data;
    unsigned char u_char8[4];
}EVENT_DATA_T;

//
//// 0x020C read
//typedef union {
//     struct {
//        uint8_t mark_hero_progress;
//        uint8_t mark_engineer_progress;
//        uint8_t mark_standard_3_progress;
//        uint8_t mark_standard_4_progress;
//        uint8_t mark_standard_5_progress;
//        uint8_t mark_sentry_progress;
//    } data;
//    unsigned char u_char8[6];
//}RADAR_MARK_DATA_T;


#pragma pack()


/***********************************    ↓    DJI提供的CRC校检函数   ↓  ***********************************/

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);\

/***********************************    ↑    DJI提供的CRC校检函数   ↑  ***********************************/


#endif
