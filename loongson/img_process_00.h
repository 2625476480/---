#ifndef __CAMERA_H
#define __CAMERA_H
#include "zf_common_headfile.h"

//说 明：本宏函数的参数不能为表达式，只能为具体数值
#define Limit_Min_Max(data, min, max) (((data) > (max)) ? (max):(((data) < (min)) ? (min):(data)))

//赛道类型结构体
typedef struct YUAN_SU{
     int16 Ramp;                   //坡道
     int16 straight;               //直道
     int16 bend;                   //弯道
     int16 Cross;                  //十字
     int16 LeftCirque;             //左环岛
     int16 RightCirque;            //右环岛
     int16 L_RoadBlock;            //左障碍
     int16 R_RoadBlock;            //右障碍
     int16 Starting_Line;          //斑马线
     uint8 Garage;
     uint8 GarageSide; // 0-左 1-右
}st_Camera;

extern struct YUAN_SU road_type, stage, distance, num;
extern uint8 garage_detect_count;
extern uint16 time_prevent_check_startline;

extern uint8 Threshold_static;   //阈值静态下限225
void Camera_All_Deal(void);
////摄像头搜线部分///
void Horizontal_line(void);
void Longest_White_Column(void);
extern int Longest_White_Column_Right[2];
extern int Longest_White_Column_Left[2];

int8 Lost_line_right(void);
int8 Lost_line_left(void);
void Search_Line(void);
void Camera_Parameter_Init(st_Camera *sptr);
void Transfer_Camera(uint8 * p, uint8 *q, int32 pixel_num);
void Get01change_Dajin(void);
uint8 Threshold_Deal(uint8* image, uint16 col, uint16 row, uint32 pixel_threshold);

////元素识别部分///
void Check_Starting_Line(uint8 start_point, uint8 end_point, uint8 qiangdu);
extern uint8 flag_starting_line;
extern uint8 black_blocks_l, black_blocks_r;
extern uint8 cursor_l, cursor_r;
void Check_Zhidao(void);
extern uint8 sudu_yingzi;
void Check_Bianxian(uint8 type);
extern uint8 length_l, length_r;
void Diuxian_Weizhi_Test(uint8 type, uint8 startline, uint8 endline);
extern uint8 diuxian_hang, budiuxian_hang;
uint8 Bianxian_Guaidian(uint8 type, uint8 startline, uint8 endline );

////元素处理部分///
void Get_Width_Road(void);
/*****************搜索圆环部分函数************/
void Ring_Test(void);
void Roadblock_Test(void);
void Cross_Test(void);
void Garage_Test(void);
//void Check_Corss_Point(uint8 type);
void Handle_Cross();
extern uint8 Left_Up_Find;
extern uint8 Right_Up_Find;
void Find_Up_Point(uint8 start,uint8 end);
extern uint8 Right_Down_Find;
extern uint8 Left_Down_Find;
void Find_Down_Point(uint8 start,uint8 end);

void Add_Line(uint8 type,uint8 x1,int16 y1,uint8 x2,int16 y2);
void Lengthen_Boundry(uint8 type,uint8 start,uint8 end);
//extern uint8 Row_l_1, Col_l_1, Row_l_2, Col_l_2;
//extern uint8 Row_r_1, Col_r_1, Row_r_2, Col_r_2;

void Have_Count(uint8 type,uint8 start_line,uint8 end_line);
void QianZhan(uint8 start_line,uint8 end_line);  //type:0,中线白点，1左边，2右边
void Test_Element_first(void);
void Test_guai(uint8 type,uint8 start_line,uint8 end_line);
void Count_Point(uint8 type,uint8 start_line);

uint16 Check_Cirque(uint8 type, uint8 startline);
void Handle_Left_Cirque(void);
void Handle_Right_Cirque(void);
void Handle_Roadblock(void);

void Integration(void);

void HDPJ_lvbo(int16 data[], uint8 N, uint8 size);
void Element_Test(void);
void Element_Show(void);
void  Element_Handle(void);

/*******************end************************/


/*****************搜索圆环部分变量************/
extern uint8 r_haveline, l_haveline;
extern uint8 Qianzhan_Middle, Qianzhan_Left, Qianzhan_Right;
extern uint8 Point_Guai_right,Point_Guai_left;
extern uint8 left_guai_up,right_guai_up,left_guai_down,right_guai_down;

extern uint8 L_integration_flag,R_integration_flag;
extern uint32 Left_CirqueIntegration,Right_CirqueIntegration;
/*******************end************************/

////计算偏差和其他部分////

extern uint8 image_yuanshi[UVC_HEIGHT][UVC_WIDTH];
extern uint8 image_01[UVC_HEIGHT][UVC_WIDTH];
extern uint8 Threshold;
extern uint8 search_line_end;
extern uint8 road_width[UVC_HEIGHT];
extern uint8 l_lose_value, r_lose_value;
extern uint8 r_losemax, l_losemax;
extern uint8 chang_point_l,chang_point_r;

void Calculate_Offset(void);
extern float offset_1;
extern float offset;
extern uint8 flag_stop;
void Blacking(void);
extern uint8 controll_h_max ;
extern uint8 controll_h_min ;
void Outside_protect(uint8 value);
extern int16 l_line_x[UVC_HEIGHT], r_line_x[UVC_HEIGHT], m_line_x[UVC_HEIGHT];
extern int16 l_line_x_l[UVC_HEIGHT], r_line_x_l[UVC_HEIGHT], m_line_x_l[UVC_HEIGHT];
extern uint8 kuandu_saidao [UVC_HEIGHT-1];
extern uint8 flag_starting_line;
void Check_Starting_Line(uint8 start_point, uint8 end_point, uint8 qiangdu);


void Datasend_Side_line(uint8 type);
void Draw_Cross(uint16 x_point, uint16 y_point, const uint16 color, uint8 num);

void Check_bianxian(uint8 type);
extern uint8 length_l_line, length_r_line;

int clip(int x, int low, int up);
float fclip(float x, float low, float up);

void Handle_Garage(void);
void Generate_Garage_Path(void);

#endif