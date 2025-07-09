#include "img_process_00.h"
#include "math.h"
//#include "fuse.h"


/******此处设置为偏差几行到几行，间距最好是20，方便调PID********/
#define CONTROLL_H_MAX  90
#define CONTROLL_H_MIN  70

uint8 controll_h_max = 80;  //80
uint8 controll_h_min;

uint32 u32ChuKu_Time=0;  //发车计时
uint32 u32Barrier_Time=0;  //横断计时

uint16 time_prevent_check_startline;

 /***********车放在直道时测量的宽度********/
uint8  road_width_measured[UVC_HEIGHT-1] =
{
    159,159,159,159,159,159,159,159,159,159, 
    159,159,159,159,159,159,159,159,159,159, 
    159,159,159,157,156,156,155,154,153,153, 
    152,152,150,150,149,149,148,147,146,145, 
    145,144,143,142,142,141,140,139,138,138, 
    137,136,135,134,134,131,129,127,125,124, 
    122,120,118,116,115,113,111,109,107,106, 
    104,102,100,98,96,95,93,91,89,87, 
    86,84,81,79,77,76,74,72,70,68, 
    66,65,62,61,58,58,55,54,51,49, 
    48,46,44,42,39,39,36,34,32,30, 
    28,26,28
}; //第i行的赛道宽度是road_width_measured[UVC_HEIGHT-1-i]

/**********元素处理结构体**********/
struct YUAN_SU road_type, stage, distance, num;

uint8 image_yuanshi[UVC_HEIGHT][UVC_WIDTH];      //原始图像
uint8 image_01[UVC_HEIGHT][UVC_WIDTH];           //二值化图像
uint8  op;

void Show_Judge_Cirque(void)
{
 
}

void Show_Right_Cirque()
{

}


////摄像头处理全流程
void Camera_All_Deal(void)
{
    if(run_state != 6)
    {
        Transfer_Camera(rgay_image, image_yuanshi[0], UVC_WIDTH*UVC_HEIGHT);  //图像转存


        Get01change_Dajin();                          //图像二值化

        Longest_White_Column();                     //最长白列
        Search_Line();                                //搜线
        //Show_Judge_Cirque();
        //Show_Left_Cirque();
        //Show_Right_Cirque();
 
        if(run_state == 0 || run_state == 1 || run_state == 3)
    	{
            Element_Test();                               //元素识别
            Calculate_Offset();                           //计算偏差
        }
        else if (run_state == 4)
        {
            QianZhan(UVC_HEIGHT-1, 1);
            if(ofs_last < -6)
            {
                for(uint8 i = UVC_HEIGHT -1; i > 10; i--)
                {
                    l_line_x[i] = r_line_x[i] - road_width_measured[UVC_HEIGHT -1 -i] + 8;
                }
            }
            if(Qianzhan_Middle < 30)
            {
                run_state = 5;
            }
            Calculate_Offset();    
        }
        else if(run_state == 2)
        {
            Element_Test();
            if(ofs_last > 8)
            {
                for(uint8 i = UVC_HEIGHT -1; i > 10; i--)
                {
                    r_line_x[i] = l_line_x[i] + road_width_measured[UVC_HEIGHT -1 -i] + 25;
                }
            }
            Calculate_Offset();
        }
        else if(run_state == 5)
        {
            // system_delay_ms(1000);
            servocontrol(0);
            pwm_set_duty(MOTOR1_PWM, 0);
            pwm_set_duty(MOTOR2_PWM, 0);
            if(func_abs(encoder_left + encoder_right) < 5 )
                run_state = 6;
        }
        else if(run_state == 7)
        {
            
        }
        //Element_Handle();                             //元素处理
        //Handle_Right_Cirque();

    }

    


}


void Camera_Parameter_Init(st_Camera *sptr)
{
    sptr->Ramp = 0;
    sptr->straight = 0;
    sptr->bend  = 0;
}

//-----------------------------------------------------------------------------------------
//  @brief      图像转存
//  @param      *p              待转存数组地址
//  @param      *q              转存后数组地址
//  @param      pixel_num       数组长度
//  @return     void
//  Sample usage:    Transfer_Camera(mt9v03x_image[0], image_yuanshi[0], UVC_WIDTH*UVC_HEIGHT);
//-----------------------------------------------------------------------------------------
void Transfer_Camera(uint8 *p, uint8 *q, int32 pixel_num)
{
    for(int32 i = 0; i < pixel_num; i++)
        *(q +i) = *(p +i);
}

//-----------------------------------------------------------------------------------------
//  @brief      求阈值并二值化
//  @param      void
//  @return     void
//  Sample usage:    Get01change_Dajin();
//  explain       该函数里面调用了Threshold_Deal(image_yuanshi[0], UVC_WIDTH, UVC_HEIGHT, Threshold_detach);
//-----------------------------------------------------------------------------------------
uint8 Threshold;                //阈值
uint8 Threshold_static = 40;   //阈值静态下限225
uint16 Threshold_detach = 255;  //阳光算法分割阈值
void Get01change_Dajin()
{
      Threshold = Threshold_Deal(image_yuanshi[0], UVC_WIDTH, UVC_HEIGHT, Threshold_detach);

      if (Threshold < Threshold_static)
      {
          Threshold = Threshold_static;
      }

      uint8 thre;
      for(uint8 y = 0; y < UVC_HEIGHT; y++)  //这里考虑到了图像边缘的光照偏弱的问题
      {
        for(uint8 x = 0; x < UVC_WIDTH; x++)
        {
          if (x <= 15)                      //受影响的边缘范围
            thre = Threshold - 10;          //修改可调节图像边缘的清晰度
          else if (x >= UVC_WIDTH-15)
            thre = Threshold - 10;
          else
            thre = Threshold;

          if (image_yuanshi[y][x] > thre)         //数值越大，显示的内容越多，较浅的图像也能显示出来
            image_01[y][x] = 255;  //白
          else
            image_01[y][x] = 0;   //黑
        }
      }
}

//-----------------------------------------------------------------------------------------
//  @brief      求阈值
//  @param      *image              待求阈值的图像数组
//  @param      x                   图像宽度
//  @param      y                   图像高度
//  @param      pixel_threshold     阳光算法分割阈值
//  @return     void
//  Sample usage:    Threshold_Deal(image_yuanshi[0], UVC_WIDTH, UVC_HEIGHT, Threshold_detach);
//  explain：       该函数在Get01change_Dajin();里面被调用
//  Threshold = Threshold_Deal(image_yuanshi[0], UVC_WIDTH, UVC_HEIGHT, Threshold_detach);
//-----------------------------------------------------------------------------------------

uint8 Threshold_Deal(uint8* image, uint16 x, uint16 y, uint32 pixel_threshold)
{
      #define GrayScale 256
      uint16 width = x;
      uint16 height = y;
      int pixelCount[GrayScale];
      float pixelPro[GrayScale];
      int i, j;
      int pixelSum = width * height;
      uint8 threshold = 0;
      uint8* data = image;  //指向像素数据的指针
      for (i = 0; i < GrayScale; i++)
      {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
      }
      uint32 gray_sum = 0;
      //统计灰度级中每个像素在整幅图像中的个数
      for (i = 0; i < height; i += 1)
      {
        for (j = 0; j < width; j += 1)
        {
          pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
          gray_sum += (int)data[i * width + j];  //灰度值总和
          //}
        }
      }
      //计算每个像素值的点在整幅图像中的比例
      for (i = 0; i < GrayScale; i++)
      {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
      }

      //遍历灰度级[0,255]
      float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
      w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
      for (uint32 j = 0; j < pixel_threshold; j++)
      {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和 即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

        w1 = 1 - w0;
        u1tmp = gray_sum / pixelSum - u0tmp;

        u0 = u0tmp / w0;    //背景平均灰度
        u1 = u1tmp / w1;    //前景平均灰度
        u = u0tmp + u1tmp;  //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
          deltaMax = deltaTmp;
          threshold = (uint8)j;
        }
        if (deltaTmp < deltaMax)
        {
          break;
        }
      }
  return threshold;
}

//像素滤波
void Pixle_Filter()
{
  for(uint8 y = 10; y < UVC_HEIGHT-10; y++)
  {
    for(uint8 x = 10; x < UVC_WIDTH -10; x++)
    {
      if((image_01[y][x] == 0) && (image_01[y-1][x] + image_01[y+1][x] + image_01[y][x-1] + image_01[y][x+1] >= 3*255))
      { //一个黑点的上下左右的白点大于等于三个，令这个点为白
          image_01[y][x] = 255;
      }
      else if((image_01[y][x] != 0) && (image_01[y-1][x] + image_01[y+1][x] + image_01[y][x-1] + image_01[y][x+1] < 2*255))
       {//一个白点的上下左右的黑点大于等于三个，令这个点为黑
          image_01[y][x] = 0;
       }
    }
  }
}

uint8 search_line_end = 0;
int16 l_line_x[UVC_HEIGHT], r_line_x[UVC_HEIGHT], m_line_x[UVC_HEIGHT];        //储存原始图像的左右边界的列数
int16 l_line_x_l[UVC_HEIGHT], r_line_x_l[UVC_HEIGHT], m_line_x_l[UVC_HEIGHT];  //储存原始图像的左右边界的列数
uint8 l_lose_value = 0, r_lose_value = 0;                                   //左右丢线数
uint8 both_lose_value = 0;                                     //两边都丢线数
uint8 l_search_flag[UVC_HEIGHT], r_search_flag[UVC_HEIGHT];                   //是否搜到线的标志
uint8 l_width, r_width;                                                     //循环变量名
uint8 l_search_start, r_search_start;                                       //搜线起始x坐标
uint8 r_losemax, l_losemax;
uint8 l_line_x_yuanshi[UVC_HEIGHT], r_line_x_yuanshi[UVC_HEIGHT];
uint8 road_width[UVC_HEIGHT];

int Boundry_Start_Right = 0; //右边界起始位置
int Boundry_Start_Left = 0;  //左边界起始位置
int  Longest_White_Column_Left[2]={0};
int Longest_White_Column_Right[2]={0};
//-----------------------------------------------------------------------------------------
//  @brief      搜左右边线
//  @param      void
//  @return     void
//  Sample usage:    Search_Line();
//-----------------------------------------------------------------------------------------
void Longest_White_Column()//最长白列巡线
{
    int i, j;
    int start_column=20;//最长白列的搜索区间
    int end_column=UVC_WIDTH-20;
    int White_Column[UVC_WIDTH]={0};
    Boundry_Start_Left  = 0;//第一个非丢线点,常规边界起始点
    Boundry_Start_Right = 0;
    r_lose_value = 0;                              //搜线之前将丢线数清零
    l_lose_value = 0;                              //搜线之前将丢线数清零
    Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    
    for(i=0;i<=UVC_WIDTH-1;i++)
    {
        White_Column[i] = 0;
    }


    //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
    for (j =start_column; j<=end_column; j++)
    {
        for (i = UVC_HEIGHT - 1; i >= 0; i--)
        {
            if(image_01[i][j] == 0)
                break;
            else
                White_Column[j]++;
        }
    }

    //从左到右找左边最长白列
    Longest_White_Column_Left[0] =0;
    for(i=start_column;i<=end_column;i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
        }
    }
    //从右到左找右左边最长白列
    Longest_White_Column_Right[0] = 0;//【0】是白列长度
    for(i=end_column;i>=start_column;i--)//从右往左，注意条件，找到左边最长白列位置就可以停了
    {
        if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
        }
    }

} 

//-----------------------------------------------------------------------------------------
//  @brief      搜左右边线
//  @param      void
//  @return     void
//  Sample usage:    Search_Line();
//-----------------------------------------------------------------------------------------
 void Search_Line()
 {
     uint8 l_flag = 0, r_flag = 0;
     r_lose_value = l_lose_value = 0;                              //搜线之前将丢线数清零
    search_line_end = Longest_White_Column_Left[0];
    l_search_start = Longest_White_Column_Left[1]; //左边搜线起始横坐标
    r_search_start = Longest_White_Column_Right[1]; //右边搜线起始横坐标
     for(uint8 y = UVC_HEIGHT - 1; y > UVC_HEIGHT - search_line_end; y--)        //确定每行的搜线起始横坐标
     {

         for(l_width = l_search_start; l_width > 1; l_width--)      //左边搜线
         {
            if(image_01[y][l_width -2] == 0 && image_01[y][l_width - 1] == 0 && image_01[y][l_width] != 0 && l_width > 2)
            {//黑黑白
                l_line_x[y] = l_width - 1;
                l_line_x_yuanshi[y] = l_width - 1;
                l_search_flag[y] = 1;
                if(Boundry_Start_Left == 0) //第一个非丢线点
                {
                    Boundry_Start_Left = y;
                }
                break;
            }
            else if(l_width == 2)
            {
                if(l_flag == 0)
                {
                    l_flag = 1;
                    l_losemax = y +1;
                }

                l_line_x[y] = 0;
                l_line_x_yuanshi[y] = 0;
                l_search_flag[y] = 0;
                l_lose_value++;
              break;
            }
          }

          for(r_width = r_search_start; r_width < (UVC_WIDTH - 2); r_width++)      //右边搜线
             {
                 if(image_01[y][r_width] != 0 && image_01[y][r_width +1] == 0 && image_01[y][r_width +2] == 0 && r_width < UVC_WIDTH - 3)
                 {//白黑黑
                     r_line_x[y] = r_width + 1;
                     r_line_x_yuanshi[y] = r_width + 1;
                     r_search_flag[y] = 1;
                        if(Boundry_Start_Right == 0) //第一个非丢线点
                        {
                            Boundry_Start_Right = y;
                        }
                     break;
                 }
                 else if(r_width == UVC_WIDTH - 3)
                 {
                     if(r_flag==0)
                     {
                         r_flag = 1;
                         r_losemax = y + 1;
                     }
                 r_line_x[y] = UVC_WIDTH - 1;
                 r_line_x_yuanshi[y] = UVC_WIDTH - 1;
                 r_search_flag[y] = 0;
                 r_lose_value++;
                    break;
                }
             }
             if(l_search_flag[y] == 0 && r_search_flag[y] == 0) //两边都丢线
             {
                 both_lose_value++;
             }
          road_width[y] = r_line_x[y] - l_line_x[y];
          m_line_x[y] = (l_line_x[y] + r_line_x[y])/2;

         }
         for (int y = UVC_HEIGHT - search_line_end; y > 0; y--)
         {
            l_line_x[y] = 0;
            r_line_x[y] = UVC_WIDTH - 1;
         }
        //  printf("r_lose_value = %d.\r\n", r_lose_value);
 }


 /************************************************************
 【函数名称】Lost_line_right
 【功    能】右侧图像丢线检查函数
 【参    数】无
 【返 回 值】-1为未丢线 其他为丢线起始行
 【实    例】Lost_line_right();
 【注意事项】无
 ************************************************************/

 int8 Lost_line_right(void)
 {
   uint8 i;
   for(i=60;i>20;i--)
   if(r_line_x[i]==179)     return i;
   return -1;
 }

 /************************************************************

 【函数名称】Lost_line_left
 【功    能】左侧图像丢线检查函数
 【参    数】无
 【返 回 值】-1为未丢线 其他为丢线起始行
 【实    例】Lost_line_left();
 【注意事项】无
 ************************************************************/

 int8 Lost_line_left(void)
 {
   uint8 i;
   for(i=60;i>20;i--)
   if(l_line_x[i]==0) return i;
   return -1;
 }

 //检测边线
 uint8 length_l_line, length_r_line;
 void Check_bianxian(uint8 type)
 {
     uint8 length_l_line, length_r_line;
     if(type == 1)
     {
         length_l_line = 0;
         for(uint8 y = UVC_HEIGHT-1; y > 10; y--)
         {
             if((l_line_x[y] <= l_line_x[y-1]) && l_line_x[y] != 0)   //两边不丢线才计算直道长度
             {
                 length_l_line++;
             }
         }
     }
     else if(type == 2)
     {
         length_r_line = 0;
         for(uint8 y = UVC_HEIGHT-1; y > 10; y--)
         {
             if((r_line_x[y] >= r_line_x[y-1]) && r_line_x[y] != UVC_WIDTH -1)
             {
                 length_r_line++;
             }
         }
     }
 }

//-----------------------------------------------------------------------------------------
//  @brief      偏差计算
//  @param      void
//  @return     void
//  Sample usage:    Calculate_Offset();
//-----------------------------------------------------------------------------------------
uint16 bizhangtime,bizhangertime;
uint8 bizhangset;
float offset_1;
float offset;    //摄像头处理得到的偏差(车在偏左侧是负值，在偏右侧是正值)

void Calculate_Offset()
{
    offset_1 = 0;
    for(uint8 y = UVC_HEIGHT -1; y >= 10; y--)
    {
        m_line_x[y] = (l_line_x[y] + r_line_x[y])/2;
    }

    for(uint8 y = CONTROLL_H_MAX; y > CONTROLL_H_MIN; y--)//for(uint8 y = UVC_HEIGHT -35; y >=UVC_HEIGHT -45; y--)
    {
        offset_1 += (m_line_x[y] - UVC_WIDTH/2);
    }
    if(road_type.L_RoadBlock)
    {
            offset_1 += 200;
    }
    else if(road_type.R_RoadBlock)
    {
            offset_1 -= 200;
    }

       offset = (float)(offset_1/20);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      元素识别处理
//  @param      void
//  @return     void
//  Sample usage:   Element_Test();
//-------------------------------------------------------------------------------------------------------------------
void Element_Test()
{
    //imu660ra_get_acc();
    Check_bianxian(1);
    Check_bianxian(2);

    QianZhan(UVC_HEIGHT-1, 1);
    Have_Count(1, controll_h_max, controll_h_max -40);
    Have_Count(2, controll_h_max, controll_h_max -40);

    Test_guai (1, controll_h_max, 36);//36这个值不能太小，不然检测范围到太高的地方，检测右环岛时会误判左边也有拐点
    Test_guai (2, controll_h_max, 36);
    Test_guai (3, controll_h_max, 36);
    Test_guai (4, controll_h_max, 36);

    //Cross_Test();

    Garage_Test();

    //单独测试环岛
    //Ring_Test();
/*
    if(!road_type.Cross && !road_type.L_RoadBlock && !road_type.R_RoadBlock && !road_type.LeftCirque && !road_type.RightCirque )
    {
        if(num.LeftCirque == 0 || num.RightCirque == 0)
        {
            Ring_Test();                //判断是否为环岛
        }
        if(num.L_RoadBlock == 0 && num.R_RoadBlock == 0)
        {
            Roadblock_Test();
        }
    }
*/
}


void  Element_Handle(void)              //元素处理
{
    if(road_type.Cross)
    {
        Handle_Cross();
    }
    else if(road_type.LeftCirque)
    {
        Handle_Left_Cirque();
    }
    else if(road_type.RightCirque)
    {
        Handle_Right_Cirque();
    }

    else if(road_type.L_RoadBlock || road_type.R_RoadBlock)
    {
        Handle_Roadblock();
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      摄像头判断是否出界
//  @param      void
//  @return     void
//  Sample usage:    Outside_protect(100);
//-------------------------------------------------------------------------------------------------------------------
uint8 flag_stop;
void Outside_protect(uint8 value)
{
    uint8 j = 0;
    for(int16 x = 0; x < UVC_WIDTH - 1; x++)
    {
        if(image_01[UVC_HEIGHT -2][x] == 0 )  //&& 0 == road_type.barrier &&  Element == nothing
        {
            j++;
            if(j > value )  //摄像头识别出界
            {
                flag_stop = 1;
                //SysFlag.Stop_Run = ON;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief     获取赛道宽度
//  @param      void
//  @return     void
//  Sample usage:    Get_Width_Road();
//  explain：  通过蓝牙传回来，使用逐飞默认串口 在cpu_main.c中的debug_init()声明了默认串口;
//-------------------------------------------------------------------------------------------------------------------
void Get_Width_Road(void)
{
    //打印赛道宽度
    for(uint8 y = UVC_HEIGHT-1; y > search_line_end; y--)
    {
        if(y == UVC_HEIGHT-1)
        {
            printf("%d.\r\n",  1022);
            printf(",");
        }
        if(y%10 == 0)
        {
            printf("%d", r_line_x[y] - l_line_x[y]);
            printf(", \n");
        }
        else
        {
            printf("%d", r_line_x[y] - l_line_x[y]);
            printf(",");
        }
    }
}


void Search_Ring(void)
{
    //1.找中线白点
    //2.找左右丢线点
}


uint8 Qianzhan_Middle, Qianzhan_Left, Qianzhan_Right;
void QianZhan(uint8 start_line,uint8 end_line)  //type:0,中线白点，1左边，2右边
{

      Qianzhan_Middle = 0;
      for(uint8 x = start_line; x > end_line; x--)
      {
          if(image_01[x][UVC_WIDTH/2] != 0)
          {
              Qianzhan_Middle++;
          }
          else
          {
            break;
          }
      }

       Qianzhan_Left=0;
     for(uint8 x=start_line;x>end_line;x--)
        {
           if(image_01[x][UVC_WIDTH/2-30]!=0)
          {
            Qianzhan_Left++;
           }
           else
           {
             break;
           }
        }

     Qianzhan_Right = 0;
     for(uint8 x=start_line;x>end_line;x--)
    {
       if(image_01[x][UVC_WIDTH/2+30]!=0)
      {
        Qianzhan_Right++;
       }
         else
         {
            break;
         }
    }
   }


uint8 r_haveline, l_haveline;
void Have_Count(uint8 type, uint8 start_line, uint8 end_line)
{
    if(type == 1)
    {
       l_haveline = 0;
       for(uint8 i = start_line; i > end_line; i--)
       {
           if(l_search_flag[i] == 1)
           {
               l_haveline++;
           }
       }
    }
    else if(type == 2)
   {
      r_haveline = 0;
      for(uint8 i = start_line; i > end_line; i--)
      {
          if(r_search_flag[i] == 1)
          {
              r_haveline++;
          }
      }
   }
}
uint8 r_search_flag_62 = 0;
uint8 r_search_flag_61 = 0;
uint8 r_search_flag_60 = 0;
uint8 r_search_flag_59 = 0;
uint8 r_search_flag_58 = 0;
uint8 r_search_flag_57 = 0;
/********************用于检测圆环拐点****************/
uint8 left_guai_down, right_guai_down;
uint8 left_guai_up, right_guai_up;
void Test_guai(uint8 type, uint8 start_line, uint8 end_line)
{
    if(type == 1)
    {
       left_guai_down = 0;
       for(uint8 i = start_line; i > end_line; i--)
       {
           if(l_search_flag[i+1] == 1 && l_search_flag[i] == 1 && l_search_flag[i -1] == 0 && l_search_flag[i -2] == 0)
           {
               left_guai_down = i;
               break;
           }
       }
    }
    else if(type == 2)
    {
        right_guai_down = 0;
        for(uint8 i = start_line; i > end_line; i--)
        {
            if(r_search_flag[i +1] == 1 && r_search_flag[i] == 1 && r_search_flag[i -1] == 0 && r_search_flag[i -2] == 0)
            {
                right_guai_down = i;
                break;
            }
        }
    }
    else if(type == 3)
    {
       left_guai_up = 0;
       for(uint8 i = start_line; i > end_line; i--)
       {
           if(l_search_flag[i+1] == 0 && l_search_flag[i] == 0 && l_search_flag[i -1] == 1 && l_search_flag[i -2] == 1 )
           {
               left_guai_up = i;
               break;
           }
       }
    }
    else if(type == 4)
    {
        right_guai_up = 0;
        for(uint8 i = start_line; i > end_line; i--)
        {
            if(r_search_flag[i +1] == 0 && r_search_flag[i] == 0 && r_search_flag[i -1] == 1 && r_search_flag[i -2] == 1 )
            {
                right_guai_up = i;
                break;
            }
        }

    }
}
/********************Test_guai_end****************/

/********************用于检测拐点周围点****************/
uint8 Point_Guai_right,Point_Guai_left;
void Count_Point(uint8 type,uint8 start_line)
{
    if(type==1&&start_line!=0)
    {
        Point_Guai_left=0;
        for(uint8 i=start_line-2;i>start_line-7;i--)
          {
              for(uint8 j=l_line_x[start_line+2];j<l_line_x[start_line+2]-10;j--)
              {
                  if(image_01[i][j]==1)
                  {
                      Point_Guai_left++;
                  }
              }
          }
    }
    else if(type==2&&start_line!=0)
    {
        Point_Guai_right=0;
   for(uint8 i=start_line-2;i>start_line-7;i--)
   {
       for(uint8 j=r_line_x[start_line+2];j<r_line_x[start_line-2]+10;j++)
       {
           if(image_01[i][j]==1)
           {
               Point_Guai_right++;
           }
       }
   }
  }
}
/********************Count_Point_end****************/

/********************用于检测环岛****************/
void Ring_Test(void)
{
    if(Qianzhan_Middle > 90 && left_guai_down != 0 && right_guai_down == 0 && r_haveline >= 38 && l_haveline < 15 && !road_type.RightCirque)
    {
        road_type.LeftCirque = 1;
        num.LeftCirque++;
    }
    if(Qianzhan_Middle > 90 && left_guai_down == 0 && right_guai_down != 0 && r_haveline < 15 && l_haveline >= 38 && !road_type.LeftCirque)
    {
        road_type.RightCirque = 1;
        num.RightCirque++;
    }
}
/********************Ring_Test_end****************/

/********************用于检测障碍物****************/
void Roadblock_Test(void)
{
    if(l_line_x[50] - l_line_x[60] > 15 && abs(l_line_x[45] - l_line_x[50]) < 5 && r_lose_value < 10&& l_lose_value < 10)  //检测左边是否有路障
    {
        //gpio_set_level(E9,0);
        road_type.L_RoadBlock = 1;
        num.L_RoadBlock++;
    }
    else if(r_line_x[60] - r_line_x[50] > 15  && abs(r_line_x[45] - r_line_x[50]) < 5 && r_lose_value < 10 && l_lose_value < 10)  //检测右边是否有路障
    {
        //gpio_set_level(E9,0);
        road_type.R_RoadBlock = 1;
        num.R_RoadBlock++;
    }
}

/********************用于检测十字路口****************/
void Cross_Test(void)
{
    if(Qianzhan_Middle > 90 && l_lose_value > 38 && r_lose_value > 38)
    {
        //Find_Up_Point(5,80);
        //if(Left_Up_Find!=0 && Right_Up_Find!=0)
            road_type.Cross = 1;
    }
}
/********************Cross_Test_end****************/


void Garage_Test(void)
{
    Find_Up_Point(80,UVC_HEIGHT - Longest_White_Column_Left[0] + 15) ;
    if(road_type.Garage == 0 && Qianzhan_Middle > 90 && r_lose_value > 35 && (right_guai_down > 55 || Right_Up_Find > 30))
    {
        //Find_Up_Point(5,80);
        //if(Left_Up_Find!=0 && Right_Up_Find!=0)
            road_type.Garage = 1;
            if(run_state == 0)
            	run_state = 1;
            if(run_state == 2)
            	run_state = 3;
            gpio_set_level(BEEP, 0x1);
    }
    if (run_state == 1)
    {
        if(road_type.Garage == 1)
        {
            if(Right_Up_Find > 50 || r_lose_value > 90)
            {
                road_type.Garage = 2;
            }
            for(uint8 i = UVC_HEIGHT -1; i > UVC_HEIGHT - Longest_White_Column_Left[0]; i--)
            {
                r_line_x[i] = l_line_x[i] + road_width_measured[UVC_HEIGHT -1 -i] + 20;
            }

        }
        else if(road_type.Garage == 2)
        {
            if(Right_Up_Find < 20 || (r_line_x[right_guai_up] > 140 && right_guai_up > 80) )
            {
                road_type.Garage = 3;
                gpio_set_level(BEEP, 0x0);
                movelength = 0;
            }

        }
        else if(road_type.Garage == 3)//为了增加扩展性添加的过渡状态，实际上这个状态没有用到
        {
            if(movelength > 60)
            {
                road_type.Garage = 5;
                movelength = 0;
            }
        }
        else if(road_type.Garage == 4)// 开环时的判断停止状态，闭环时不用
        {
            if((encoder_left + encoder_right) <= 20)
            {
                road_type.Garage = 5;
                movelength = 0;
            }
        }
        else if(road_type.Garage == 5)
        {

            if(movelength < -15)
            {
                road_type.Garage = 51;
            }
        }
        else if(road_type.Garage == 51)
        {
            if(func_abs(Qianzhan_Left - Qianzhan_Right) < 3 && func_abs(offset) < 4)
            {
                road_type.Garage = 6;
            }
        }
        else if(road_type.Garage == 6)
        {
            if(Qianzhan_Middle > 80)
            {
                // road_type.Garage = 7;

                run_state = -1;
            }
        }
        else if(road_type.Garage == 7)
        {
            if(Qianzhan_Middle < 70)
            {
                road_type.Garage = 8;
                gpio_set_level(BEEP, 0x1);
                movelength = 0;
            }
        }
        else if(road_type.Garage == 8)
        {
            if(movelength > 30 )
            {
                road_type.Garage = 81;
            }
        }
        else if(road_type.Garage == 81)
        {
            if(func_abs(offset) < 10 )
            {
                gpio_set_level(BEEP, 0x0);
                road_type.Garage = 0;
                run_state = 2;
            }
        }
        else if(road_type.Garage == 9)//临时状态
        {
            
        }
    }
    if(run_state == 3)
    {
        if(road_type.Garage == 1)
        {
            if(Right_Up_Find > 30 )
            {
                road_type.Garage = 2;
            }
            for(uint8 i = UVC_HEIGHT -1; i > UVC_HEIGHT - Longest_White_Column_Left[0]; i--)
            {
                r_line_x[i] = l_line_x[i] + road_width_measured[UVC_HEIGHT -1 -i] + 20;
            }

        }
        else if(road_type.Garage == 2)
        {
            if(Right_Up_Find < 20 || (r_line_x[right_guai_up] > 140 && right_guai_up > 80) )
            {
                road_type.Garage = 3;
                gpio_set_level(BEEP, 0x0);
                movelength = 0;
            }
            for(uint8 i = UVC_HEIGHT -1; i > UVC_HEIGHT - Longest_White_Column_Left[0]; i--)
            {
                r_line_x[i] = l_line_x[i] + road_width_measured[UVC_HEIGHT -1 -i] + 20;
            }
        }
        else if(road_type.Garage == 3)//为了增加扩展性添加的过渡状态，实际上这个状态没有用到
        {
            if(movelength > 55)
            {
                road_type.Garage = 4;
            }
        }
        else if(road_type.Garage == 4)
        {
            if(Qianzhan_Middle < 35)
            {
                road_type.Garage = 41;
                movelength = 0;
            }
        }
        else if(road_type.Garage == 41)
        {
            if(encoder_left + encoder_right <= 20)
            {
                road_type.Garage = 5;
                movelength = 0;
            }
        }
        else if(road_type.Garage == 5)
        {

            if(movelength < -20)
            {
                road_type.Garage = 51;
                movelength = 0;
            }
        }
        else if(road_type.Garage == 51)
        {

            if(movelength < -45)
            {
                road_type.Garage = 6;
            }
        }
        else if(road_type.Garage == 61)
        {
            if(Qianzhan_Middle > 20)
            {
                road_type.Garage = 7;
                movelength = 0;
            }
        }

        
        else if(road_type.Garage == 6)
        {
            if(Qianzhan_Middle < 30)
            {
                road_type.Garage = 61;
                movelength = 0;
            }
        }
        else if(road_type.Garage == 7)
        {
            if(func_abs(encoder_left + encoder_right) < 20)

            {
                road_type.Garage = 71;
                gpio_set_level(BEEP, 0x1);
                movelength = 0;
            }
        }
        else if(road_type.Garage == 71)
        {
            if(Qianzhan_Middle > 50)
            {
                road_type.Garage = 81;
                gpio_set_level(BEEP, 0x1);
                movelength = 0;
            }
        }
        else if(road_type.Garage == 8)
        {
            if(movelength > 15 )
            {
                road_type.Garage = 9;
                movelength  =   0;
            }
        }
        else if(road_type.Garage == 81)
        {
            if(movelength > 30 )
            {
                road_type.Garage = 8;
                movelength = 0;
            }
        }
        else if(road_type.Garage == 9)//临时状态
        {
            if(movelength > 15)
            {
                gpio_set_level(BEEP, 0x0);
                road_type.Garage = 0;
                run_state = 4;
            }
            
        }
    }
    
}








/********************用于处理左环岛****************/
uint8 L_integration_flag = 0, R_integration_flag = 0;
void Handle_Left_Cirque(void)
{
    /*if(road_type.LeftCirque)
    {
        switch(stage.LeftCirque)
        {
            case 0:
                if(road_width[50] >75)  //中间行
                {
//                    if( r_haveline<=38)
//                    {
//                       road_type.LeftCirque = 0;
//                       num.LeftCirque=0;
//                       //break;
//                    }
                    gpio_set_level(BEEP, 0x1);
                    stage.LeftCirque = 1;
                }
            break;
            case 1:
                if(road_width[50] < 73)
                {
                    gpio_set_level(BEEP, 0x0);
                    stage.LeftCirque = 2;
                }
            break;
            case 2:
                if(road_width[50] > 86)
                {
                    gpio_set_level(BEEP, 0x1);
                    stage.LeftCirque = 3;
                }
            break;
            case 3:
                if(Left_CirqueIntegration > 90) //进环
                {
                    //BUZZER_OFF;
                    //gpio_set_level(E9,1);
                    stage.LeftCirque = 4;
                }
            break;
            case 4:
                if(Left_CirqueIntegration >600 ) //环内
                {
                    //BUZZER_ON;
                    //gpio_set_level(E9,0);
                    stage.LeftCirque = 5;
                }
            break;
            case 5:
                if(Left_CirqueIntegration > 1000) //出环
                {
                    //BUZZER_OFF;
                    //gpio_set_level(E9,1);
                    stage.LeftCirque=6;
                }
            break;
            case 6:
                if(Left_CirqueIntegration > 1200)  //直行
                {
                    //BUZZER_ON;
                    stage.LeftCirque = 7;
                    //gpio_set_level(E9,0);
                }
            break;
        }
        if(stage.LeftCirque == 0 || stage.LeftCirque == 1 || stage.LeftCirque == 2)  //直行
        {
            if( r_haveline<=38)
            {
                road_type.LeftCirque = 0;
                num.LeftCirque=0;
            }
            for(uint8 i = UVC_HEIGHT-1; i > 10; i--)
            {
                l_line_x[i] = r_line_x[i] - road_width_measured[UVC_HEIGHT-1 - i]+10;
            }

        }
        if(stage.LeftCirque == 3)  //进环
        {
            L_integration_flag = 1;
            for(uint8 i = UVC_HEIGHT-1; i > 10; i--)
            {
                r_line_x[i] = l_line_x[i] + road_width_measured[UVC_HEIGHT-1 -i]-20;
            }
        }
        if(stage.LeftCirque ==4 )  //环内
        {

        }
        if(stage.LeftCirque == 5)  //出环
        {

            for(uint8 i = UVC_HEIGHT-1; i > 10; i--)
            {
                r_line_x[i] = l_line_x[i] + road_width_measured[UVC_HEIGHT -1 -i] -20;
            }
        }
        if(stage.LeftCirque == 6)  //直行
        {
            //BUZZER_OFF;
//            Search_Line();
            for(uint8 i = UVC_HEIGHT-1; i > 10; i--)
            {
                l_line_x[i] = r_line_x[i] - road_width_measured[UVC_HEIGHT -1 -i]+15;
            }
        }
        if(stage.LeftCirque == 7)
        {
            road_type.LeftCirque = 0;
            stage.LeftCirque = 0;
            L_integration_flag = 0;
            Left_CirqueIntegration = 0;
            //num.LeftCirque = 0;
        }
    }*/
}
/********************Handle_Left_Cirque_end****************/

/********************用于处理右环岛****************/
void Handle_Right_Cirque(void)
{
    /*if(road_type.RightCirque)
    {
        switch(stage.RightCirque)
        {
           case 0:
                if(road_width[60] >90)  //中间行
                {
                    gpio_set_level(BEEP, 0x1);
                    stage.RightCirque = 1;
                }
            break;
            case 1:
                if(road_width[60] < 85)
                {
                    gpio_set_level(BEEP, 0x0);
                    stage.RightCirque = 2;
                }
            break;
            case 2:
                if(road_width[60] > 90)//改
                {
                    gpio_set_level(BEEP, 0x1);
                    stage.RightCirque = 3;
                    accumulated_angle = 0;
                }
            break;
            case 3:
                //if(Right_CirqueIntegration > 90)
                if( - accumulated_angle > 45)
                {
                    gpio_set_level(BEEP, 0x0);
                    stage.RightCirque = 4;
                }
            break;
            case 4:
                if( - accumulated_angle > 210)  
                {
                    gpio_set_level(BEEP, 0x1);
                    stage.RightCirque = 5;
                }
            break;
            case 5:
                if( - accumulated_angle > 300)
                {
                    gpio_set_level(BEEP, 0x0);
                    stage.RightCirque = 6;
                }
            break;
            case 6:
                if(right_guai_up > 80)
                {
                    gpio_set_level(BEEP, 0x0);
                    stage.RightCirque = 7;
                }
            break;
        }
        if(stage.RightCirque == 0||stage.RightCirque == 1||stage.RightCirque == 2)  //直行
        {
            if(l_haveline<=38)
            {
                road_type.RightCirque = 0;
                num.RightCirque=0;
                //break;
            }
            for(uint8 i = UVC_HEIGHT -1; i > 10; i--)
            {
                r_line_x[i] = l_line_x[i] + road_width_measured[UVC_HEIGHT -1 -i];
            }
        }
        if(stage.RightCirque == 3) //进环
        {
            
            R_integration_flag = 1;

            for(uint8 i = UVC_HEIGHT -1; i > 10; i--)
            {
                //进环补线
                l_line_x[i] = r_line_x[i] - road_width_measured[UVC_HEIGHT -1 -i]-20;
            }
        }
        if(stage.RightCirque == 4) //环内
        {
            //BUZZER_OFF;
        }
        if(stage.RightCirque == 5)  //出环
        {
            //BUZZER_ON;
            for(uint8 i = UVC_HEIGHT -1; i > 10; i--)
            {
                l_line_x[i] = r_line_x[i] - road_width_measured[UVC_HEIGHT -1 -i]-15;
            }
        }
        if(stage.RightCirque == 6)  //直行
        {
            //BUZZER_OFF;
            for(uint8 i = UVC_HEIGHT -1; i > 10; i--)
            {
                r_line_x[i] = l_line_x[i] + road_width_measured[UVC_HEIGHT -1 -i];
            }
        }
        if(stage.RightCirque == 7)
        {
            stage.RightCirque = 0;
            R_integration_flag = 0;
            road_type.RightCirque = 0;
            Right_CirqueIntegration = 0;
            //num.RightCirque = 0;
        }
    }*/
}

/********************Handle_Left_Cirque_end****************/

/********************用于处理障碍物****************/
uint16 Roadblock_lent;
void Handle_Roadblock(void)
{
    //BUZZER_ON;
    if (distance.L_RoadBlock > 1500)
    {
        road_type.L_RoadBlock = 0;
        distance.L_RoadBlock  = 0;
        //gpio_set_level(E9,1);
        //BUZZER_OFF;
    }
    else if(distance.R_RoadBlock > 1500)
    {
        road_type.R_RoadBlock = 0;
        distance.R_RoadBlock  =0;
        //gpio_set_level(E9,1);
        //BUZZER_OFF;
    }
}
/********************Handle_Roadblock_end****************/

/***************************用于处理十字***********************/
//已删除

/*******************寻找十字上面的两个点*****************/
uint8 Left_Up_Find=0;
uint8 Right_Up_Find=0;
void Find_Up_Point(uint8 start,uint8 end)
{
    Left_Up_Find = 0;
    Right_Up_Find = 0;
    for(uint8 i=start;i>=end;i--)
    {

        if(abs(l_line_x[i]-l_line_x[i-2])<5&&
           abs(l_line_x[i-2]-l_line_x[i-4])<5&&
           abs(l_line_x[i-4]-l_line_x[i-6])<5&&
              (l_line_x[i]-l_line_x[i+2])>15&&
              (l_line_x[i]-l_line_x[i+4])>25&&
              (l_line_x[i]-l_line_x[i+6])>  25)
        {
            Left_Up_Find=i;//获取行数即可
        }

        if(abs(r_line_x[i]-r_line_x[i-1])<=3&&
           abs(r_line_x[i-1]-r_line_x[i-2])<=3&&
           abs(r_line_x[i-2]-r_line_x[i-3])<=3&&
              (r_line_x[i]-r_line_x[i+2])<=-5&&
              (r_line_x[i]-r_line_x[i+3])<=-7&&
              (r_line_x[i]-r_line_x[i+4])<=-7)
        {
            Right_Up_Find=i;//获取行数即可
            break;
        }
//        if(Left_Up_Find!=0&&Right_Up_Find!=0)//下面两个找到就出去
//        {
//            return;
//        }
    }
}

//    if(abs(Right_Up_Find-Left_Up_Find)>=80)//纵向撕裂过大，视为误判
//    {
//        Right_Up_Find=0;
//        Left_Up_Find=0;
//    }



/*-------------------------------------------------------------------------------------------------------------------
  @brief     找下面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int start,int end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
uint8 Right_Down_Find=0;
uint8 Left_Down_Find=0;
void Find_Down_Point(uint8 start,uint8 end)                 //用于判别正入或者斜入十字
{
//    if(start>=UVC_HEIGHT-1-5)//起始点位置校正，排除数组越界的可能
//        start=UVC_HEIGHT-1-5;
//    else if(start<=5)
//        start=5;
//    if(end>=UVC_HEIGHT-1-5)
//        end=UVC_HEIGHT-1-5;
//    else if(end<=5)
//        end=5;
    for(uint8 i=start;i>=end;i--)
    {
        if(//只找第一个符合条件的点
           abs(l_line_x[i]-l_line_x[i+1])<=5&&//角点的阈值可以更改
           abs(l_line_x[i+1]-l_line_x[i+2])<=5&&
           abs(l_line_x[i+2]-l_line_x[i+3])<=5&&
           (l_line_x[i]-l_line_x[i-2])>=8&&
           (l_line_x[i]-l_line_x[i-3])>=15&&
           (l_line_x[i]-l_line_x[i-4])>=15)
        {
            Left_Down_Find=i;//获取行数即可
        }
        if(//只找第一个符合条件的点
           abs(r_line_x[i]-r_line_x[i+1])<=5&&//角点的阈值可以更改
           abs(r_line_x[i+1]-r_line_x[i+2])<=5&&
           abs(r_line_x[i+2]-r_line_x[i+3])<=5&&
              (r_line_x[i]-r_line_x[i-2])<=-8&&
              (r_line_x[i]-r_line_x[i-3])<=-15&&
              (r_line_x[i]-r_line_x[i-4])<=-15)
        {
            Right_Down_Find=i;
        }
//        if(Left_Down_Find!=0&&Right_Down_Find!=0)//两个找到就退出
//        {
//            return;
//        }
    }

}

/********************正入十字补线****************/
void Add_Line(uint8 type,uint8 x1,int16 y1,uint8 x2,int16 y2)//左补线,补的是边界          左下（x1,y1）  左上（x2，y2）
{
    int hy;
    if(x1>=UVC_HEIGHT-1)//起始点位置校正，排除数组越界的可能
       x1=UVC_HEIGHT-1;
    else if(x1<=10)
        x1=10;
    if(y1>=UVC_WIDTH-1)
    y1=UVC_WIDTH-1;
    else if(y1<=10)
    y1=10;
    if(x2>=UVC_HEIGHT-1)
    x2=UVC_HEIGHT-1;
    else if(x2<=10)
         x2=10;
    if(y2>=UVC_WIDTH-1)
    y2=UVC_WIDTH-1;
    else if(y2<=10)
         y2=10;

    if(type == 1)//左补线,补的是边界          左下（x2,y2）  左上（x1，y1）
    {
        for(uint8 i=x2;i>=x1;i--)//根据斜率补线即可
        {
            hy=(i-x2)*(y2-y1)/(x2-x1)+y2;
            if(hy>=UVC_WIDTH-1)
                hy=UVC_WIDTH;
            else if(hy<=0)
                hy=0;
            l_line_x[i]=hy;
        }
    }
    else if(type == 2)//右补线，补的是边界         右下（x2,y2）  左上（x1，y1）
    {
        for(uint8 i=x2;i>=x1;i--)//根据斜率补线即可
        {
            hy=(i-x2)*(y2-y1)/(x2-x1)+y2;
            if(hy>=UVC_WIDTH-1)
                hy=UVC_WIDTH;
            else if(hy<=0)
                hy=0;
            r_line_x[i]=hy;
        }
    }
}
/********************斜入十字补线****************/
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右左边界延长
  @param     延长起始行数，延长到某行
  Sample     Lengthen_Right_Boundry(int start,int end)；
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Boundry(uint8 type,uint8 start,uint8 end)
{
    float k=0;
    if(start>=UVC_HEIGHT-1-5)//起始点位置校正，排除数组越界的可能
        start=UVC_HEIGHT-1-5;
    else if(start<=5)
        start=5;
    if(end>=UVC_HEIGHT-1-5)
        end=UVC_HEIGHT-1-5;
    else if(end<=5)
        end=5;
    if(type == 1)
    {
        k=(float)(l_line_x[start]-l_line_x[start-4])/5.0;//这里的k是斜率
        for(uint8 i=start;i<=end;i++)
        {
            l_line_x[i]=(int)(i-start)*k+l_line_x[start];//(x=(y-y1)*k+x1),点斜式变形
            if(l_line_x[i]>=UVC_WIDTH-1)
            {
                l_line_x[i]=UVC_WIDTH-1;
            }
            else if(l_line_x[i]<=0)
            {
                l_line_x[i]=0;
            }
        }
    }
    if(type == 2)
    {
        k=(float)(r_line_x[start]-r_line_x[start-4])/5.0;//这里的k是斜率
        for(uint8 i=start;i<=end;i++)
        {
            r_line_x[i]=(int)(i-start)*k+r_line_x[start];//(x=(y-y1)*k+x1),点斜式变形
            if(r_line_x[i]>=UVC_WIDTH-1)
            {
                r_line_x[i]=UVC_WIDTH-1;
            }
            else if(r_line_x[i]<=0)
            {
                r_line_x[i]=0;
            }
        }
    }
}
/*******************十字处理函数*****************/
void Handle_Cross()
{
    //待补充
}

/********************用于角速度积分****************/

uint32 Left_CirqueIntegration = 0, Right_CirqueIntegration = 0;
void Integration(void)
{
 // if(L_integration_flag == 1)
  // {
  //     Left_CirqueIntegration
   //}
  // if(R_integration_flag == 1)
   //{
    //   Right_CirqueIntegration = abs(accumulated_angle);
  // }
   //也可以在主函数里的pit_callback中根据r_inte_flag是否为1来判断是否进行积分，不是一就不对accumulate累加
}

/********************Test_Element_first_end****************/


void Datasend_Side_line(uint8 type)
{
    if(type == 1)
    {
        for (uint8 i = 0; i < UVC_HEIGHT - 2; i++)
        {
           if(i == 0)
           {
               printf("start");
               printf("\n");
           }
           else
           {
               printf("%d", l_line_x[i]);
               printf(",");
               if(i%10 == 0)
               {
                   printf("\n");
               }
           }
        }
    }
    else if(type == 2)
    {
        for (uint8 i = 0; i < UVC_HEIGHT - 2; i++)
        {
           if(i == 0)
           {
               printf("start");
               printf("\n");
           }
           else
           {
               printf("%d", r_line_x[i]);
               printf(",");
               if(i%10 == 0)
               {
                   printf("\n");
               }
           }
        }
    }
    else if(type == 3)
    {
        for (uint8 i = UVC_HEIGHT - 1; i > 2; i--)
        {
           if(i == UVC_HEIGHT - 1)
           {
               printf("start");
               printf("\n");
           }
           else
           {
               printf("%d", r_line_x[i] - l_line_x[i]);
               printf(",");
               if(i%10 == 0)
               {
                   printf("\n");
               }
           }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在一个点画十字
// 参数说明     void
// 返回参数     uint32 电压
// 使用示例     Voltage_Detection(ADC_VOLTAGE);
//-------------------------------------------------------------------------------------------------------------------
void Draw_Cross(uint16 x_point, uint16 y_point, const uint16 color, uint8 num)
{
    for (uint8 x = x_point -num; x < x_point +num; x++)
    {
        ips200_draw_point(x, y_point, color);
    }
    for (uint8 y = y_point -num; y < y_point +num; y++)
    {
        ips200_draw_point(x_point, y, color);
    }
}


int clip(int x, int low, int up)
{
    return x > up ? up : x < low ? low : x;
}


float fclip(float x, float low, float up)
{
    return x > up ? up : x < low ? low : x;
}


//中线滑动平均值滤波
int16 Sum1;
void HDPJ_lvbo(int16 data[], uint8 N, uint8 size)
{//data[] 要滤波的数组   N 滤波强度   size 滤波起始点
    Sum1 = 0;
    for(uint8 j = 0; j < size; j++)
    {
        if(j < N /2)
        {
            for(uint8 k = 0; k < N; k++)
            {
                Sum1 += data[j +k];
            }
            data[j] = Sum1 /N;
        }
        else if(j < size - N/2)
        {
            for(uint8 k = 0; k < N/2; k++)
            {
                Sum1 += (data[j +k] +data[j -k]);
            }
            data[j] = Sum1 /N;
        }
        else
        {
            for(uint8 k = 0; k < size -j; k++)
            {
                Sum1 += data[j +k];
            }
            for(uint8 k =0; k <(N -size +j); k++)
            {
                Sum1 += data[j -k];
            }
            data[j] = Sum1 /N;
        }
        Sum1 = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief     识别斑马线
//  @param      start_point：识别的y起点   end_point:识别的y终点  qiangdu：识别的强度
//  @return     void
//  Sample usage:      Check_Starting_Line(65, 55, 1);
//  explain：  qiangdu越小越容易判断成功，且不会误判
//             该函数后面几句代码进行了车库方向的判断
//-------------------------------------------------------------------------------------------------------------------
uint8 flag_starting_line;
uint8 black_blocks_l, black_blocks_r;
uint8 cursor_l, cursor_r;
void Check_Starting_Line(uint8 start_point, uint8 end_point, uint8 qiangdu)
{
    uint8 times = 0;
    flag_starting_line = 0;
    for(uint8 y = start_point; y >= end_point; y--)  //判断斑马线
    {
        black_blocks_l = black_blocks_r = cursor_l = cursor_r = 0;
        for(uint8 width_l = UVC_WIDTH/2,width_r = UVC_WIDTH/2; width_l > 1 && width_r < UVC_WIDTH-2; width_l--,width_r++)
        {
            if(image_01[y][width_l] == 0)
            {
                if(cursor_l > 16)
                {
                    break;    //当黑色元素超过栈长度的操作
                }
                else
                {
                    cursor_l++;
                }
            }
            else
            {
                if(cursor_l >= qiangdu && cursor_l <= qiangdu+6)
                {
                    black_blocks_l++;
                    cursor_l = 0;
                }
                else
                {
                    cursor_l = 0;
                }
            }

        if(image_01[y][width_r] == 0)
        {
            if (cursor_r > 16)
            {
                break;    //当黑色元素超过栈长度的操作
            }
            else
            {
                cursor_r++;
            }
        }
        else
        {
            if(cursor_r >= qiangdu && cursor_r <= qiangdu + 6)
            {
                black_blocks_r++;
                cursor_r = 0;
            }
            else
            {
                cursor_r = 0;
            }
        }
    }

    if((black_blocks_l + black_blocks_r) >= 4)
    {
        times++;
    }

    if(times >= (start_point -end_point -5))
    {
        flag_starting_line = 1;
    }
    else
    {
        flag_starting_line = 0;
    }
  }
}
