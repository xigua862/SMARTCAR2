/*****************************************************************************
* 文件名称: Menu.c
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     菜单界面：主菜单导航、PID参数调节、偏移量观察、参数保存/恢复/复位
*****************************************************************************/

#include "my_headfile.h"

uint8 key1_state = 1;                                                               // 按键动作状态
uint8 key2_state = 1;                                                               // 按键动作状态
uint8 key3_state = 1;                                                               // 按键动作状态
uint8 key4_state = 1;                                                               // 按键动作状态
uint8 key5_state = 1;                                                               // 按键动作状态
uint8 go_state = 1;                                                               // 按键动作状态

uint8 key1_state_last = 0;                                                          // 上一次按键动作状态
uint8 key2_state_last = 0;                                                          // 上一次按键动作状态
uint8 key3_state_last = 0;                                                          // 上一次按键动作状态
uint8 key4_state_last = 0;                                                          // 上一次按键动作状态
uint8 key5_state_last = 0;                                                          // 上一次按键动作状态
uint8 go_state_last = 0;                                                          // 上一次按键动作状态

uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;
uint8 key5_flag;
uint8 go_flag;


/*本菜单代码思路:首先定义一个结构体包含(索引，上翻，下翻，确定，显示内容(该显示内容是一个函数指针void (*current_operation)();)
  然后用该结构体类型定义一个数组，该数组为结构体数组，每个元素都是结构体，可以访问结构体成员，(直观来看是将数组的值赋给成员)，实现上翻下翻和确认
  而操作函数的实现来源于结构体成员(函数指针)，该指针最后会赋值给一个指针变量，经过一系列变换后，最终实现函数操作
*/

void (*current_operation_index)(void);       //显示函数索引指针(当前操作索引)

uint8  func_index = 0; //初始显示欢迎界面
uint8  last_index = 127; //last初始为无效值

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蜂鸣器初始化
// 参数说明     无
// 返回参数     void
// 备注         蜂鸣器引脚配置为推挽输出，默认低电平(不响)
//-------------------------------------------------------------------------------------------------------------------
void Buzzer_init(void)//蜂鸣器初始化
{
    gpio_init(BUZZER_PIN, GPO, 0, GPO_PUSH_PULL);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蜂鸣器鸣响指定时长
// 参数说明     time2    鸣响时长(ms)
// 返回参数     void
// 备注         用于按键反馈与开机自检
//-------------------------------------------------------------------------------------------------------------------
void Buzzer_check(uint16 time2)//蜂鸣器自检函数
{
    gpio_set_level(BUZZER_PIN,1);
    system_delay_ms(time2);
    gpio_set_level(BUZZER_PIN,0);
}

key_table table_dispaly[]=                 //结构体数组
{
//{索引，向上，向下，确认，显示函数}
    //第0层
    {0,2,1,3,(*fun_A)},                     
    {1,0,2,4,(*fun_B)},
    {2,1,0,5,(*fun_C)},
	
    {3,3,3,0,(*fun_PID)},
    {4,4,4,1,(*fun_Offset)},
    {5,5,5,2,(*fun_Flash)},


};

///*********第0层***********/
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主菜单第1项显示(箭头指向 PID)
// 参数说明     无
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void fun_A(void)
{
	ips114_show_string(0,  16*1, "->");
    ips114_show_string(20, 16*1, "PID");                 ips114_show_string(180, 110, "Page_1");
    ips114_show_string(20, 16*2, "Offset");
    ips114_show_string(20, 16*3, "Flash");


}

////////////////////////////////////////////////////////////////////////////////////////////////////////第一层///////////////////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主菜单第2项显示(箭头指向 Offset)
// 参数说明     无
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void fun_B(void)
{

    ips114_show_string(0,  16*2, "->");
    ips114_show_string(20, 16*1, "PID");                 ips114_show_string(180, 110, "Page_1");
    ips114_show_string(20, 16*2, "Offset");
    ips114_show_string(20, 16*3, "Flash");


}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主菜单第3项显示(箭头指向 Flash)
// 参数说明     无
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void fun_C(void)
{
    ips114_show_string(0,  16*3, "->");
    ips114_show_string(20, 16*1, "PID");                 ips114_show_string(180, 110, "Page_1");
    ips114_show_string(20, 16*2, "Offset");
    ips114_show_string(20, 16*3, "Flash");



}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     公共参数面板绘制(14行参数)
// 参数说明     无
// 返回参数     void
// 备注         fun_PID 各页面共用，每圈循环画一次
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------
// 公共参数面板：14 行参数显示（fun_PID 各页面共用，每圈循环画一次）
// 重构前这段代码在 14 个页面里重复了 14 遍，现提取为独立函数
//-------------------------------------------------------------------
static void fun_pid_draw_panel(void)
{
    ips114_show_string(20,  16*0, "spd");    ips114_show_int16(16*4, 16*0, car_speed);
    ips114_show_string(20,  16*1, "f_d");    ips114_show_int16(16*4, 16*1, fly_dis);
    ips114_show_string(20,  16*2, "f_t");    ips114_show_int16(16*4, 16*2, fly_tim);
    ips114_show_string(20,  16*3, "r_d_d");  ips114_show_int16(16*4, 16*3, in_ring_dis_d);
    ips114_show_string(20,  16*4, "r_d_x");  ips114_show_int16(16*4, 16*4, in_ring_dis_x);
    ips114_show_string(20,  16*5, "off_d");  ips114_show_float(16*4, 16*5, r_offset_d, 2, 2);
    ips114_show_string(20,  16*6, "off_x");  ips114_show_float(16*4, 16*6, r_offset_x, 2, 2);
    ips114_show_string(20,  16*7, "r_th");   ips114_show_int16(16*4, 16*7, in_ring_threshold);
    ips114_show_string(16*10,16*0, "out_r"); ips114_show_int16(16*12, 16*0, out_ring_tim);
    ips114_show_string(16*10,16*1, "B");     ips114_show_float(16*12, 16*1, Kb,2,2);
    ips114_show_string(16*10,16*2, "kp");    ips114_show_int16(16*12, 16*2, angle_kp);
    ips114_show_string(16*10,16*3, "kd");    ips114_show_int16(16*12, 16*3, angle_kd);
    ips114_show_string(16*10,16*4, "kp_r");  ips114_show_int16(16*12, 16*4, angle_rg1_kp);
    ips114_show_string(16*10,16*5, "kd_r");  ips114_show_int16(16*12, 16*5, angle_rg1_kd);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID 参数调节菜单
// 参数说明     无
// 返回参数     void
// 备注         翻页: key2=上翻 key1=下翻(1~14循环)，换页清屏，key3退出；调节: key5=+ key4=-
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------
// fun_PID：参数调节菜单（重构版）
// 翻页：key2=上翻  key1=下翻（1~14 循环），换页清屏；退出：key3
// 调节：key5=+  key4=-
// 箭头：1~8 页指左列(0, 16*(i-1))，9~14 页指右列(134, 16*(14-i))
//-------------------------------------------------------------------
void fun_PID(void)
{
    uint8 i = 1;
    while(key3_flag == 0)
    {
        if(key2_flag == 1)
        {
            i ++;
            if(i > 14)i = 1;
            key2_clear();
            ips114_clear(RGB565_WHITE);
        }
        else if(key1_flag == 1)
        {
            i --;
            if(i == 0 || i >= 14)i = 14;
            key1_clear();
            ips114_clear(RGB565_WHITE);
        }

        // 公共参数面板（每圈画一次，取代原 14 页重复代码）
        fun_pid_draw_panel();

        // 箭头
        if(i <= 8)
            ips114_show_string(0,   16*(i-1),  "->");
        else
            ips114_show_string(134, 16*(14-i), "->");

        // 参数调节
        switch(i)
        {
            case 1:
                if(key5_flag==1){key5_clear(); car_speed += 10;}
                if(key4_flag==1){key4_clear(); car_speed -= 10;}
                break;
            case 2:
                if(key5_flag==1){key5_clear(); fly_dis += 100;}
                if(key4_flag==1){key4_clear(); fly_dis -= 100;}
                break;
            case 3:
                if(key5_flag==1){key5_clear(); fly_tim += 10;}
                if(key4_flag==1){key4_clear(); fly_tim -= 10;}
                break;
            case 4:
                if(key5_flag==1){key5_clear(); in_ring_dis_d += 100;}
                if(key4_flag==1){key4_clear(); in_ring_dis_d -= 100;}
                break;
            case 5:
                if(key5_flag==1){key5_clear(); in_ring_dis_x += 100;}
                if(key4_flag==1){key4_clear(); in_ring_dis_x -= 100;}
                break;
            case 6:
                if(key5_flag==1){key5_clear(); r_offset_d += 2;}
                if(key4_flag==1){key4_clear(); r_offset_d -= 2;}
                break;
            case 7:
                if(key5_flag==1){key5_clear(); r_offset_x += 2;}
                if(key4_flag==1){key4_clear(); r_offset_x -= 2;}
                break;
            case 8:
                if(key5_flag==1){key5_clear(); in_ring_threshold += 100;}
                if(key4_flag==1){key4_clear(); in_ring_threshold -= 100;}
                break;
            case 9:
                if(key5_flag==1){key5_clear(); angle_rg1_kd += 2;}
                if(key4_flag==1){key4_clear(); angle_rg1_kd -= 2;}
                break;
            case 10:
                if(key5_flag==1){key5_clear(); angle_rg1_kp += 2;}
                if(key4_flag==1){key4_clear(); angle_rg1_kp -= 2;}
                break;
            case 11:
                if(key5_flag==1){key5_clear(); angle_kd += 2;}
                if(key4_flag==1){key4_clear(); angle_kd -= 2;}
                break;
            case 12:
                if(key5_flag==1){key5_clear(); angle_kp += 2;}
                if(key4_flag==1){key4_clear(); angle_kp -= 2;}
                break;
            case 13:
                if(key5_flag==1){key5_clear(); Kb += 0.05;}
                if(key4_flag==1){key4_clear(); Kb -= 0.05;}
                break;
            case 14:
                if(key5_flag==1){key5_clear(); out_ring_tim += 20;}
                if(key4_flag==1){key4_clear(); out_ring_tim -= 20;}
                break;
            default:
                break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏移量观察界面
// 参数说明     无
// 返回参数     void
// 备注         实时显示四路电感、归一化值、误差、环状态、积分角度、轮速等调试信息；
//             按 key3 退出
//-------------------------------------------------------------------------------------------------------------------
void fun_Offset(void)
{
		while(key3_flag == 0)
		{
			ips114_show_string(0,0*16,"L1");				ips114_show_int16(40,0*16,adc_x_l);
			ips114_show_string(0,1*16,"L2");			  ips114_show_int16(40,1*16,adc_y_l);
			ips114_show_string(0,2*16,"R2");				ips114_show_int16(40,2*16,adc_y_r);
			ips114_show_string(0,3*16,"R1");				ips114_show_int16(40,3*16,adc_x_r);
			
			ips114_show_string(0,4*16,"an_Z");		  ips114_show_int16(40,4*16,angle_integral_Z);
			ips114_show_string(0,5*16,"an_Y");		  ips114_show_int16(40,5*16,angle_integral_Y);
			ips114_show_string(0,6*16,"dis");				ips114_show_int16(40,6*16,motor_distance);
			ips114_show_string(0,7*16,"fly");				ips114_show_int16(40,7*16,flyboard_flag);
			
			ips114_show_string(120,0*16,"LX");			ips114_show_int16(160,0*16,LX);
			ips114_show_string(120,1*16,"LY");			ips114_show_int16(160,1*16,LY);
			ips114_show_string(120,2*16,"RY");			ips114_show_int16(160,2*16,RY);
			ips114_show_string(120,3*16,"RX");			ips114_show_int16(160,3*16,RX);
			
			ips114_show_string(120,4*16,"ERR");   	ips114_show_int16(160,4*16,ERR);
			ips114_show_string(120,5*16,"ring");   	ips114_show_int16(160,5*16,ring_flag);
            
      ips114_show_int16(120,6*16,IMU_Data.gyro_z);   	ips114_show_int16(160,6*16,IMU_Data.gyro_y);
			ips114_show_int16(120,7*16,encoder_data_dir_L); ips114_show_int16(160,7*16,encoder_data_dir_R);
		}

	

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     参数存取界面(Save/Load/Reset)
// 参数说明     无
// 返回参数     void
// 备注         按 key5 执行当前项：保存参数到 EEPROM / 从 EEPROM 加载 / 恢复出厂默认；
//             按 key3 退出
//-------------------------------------------------------------------------------------------------------------------
void fun_Flash(void)
{

    uint8 i = 1;
    while(key3_flag == 0){
        if(key2_flag == 1){
               i ++;
               if(i > 3)i = 1;
               key2_clear();
               ips114_clear(RGB565_WHITE);
         }
        else if(key1_flag == 1){
                i --;
                if(i == 0 || i >= 3)i = 3;
                key1_clear();
                ips114_clear(RGB565_WHITE);
        }
        if(i == 1)
        {
            ips114_show_string(0,  16*0, "->");
            ips114_show_string(20, 16*0, "Save");
            ips114_show_string(20, 16*1, "Load");
            ips114_show_string(20, 16*2, "Reset");
            if(key5_flag == 1)
            {
                key5_clear();
                sys_param_save();//保存现在参数
                ips114_show_string(60, 16*5, "Save succeed");
                system_delay_ms(500);
            }
        }
        else if(i == 2)
        {
            ips114_show_string(0,  16*1, "->");
            ips114_show_string(20, 16*0, "Save");
            ips114_show_string(20, 16*1, "Load");
            ips114_show_string(20, 16*2, "Reset");
            if(key5_flag == 1)
            {
                key5_clear();
                sys_param_load();//将内存参数取出
                ips114_show_string(60, 16*5, "Load succeed");
                system_delay_ms(500);

            }
        }
        else if(i == 3)
        {
            ips114_show_string(0,  16*2, "->");
            ips114_show_string(20, 16*0, "Save");
            ips114_show_string(20, 16*1, "Load");
            ips114_show_string(20, 16*2, "Reset");
            if(key5_flag == 1)
            {
                key5_clear();
                sys_param_reset();//恢复默认值
                ips114_show_string(60, 16*5, "Reset succeed");
                system_delay_ms(500);

            }
        }
    }
    
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键初始化
// 参数说明     无
// 返回参数     void
// 备注         6 个按键均配置为输入上拉(KEY1~KEY5 + 发车键 gogogo)
//-------------------------------------------------------------------------------------------------------------------
void Key_init(void)//按键与LED初始化
{
       gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY1 输入 默认高电平 上拉输入
       gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY2 输入 默认高电平 上拉输入
       gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY3 输入 默认高电平 上拉输入
       gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入
			 gpio_init(KEY5, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入
			 gpio_init(gogogo, GPI, GPIO_HIGH, GPI_PULL_UP);        // 初始化 gogogog 输入 默认高电平 上拉输入
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键扫描(消抖/沿检测)
// 参数说明     无
// 返回参数     void
// 备注         由 2ms 定时器中断调用(发车前)；按键松开沿置位对应 flag，
//             松开沿触发可避免长按重复触发
//-------------------------------------------------------------------------------------------------------------------
void key_scan(void)//按键扫描
{

        //保存按键状态
        key1_state_last = key1_state;
        key2_state_last = key2_state;
        key3_state_last = key3_state;
        key4_state_last = key4_state;
				key5_state_last = key5_state;
				go_state_last   = go_state;

        //读取当前按键状态
        key1_state = gpio_get_level(KEY1);
        key2_state = gpio_get_level(KEY2);
        key3_state = gpio_get_level(KEY3);
        key4_state = gpio_get_level(KEY4);
				key5_state = gpio_get_level(KEY5);
				go_state   = gpio_get_level(gogogo);

        //检测到按键按下之后  并放开置位标志位
        if(key1_state && !key1_state_last)   {key1_flag = 1;}
        if(key2_state && !key2_state_last)   {key2_flag = 1;}
        if(key3_state && !key3_state_last)   {key3_flag = 1;}
        if(key4_state && !key4_state_last)   {key4_flag = 1;}
				if(key5_state && !key5_state_last)   {key5_flag = 1;}
				if(go_state && !go_state_last)   {go_flag = 1;}
        //标志位置位之后，可以使用标志位执行自己想要做的事件
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除 key1 标志并给出蜂鸣反馈
// 参数说明     无
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void key1_clear(void)
{
  key1_flag=0;
  Buzzer_check(50);

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除 key2 标志并给出蜂鸣反馈
// 参数说明     无
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void key2_clear(void)
{
  key2_flag=0;
  Buzzer_check(50);

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除 key3 标志并给出蜂鸣反馈
// 参数说明     无
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void key3_clear(void)
{
  key3_flag=0;
  Buzzer_check(50);

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除 key4 标志并给出蜂鸣反馈
// 参数说明     无
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void key4_clear(void)
{
  key4_flag=0;
  Buzzer_check(50);

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除 key5 标志并给出蜂鸣反馈
// 参数说明     无
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void key5_clear(void)
{
  key5_flag=0;
  Buzzer_check(50);

}

//===================================================菜单初始化===================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单初始化
// 参数说明     无
// 返回参数     void
// 备注         初始化 IPS114 屏幕、按键，蜂鸣器自检一声
//-------------------------------------------------------------------------------------------------------------------
void menu_init(void)
{
    ips114_init();							                    //屏幕初始化
    Key_init();                                                 //按键初始化

    Buzzer_check(50);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主菜单轮询处理
// 参数说明     无
// 返回参数     void
// 备注         主循环中调用；按 key1/key2 上下翻页、key3 确认进入子菜单；
//             菜单索引变化时执行对应显示函数
//-------------------------------------------------------------------------------------------------------------------
void Menu(void)//菜单函数
{



                if(key1_flag)
                {

                    func_index = table_dispaly[func_index].up;    //向上翻
                    key1_clear();
                }
                if(key2_flag)
                {

                    func_index = table_dispaly[func_index].down;    //向下翻
                     key2_clear();

                }
                if(key3_flag)
                {

                    func_index = table_dispaly[func_index].enter;    //确认
                    key3_clear();

                }

            if (func_index != last_index)
            {
                current_operation_index = table_dispaly[func_index].current_operation;
                ips114_clear(RGB565_WHITE);
                (*current_operation_index)();//执行当前操作函数
                last_index = func_index;

            }
  }
