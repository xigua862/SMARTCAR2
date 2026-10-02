/*****************************************************************************
* 文件名称: Menu.h
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     菜单模块头文件：按键/蜂鸣器引脚宏、菜单结构体与函数声明
*****************************************************************************/

#ifndef MENU_H
#define MENU_H

#define BUZZER_PIN  			(IO_P67)

#define KEY1                    (IO_P72)//上
#define KEY2                    (IO_P50)//下
#define KEY3                    (IO_P33)//按下
#define KEY4                    (IO_P51)//左
#define KEY5                    (IO_P36)//右
#define gogogo              		(IO_P76)//发车

extern uint8  func_index; //初始显示欢迎界面
extern uint8  last_index; //last初始为无效值
//全局变量

extern uint8 key1_flag;
extern uint8 key2_flag;
extern uint8 key3_flag;
extern uint8 key4_flag;
extern uint8 key5_flag;
extern uint8 go_flag;

//结构体声明
typedef struct
{
    int current;
    int up;//向上翻索引号
    int down;//向下翻索引号
    int enter;//确认索引号
    void (*current_operation)();//当前页面的索引号要执行的显示函数，这是一个函数指针
}key_table;


void Buzzer_init(void);           // 蜂鸣器初始化
void Buzzer_check(uint16 time2);  // 蜂鸣器鸣响指定时长(ms)//蜂鸣器自检函数

void Key_init(void);              // 按键初始化
void key_scan(void);              // 按键扫描(2ms中断调用)
void key1_clear(void);            // 清除key1标志并蜂鸣反馈
void key2_clear(void);            // 清除key2标志并蜂鸣反馈
void key3_clear(void);            // 清除key3标志并蜂鸣反馈
void key4_clear(void);            // 清除key4标志并蜂鸣反馈
void key5_clear(void);            // 清除key5标志并蜂鸣反馈

void menu_init(void);             // 菜单初始化

//函数声明
void  Menu(void);//菜单函数         // 主菜单轮询处理


void fun_A(void);                 // 主菜单第1项显示
void fun_B(void);                 // 主菜单第2项显示
void fun_C(void);                 // 主菜单第3项显示

void fun_PID(void);               // PID参数调节界面
void fun_Offset(void);            // 偏移量观察界面
void fun_Flash(void);             // 参数存取界面(Save/Load/Reset)
//void fun_f1();

#endif