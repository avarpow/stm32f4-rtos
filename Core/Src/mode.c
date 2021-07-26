/*-------------------------------------------------------------------------------------------

 硬件平台:
 			主控器: STM32F103VET6 64K RAM 512K ROM
			驱动器: LMD18200T 
		    电源:   DC +12V

 软件平台:
 			开发环境: RealView MDK-ARM uVision4.10
			C编译器 : ARMCC
			ASM编译器:ARMASM
			连接器:   ARMLINK
			底层驱动: 各个外设驱动程序       


-------------------------------------------------------------------------------------------*/
#include "motor_control.h"
#include "motor_pwm.h"
#include "motor_pid.h"
#include "stdlib.h"
#include "stdio.h"
#include "delay.h"
#include "math.h"
#include "Upload.h"
/*------------------------------------------
 				全局变量				包括speed和angle两个变量
------------------------------------------*/
M1TypeDef M1_spe;
M2TypeDef M2_spe;
M1TypeDef M1_angle;
M2TypeDef M2_angle;

extern PIDTypdDef M1PID_spe;
extern PIDTypdDef M2PID_spe;
extern PIDTypdDef M1PID_angle;
extern PIDTypdDef M2PID_angle;

extern int Start;               //一键启动
extern int point_choose[4];     //模式六选择点
extern double target_pos[9][2]; //储存的九个目标点的位置

/*------------------------------------------
 函数功能:控制器软件复位
 函数说明:强制复位			
------------------------------------------*/
void MCU_Reset(void)
{
    __set_FAULTMASK(1); // 关闭所有中断
    NVIC_SystemReset(); // 复位
}
/*------------------------------------------
 函数功能:初始化M1结构体参数
 函数说明:			
------------------------------------------*/
void M1TypeDef_Init(void)
{
    M1_spe.CurPos = 0.0;
    M1_spe.PrevPos = 0.0;
    M1_spe.CurAcc = 0.0;
    M1_spe.PrevSpeed = 0.0;
    M1_spe.Offset = 0.0;   //允许偏差量
    M1_spe.CurSpeed = 0.0; //当前速度矢量
    M1_spe.PWM = 0;        //PWM
    M1_angle.CurPos = 0.0;
    M1_angle.PrevPos = 0.0;
    M1_angle.CurAcc = 0.0;
    M1_angle.PrevSpeed = 0.0;
    M1_angle.Offset = 0.0;   //允许偏差量
    M1_angle.CurSpeed = 0.0; //当前速度矢量
    M1_angle.PWM = 0;        //PWM
}
/*------------------------------------------
 函数功能:初始化M2结构体参数
 函数说明:			
------------------------------------------*/
void M2TypeDef_Init(void)
{
    M2_spe.CurPos = 0.0;
    M2_spe.PrevPos = 0.0;
    M2_spe.CurAcc = 0.0;
    M2_spe.PrevSpeed = 0.0;
    M2_spe.Offset = 0.0;   //允许偏差量
    M2_spe.CurSpeed = 0.0; //当前速度矢量
    M2_spe.PWM = 0;        //PWM
    M2_angle.CurPos = 0.0;
    M2_angle.PrevPos = 0.0;
    M2_angle.CurAcc = 0.0;
    M2_angle.PrevSpeed = 0.0;
    M2_angle.Offset = 0.0;   //允许偏差量
    M2_angle.CurSpeed = 0.0; //当前速度矢量
    M2_angle.PWM = 0;        //PWM
}
/*------------------------------------------
 各模式函数:

------------------------------------------*/
void Mode_1(void)
{
    float set_x; //目标点坐标(cm)
    float set_y;
    if (Start == 1) //启动
    {
        set_x = 12.5;
        set_y = 12.5;
    }
    else //停止
    {
        set_x = 22.5;
        set_y = 22.5;
    }

    /////////////////////////////////////////////位置环PID///////////////////////////////
    M1PID_spe.SetPoint = set_x;
    M1PID_spe.Proportion = 50;
    M1PID_spe.Integral = 0;
    M1PID_spe.Derivative = 800;

    M2PID_spe.SetPoint = set_y;
    M2PID_spe.Proportion = 50;
    M2PID_spe.Integral = 0;
    M2PID_spe.Derivative = 800;

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X方向PID计算
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y方向PID计算

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //输出限幅
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////速度环///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X方向PID计算
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y方向PID计算

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //输出限幅
    if (M1_angle.PWM < -POWER_MAX_spe)
        M1_angle.PWM = -POWER_MAX_angle;
    if (M2_angle.PWM > POWER_MAX_spe)
        M2_angle.PWM = POWER_MAX_angle;
    if (M2_angle.PWM < -POWER_MAX_spe)
        M2_angle.PWM = -POWER_MAX_angle;

    MotorMove(M1_angle.PWM + 1550, M2_angle.PWM + 1450);
}

u32 cout2 = 0; //计时变量

void Mode_2(void)
{
    float set_x; //目标点坐标(cm)
    float set_y;
    if (Start == 1)
    {
        if (cout2 < 100) //先在区域1停2s
        {
            set_x = 12.5; //目标点坐标(cm)
            set_y = 12.5;
            ++cout2;
        }
        else //再去区域5
        {
            set_x = 25; //目标点坐标(cm)
            set_y = 25;
        }
    }
    else
    {
        set_x = 0; //目标点坐标(cm)
        set_y = 0;
        cout2 = 0;
    }
    /////////////////////////////////////////////位置环PID///////////////////////////////

    M1PID_spe.SetPoint = set_x;
    M1PID_spe.Proportion = 50;
    M1PID_spe.Integral = 0;
    M1PID_spe.Derivative = 800;

    M2PID_spe.SetPoint = set_y;
    M2PID_spe.Proportion = 50;
    M2PID_spe.Integral = 0;
    M2PID_spe.Derivative = 800;

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X方向PID计算
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y方向PID计算

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //输出限幅
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////速度环///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X方向PID计算
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y方向PID计算

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //输出限幅
    if (M1_angle.PWM < -POWER_MAX_spe)
        M1_angle.PWM = -POWER_MAX_angle;
    if (M2_angle.PWM > POWER_MAX_spe)
        M2_angle.PWM = POWER_MAX_angle;
    if (M2_angle.PWM < -POWER_MAX_spe)
        M2_angle.PWM = -POWER_MAX_angle;

    MotorMove(M1_angle.PWM, M2_angle.PWM);
}

u32 cout3 = 0;
void Mode_3(void)
{
    float set_x; //目标点坐标(cm)
    float set_y;
    if (Start == 1)
    {
        /////////////////////////////////////////////位置环PID///////////////////////////////
        if (cout3 < 100) //先在区域1停2s
        {
            set_x = 12.5; //目标点坐标(cm)
            set_y = 12.5;
            ++cout3;
        }
        else if (cout3 < 300) //再去区域4停4s
        {
            set_x = 12.5; //目标点坐标(cm)
            set_y = 25;
            ++cout3;
        }
        else
        {
            set_x = 25; //目标点坐标(cm)
            set_y = 25;
        }
    }
    else
    {
        set_x = 22.5;
        set_y = 22.5;
        cout3 = 0;
    }

    M1PID_spe.SetPoint = set_x;
    M1PID_spe.Proportion = 50;
    M1PID_spe.Integral = 0;
    M1PID_spe.Derivative = 800;

    M2PID_spe.SetPoint = set_y;
    M2PID_spe.Proportion = 50;
    M2PID_spe.Integral = 0;
    M2PID_spe.Derivative = 800;

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X方向PID计算
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y方向PID计算

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //输出限幅
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////速度环///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X方向PID计算
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y方向PID计算

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //输出限幅
    if (M1_angle.PWM < -POWER_MAX_spe)
        M1_angle.PWM = -POWER_MAX_angle;
    if (M2_angle.PWM > POWER_MAX_spe)
        M2_angle.PWM = POWER_MAX_angle;
    if (M2_angle.PWM < -POWER_MAX_spe)
        M2_angle.PWM = -POWER_MAX_angle;

    MotorMove(M1_angle.PWM, M2_angle.PWM);
}

u32 cout4 = 0;
void Mode_4(void)
{
    float set_x; //目标点坐标(cm)
    float set_y;
    if (Start == 1)
    {
        /////////////////////////////////////////////位置环PID///////////////////////////////
        if (cout4 < 100) //先在区域1停2s
        {
            set_x = 12.5; //目标点坐标(cm)
            set_y = 12.5;
            ++cout4;
        }
        else //再去区域9
        {
            set_x = 37.5; //目标点坐标(cm)
            set_y = 37.5;
        }
    }
    else
    {
        set_x = 22.5;
        set_y = 22.5;
        cout4 = 0;
    }

    M1PID_spe.SetPoint = set_x;
    M1PID_spe.Proportion = 50;
    M1PID_spe.Integral = 0;
    M1PID_spe.Derivative = 800;

    M2PID_spe.SetPoint = set_y;
    M2PID_spe.Proportion = 50;
    M2PID_spe.Integral = 0;
    M2PID_spe.Derivative = 800;

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X方向PID计算
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y方向PID计算

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //输出限幅
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////速度环///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X方向PID计算
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y方向PID计算

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //输出限幅
    if (M1_angle.PWM < -POWER_MAX_spe)
        M1_angle.PWM = -POWER_MAX_angle;
    if (M2_angle.PWM > POWER_MAX_spe)
        M2_angle.PWM = POWER_MAX_angle;
    if (M2_angle.PWM < -POWER_MAX_spe)
        M2_angle.PWM = -POWER_MAX_angle;

    MotorMove(M1_angle.PWM, M2_angle.PWM);
}

u32 cout5 = 0;
void Mode_5(void)
{
    float set_x; //目标点坐标(cm)
    float set_y;
    if (Start == 1)
    {
        /////////////////////////////////////////////位置环PID///////////////////////////////

        if (cout5 < 100) //先在区域1停2s
        {
            set_x = 12.5; //目标点坐标(cm)
            set_y = 12.5;
            ++cout5;
        }
        else if (cout5 < 300) //再去区域2停4s
        {
            set_x = 25; //目标点坐标(cm)
            set_y = 12.5;
            ++cout5;
        }
        else if (cout5 < 500) //再去区域6停4s
        {
            set_x = 37.5; //目标点坐标(cm)
            set_y = 25;
            ++cout5;
        }
        else //再去区域9
        {
            set_x = 37.5; //目标点坐标(cm)
            set_y = 37.5;
        }
    }
    else
    {
        set_x = 22.5;
        set_y = 22.5;
        cout5 = 0;
    }

    M1PID_spe.SetPoint = set_x;
    M1PID_spe.Proportion = 50;
    M1PID_spe.Integral = 0;
    M1PID_spe.Derivative = 800;

    M2PID_spe.SetPoint = set_y;
    M2PID_spe.Proportion = 50;
    M2PID_spe.Integral = 0;
    M2PID_spe.Derivative = 800;

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X方向PID计算
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y方向PID计算

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //输出限幅
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////速度环///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X方向PID计算
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y方向PID计算

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //输出限幅
    if (M1_angle.PWM < -POWER_MAX_spe)
        M1_angle.PWM = -POWER_MAX_angle;
    if (M2_angle.PWM > POWER_MAX_spe)
        M2_angle.PWM = POWER_MAX_angle;
    if (M2_angle.PWM < -POWER_MAX_spe)
        M2_angle.PWM = -POWER_MAX_angle;

    MotorMove(M1_angle.PWM, M2_angle.PWM);
}

u32 cout6 = 0;
void Mode_6(void)
{
    float set_x; //目标点坐标(cm)
    float set_y;
    if (Start == 1)
    {
        /////////////////////////////////////////////位置环PID///////////////////////////////
        if (cout6 < 100) //先在区域1停2s
        {
            set_x = target_pos[point_choose[0]][0]; //目标点坐标(cm)
            set_y = target_pos[point_choose[0]][1];
            ++cout6;
        }
        else if (cout6 < 300) //再去区域2停4s
        {
            set_x = target_pos[point_choose[1]][0]; //目标点坐标(cm)
            set_y = target_pos[point_choose[1]][1];
            ++cout6;
        }
        else if (cout6 < 500) //再去区域6停4s
        {
            set_x = target_pos[point_choose[2]][0]; //目标点坐标(cm)
            set_y = target_pos[point_choose[2]][1];
            ++cout6;
        }
        else //再去区域9
        {
            set_x = target_pos[point_choose[3]][0]; //目标点坐标(cm)
            set_y = target_pos[point_choose[3]][1];
        }
    }
    else
    {
        set_x = 22.5;
        set_y = 22.5;
        cout6 = 0;
    }

    M1PID_spe.SetPoint = set_x;
    M1PID_spe.Proportion = 50;
    M1PID_spe.Integral = 0;
    M1PID_spe.Derivative = 800;

    M2PID_spe.SetPoint = set_y;
    M2PID_spe.Proportion = 50;
    M2PID_spe.Integral = 0;
    M2PID_spe.Derivative = 800;

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X方向PID计算
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y方向PID计算

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //输出限幅
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////速度环///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X方向PID计算
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y方向PID计算

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //输出限幅
    if (M1_angle.PWM < -POWER_MAX_spe)
        M1_angle.PWM = -POWER_MAX_angle;
    if (M2_angle.PWM > POWER_MAX_spe)
        M2_angle.PWM = POWER_MAX_angle;
    if (M2_angle.PWM < -POWER_MAX_spe)
        M2_angle.PWM = -POWER_MAX_angle;

    MotorMove(M1_angle.PWM, M2_angle.PWM);
}

u32 cout7 = 0;
void Mode_7(void)
{
    float set_x = 0;
    float set_y = 0;
    const float priod = 4000.0;      ///周期(毫秒)
    static uint32_t MoveTimeCnt = 0; //计数
    float R = 10;                    //画圆半径
    float Normalization = 0.0;
    float Omega = 0.0;

    if (Start == 1)
    {
        if (cout7 < 100) //现在区域4等2s
        {
            set_x = 12.5;
            set_y = 22.5;
        }
        else if (cout7 < 600) //在区域5周围环绕3圈以上
        {
            MoveTimeCnt += 20;                          //每20ms运算1次
            Normalization = (float)MoveTimeCnt / priod; //对单摆周期归一化
            Omega = 2.0 * 3.14159 * Normalization;      //对2π进行归一化处理
            set_x = R * sin(Omega);                     //计算出当前摆角
            set_y = R * sin(Omega + 3.141592 / 2.0);    //计算出当前摆角
        }
        else //去区域9
        {
            set_x = 52.5;
            set_y = 52.5;
        }
    }
    else
    {
        set_x = 22.5;
        set_y = 22.5;
        cout7 = 0;
    }

    M1PID_spe.SetPoint = set_x;
    M1PID_spe.Proportion = 50;
    M1PID_spe.Integral = 0;
    M1PID_spe.Derivative = 800;

    M2PID_spe.SetPoint = set_y;
    M2PID_spe.Proportion = 50;
    M2PID_spe.Integral = 0;
    M2PID_spe.Derivative = 800;

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X方向PID计算
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y方向PID计算

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //输出限幅
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////速度环///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X方向PID计算
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y方向PID计算

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //输出限幅
    if (M1_angle.PWM < -POWER_MAX_spe)
        M1_angle.PWM = -POWER_MAX_angle;
    if (M2_angle.PWM > POWER_MAX_spe)
        M2_angle.PWM = POWER_MAX_angle;
    if (M2_angle.PWM < -POWER_MAX_spe)
        M2_angle.PWM = -POWER_MAX_angle;

    MotorMove(M1_angle.PWM, M2_angle.PWM);
}

void Mode_8(void)
{
}

void MotorMove(int32_t pwm1, int32_t pwm2)
{
    TIM_SetCompare1(TIM3, pwm1); //设置PWM
    TIM_SetCompare2(TIM3, pwm2);
}
