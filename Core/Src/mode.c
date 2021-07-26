/*-------------------------------------------------------------------------------------------

 Ӳ��ƽ̨:
 			������: STM32F103VET6 64K RAM 512K ROM
			������: LMD18200T 
		    ��Դ:   DC +12V

 ���ƽ̨:
 			��������: RealView MDK-ARM uVision4.10
			C������ : ARMCC
			ASM������:ARMASM
			������:   ARMLINK
			�ײ�����: ����������������       


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
 				ȫ�ֱ���				����speed��angle��������
------------------------------------------*/
M1TypeDef M1_spe;
M2TypeDef M2_spe;
M1TypeDef M1_angle;
M2TypeDef M2_angle;

extern PIDTypdDef M1PID_spe;
extern PIDTypdDef M2PID_spe;
extern PIDTypdDef M1PID_angle;
extern PIDTypdDef M2PID_angle;

extern int Start;               //һ������
extern int point_choose[4];     //ģʽ��ѡ���
extern double target_pos[9][2]; //����ľŸ�Ŀ����λ��

/*------------------------------------------
 ��������:�����������λ
 ����˵��:ǿ�Ƹ�λ			
------------------------------------------*/
void MCU_Reset(void)
{
    __set_FAULTMASK(1); // �ر������ж�
    NVIC_SystemReset(); // ��λ
}
/*------------------------------------------
 ��������:��ʼ��M1�ṹ�����
 ����˵��:			
------------------------------------------*/
void M1TypeDef_Init(void)
{
    M1_spe.CurPos = 0.0;
    M1_spe.PrevPos = 0.0;
    M1_spe.CurAcc = 0.0;
    M1_spe.PrevSpeed = 0.0;
    M1_spe.Offset = 0.0;   //����ƫ����
    M1_spe.CurSpeed = 0.0; //��ǰ�ٶ�ʸ��
    M1_spe.PWM = 0;        //PWM
    M1_angle.CurPos = 0.0;
    M1_angle.PrevPos = 0.0;
    M1_angle.CurAcc = 0.0;
    M1_angle.PrevSpeed = 0.0;
    M1_angle.Offset = 0.0;   //����ƫ����
    M1_angle.CurSpeed = 0.0; //��ǰ�ٶ�ʸ��
    M1_angle.PWM = 0;        //PWM
}
/*------------------------------------------
 ��������:��ʼ��M2�ṹ�����
 ����˵��:			
------------------------------------------*/
void M2TypeDef_Init(void)
{
    M2_spe.CurPos = 0.0;
    M2_spe.PrevPos = 0.0;
    M2_spe.CurAcc = 0.0;
    M2_spe.PrevSpeed = 0.0;
    M2_spe.Offset = 0.0;   //����ƫ����
    M2_spe.CurSpeed = 0.0; //��ǰ�ٶ�ʸ��
    M2_spe.PWM = 0;        //PWM
    M2_angle.CurPos = 0.0;
    M2_angle.PrevPos = 0.0;
    M2_angle.CurAcc = 0.0;
    M2_angle.PrevSpeed = 0.0;
    M2_angle.Offset = 0.0;   //����ƫ����
    M2_angle.CurSpeed = 0.0; //��ǰ�ٶ�ʸ��
    M2_angle.PWM = 0;        //PWM
}
/*------------------------------------------
 ��ģʽ����:

------------------------------------------*/
void Mode_1(void)
{
    float set_x; //Ŀ�������(cm)
    float set_y;
    if (Start == 1) //����
    {
        set_x = 12.5;
        set_y = 12.5;
    }
    else //ֹͣ
    {
        set_x = 22.5;
        set_y = 22.5;
    }

    /////////////////////////////////////////////λ�û�PID///////////////////////////////
    M1PID_spe.SetPoint = set_x;
    M1PID_spe.Proportion = 50;
    M1PID_spe.Integral = 0;
    M1PID_spe.Derivative = 800;

    M2PID_spe.SetPoint = set_y;
    M2PID_spe.Proportion = 50;
    M2PID_spe.Integral = 0;
    M2PID_spe.Derivative = 800;

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X����PID����
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y����PID����

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //����޷�
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////�ٶȻ�///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X����PID����
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y����PID����

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //����޷�
    if (M1_angle.PWM < -POWER_MAX_spe)
        M1_angle.PWM = -POWER_MAX_angle;
    if (M2_angle.PWM > POWER_MAX_spe)
        M2_angle.PWM = POWER_MAX_angle;
    if (M2_angle.PWM < -POWER_MAX_spe)
        M2_angle.PWM = -POWER_MAX_angle;

    MotorMove(M1_angle.PWM + 1550, M2_angle.PWM + 1450);
}

u32 cout2 = 0; //��ʱ����

void Mode_2(void)
{
    float set_x; //Ŀ�������(cm)
    float set_y;
    if (Start == 1)
    {
        if (cout2 < 100) //��������1ͣ2s
        {
            set_x = 12.5; //Ŀ�������(cm)
            set_y = 12.5;
            ++cout2;
        }
        else //��ȥ����5
        {
            set_x = 25; //Ŀ�������(cm)
            set_y = 25;
        }
    }
    else
    {
        set_x = 0; //Ŀ�������(cm)
        set_y = 0;
        cout2 = 0;
    }
    /////////////////////////////////////////////λ�û�PID///////////////////////////////

    M1PID_spe.SetPoint = set_x;
    M1PID_spe.Proportion = 50;
    M1PID_spe.Integral = 0;
    M1PID_spe.Derivative = 800;

    M2PID_spe.SetPoint = set_y;
    M2PID_spe.Proportion = 50;
    M2PID_spe.Integral = 0;
    M2PID_spe.Derivative = 800;

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X����PID����
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y����PID����

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //����޷�
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////�ٶȻ�///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X����PID����
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y����PID����

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //����޷�
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
    float set_x; //Ŀ�������(cm)
    float set_y;
    if (Start == 1)
    {
        /////////////////////////////////////////////λ�û�PID///////////////////////////////
        if (cout3 < 100) //��������1ͣ2s
        {
            set_x = 12.5; //Ŀ�������(cm)
            set_y = 12.5;
            ++cout3;
        }
        else if (cout3 < 300) //��ȥ����4ͣ4s
        {
            set_x = 12.5; //Ŀ�������(cm)
            set_y = 25;
            ++cout3;
        }
        else
        {
            set_x = 25; //Ŀ�������(cm)
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

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X����PID����
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y����PID����

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //����޷�
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////�ٶȻ�///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X����PID����
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y����PID����

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //����޷�
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
    float set_x; //Ŀ�������(cm)
    float set_y;
    if (Start == 1)
    {
        /////////////////////////////////////////////λ�û�PID///////////////////////////////
        if (cout4 < 100) //��������1ͣ2s
        {
            set_x = 12.5; //Ŀ�������(cm)
            set_y = 12.5;
            ++cout4;
        }
        else //��ȥ����9
        {
            set_x = 37.5; //Ŀ�������(cm)
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

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X����PID����
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y����PID����

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //����޷�
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////�ٶȻ�///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X����PID����
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y����PID����

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //����޷�
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
    float set_x; //Ŀ�������(cm)
    float set_y;
    if (Start == 1)
    {
        /////////////////////////////////////////////λ�û�PID///////////////////////////////

        if (cout5 < 100) //��������1ͣ2s
        {
            set_x = 12.5; //Ŀ�������(cm)
            set_y = 12.5;
            ++cout5;
        }
        else if (cout5 < 300) //��ȥ����2ͣ4s
        {
            set_x = 25; //Ŀ�������(cm)
            set_y = 12.5;
            ++cout5;
        }
        else if (cout5 < 500) //��ȥ����6ͣ4s
        {
            set_x = 37.5; //Ŀ�������(cm)
            set_y = 25;
            ++cout5;
        }
        else //��ȥ����9
        {
            set_x = 37.5; //Ŀ�������(cm)
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

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X����PID����
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y����PID����

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //����޷�
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////�ٶȻ�///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X����PID����
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y����PID����

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //����޷�
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
    float set_x; //Ŀ�������(cm)
    float set_y;
    if (Start == 1)
    {
        /////////////////////////////////////////////λ�û�PID///////////////////////////////
        if (cout6 < 100) //��������1ͣ2s
        {
            set_x = target_pos[point_choose[0]][0]; //Ŀ�������(cm)
            set_y = target_pos[point_choose[0]][1];
            ++cout6;
        }
        else if (cout6 < 300) //��ȥ����2ͣ4s
        {
            set_x = target_pos[point_choose[1]][0]; //Ŀ�������(cm)
            set_y = target_pos[point_choose[1]][1];
            ++cout6;
        }
        else if (cout6 < 500) //��ȥ����6ͣ4s
        {
            set_x = target_pos[point_choose[2]][0]; //Ŀ�������(cm)
            set_y = target_pos[point_choose[2]][1];
            ++cout6;
        }
        else //��ȥ����9
        {
            set_x = target_pos[point_choose[3]][0]; //Ŀ�������(cm)
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

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X����PID����
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y����PID����

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //����޷�
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////�ٶȻ�///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X����PID����
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y����PID����

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //����޷�
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
    const float priod = 4000.0;      ///����(����)
    static uint32_t MoveTimeCnt = 0; //����
    float R = 10;                    //��Բ�뾶
    float Normalization = 0.0;
    float Omega = 0.0;

    if (Start == 1)
    {
        if (cout7 < 100) //��������4��2s
        {
            set_x = 12.5;
            set_y = 22.5;
        }
        else if (cout7 < 600) //������5��Χ����3Ȧ����
        {
            MoveTimeCnt += 20;                          //ÿ20ms����1��
            Normalization = (float)MoveTimeCnt / priod; //�Ե������ڹ�һ��
            Omega = 2.0 * 3.14159 * Normalization;      //��2�н��й�һ������
            set_x = R * sin(Omega);                     //�������ǰ�ڽ�
            set_y = R * sin(Omega + 3.141592 / 2.0);    //�������ǰ�ڽ�
        }
        else //ȥ����9
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

    M1_spe.PWM = PID_M1_spe_PosLocCalc(M1_spe.CurPos); //X����PID����
    M2_spe.PWM = PID_M2_spe_PosLocCalc(M2_spe.CurPos); //Y����PID����

    if (M1_spe.PWM > POWER_MAX_spe)
        M1_spe.PWM = POWER_MAX_spe; //����޷�
    if (M1_spe.PWM < -POWER_MAX_spe)
        M1_spe.PWM = -POWER_MAX_spe;
    if (M2_spe.PWM > POWER_MAX_spe)
        M2_spe.PWM = POWER_MAX_spe;
    if (M2_spe.PWM < -POWER_MAX_spe)
        M2_spe.PWM = -POWER_MAX_spe;

    //////////////////////////////////////////////�ٶȻ�///////////////////////////////////////////
    M1PID_angle.SetPoint = M1_spe.PWM;
    M1PID_angle.Proportion = 50;
    M1PID_angle.Integral = 0;
    M1PID_angle.Derivative = 800;

    M2PID_angle.SetPoint = M2_spe.PWM;
    M2PID_angle.Proportion = 50;
    M2PID_angle.Integral = 0;
    M2PID_angle.Derivative = 800;

    M1_angle.PWM = PID_M1_angle_PosLocCalc(M1_angle.CurPos); //X����PID����
    M2_angle.PWM = PID_M2_angle_PosLocCalc(M2_angle.CurPos); //Y����PID����

    if (M1_angle.PWM > POWER_MAX_spe)
        M1_angle.PWM = POWER_MAX_angle; //����޷�
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
    TIM_SetCompare1(TIM3, pwm1); //����PWM
    TIM_SetCompare2(TIM3, pwm2);
}
