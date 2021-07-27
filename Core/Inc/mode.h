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
#include "motor.h"
#include "main.h"
#include "cJSON.h"

void Mode_1(void);

void Mode_2(void);

void Mode_3(void);

void Mode_4(void);

void Mode_5(void);

void Mode_6(void);

void Mode_7(void);

void Mode_8(void);
