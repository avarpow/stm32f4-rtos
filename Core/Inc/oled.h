#ifndef __OLED__H
#define __OLED__H
#include "main.h"

//说明:
//----------------------------------------------------------------
//GND    电源地
//VCC  接3.3v电源
//D0   接PA5  (SPI1_SCK===>LCD_SCK)   串行时钟线
//D1   接PA7  (SPI1_MOSI===>LCD_MOSI) 串行数据线
//RES  接PB0   硬复位 OLED
//DC   接PG15  命令/数据标志（写0—命令/写1—数据）；
//CS   接PA4   OLED片选信号
#define OLED_DC_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_2

#define OLED_RST_GPIO_Port GPIOB
#define OLED_RST_Pin GPIO_PIN_10

// #define OLED_CS_Set() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
// #define OLED_CS_Clr() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);

#define OLED_DC_Set() HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
#define OLED_DC_Clr() HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);

#define OLED_RST_Set() HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET);
#define OLED_RST_Clr() HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);

//根据数据手册提供对应的宏定义
#define BRIGHTNESS 0xCF           //亮度
#define X_WIDTH 128               //宽度
#define Y_WIDTH 64                //长度
#define PAGE 8                    //页数
#define MAX_CHAR_POSX X_WIDTH - 6 //字符宽度需要-6
#define MAX_CHAR_POSY Y_WIDTH - 6 //字符长度需要-6

//extern SPI_HandleTypeDef hspi1;
//OLED初始化
void OLED_Init(void);
//OLED清屏
void OLED_CLS(void);
//OLED刷屏
void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);

#endif //__OLED__H