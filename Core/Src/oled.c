#include "oled.h"

uint8_t OLED_GRAM[128][8];

//延时函数定义
void uDelay(uint8_t l)
{
    uint32_t t = 10 * l;

    while (t--)
        ;
}

void Delay_ms(uint8_t n)
{
    HAL_Delay(n);
}

/*********************写一位数据到SPI*******************/
void OLED_WB(uint8_t data)
{
    //HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
    uDelay(2);
}

/*******************一个字节数据写入***********************/
void OLED_WrDat(uint8_t dat)
{
    OLED_DC_Set();
    OLED_WB(dat);
}

/********************一条指令写入**********************/
void OLED_WrCmd(uint8_t cmd)
{
    OLED_DC_Clr();
    OLED_WB(cmd);
}

/********************更新显存到OLED**********************/
void OLED_Refresh_Gram(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WrCmd(0xb0 + i); //设置页地址（0~7）
        OLED_WrCmd(0x00);     //设置显示位置—列低地址
        OLED_WrCmd(0x10);     //设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            OLED_WrDat(OLED_GRAM[n][i]);
    }
}

//画点
//x:0~127
//y:0~63
//t:1 填充 0,清空
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t pos, bx, temp = 0;
    if (x > 127 || y > 63)
        return; //3?3?·??§á?.
    pos = 7 - y / 8;
    bx = y % 8;
    temp = 1 << (7 - bx);
    if (t)
        OLED_GRAM[x][pos] |= temp;
    else
        OLED_GRAM[x][pos] &= ~temp;
}

/**********************写满屏数据**********************/
//x1,y1,x2,y2 填充区域的对角坐标
//确保x1<=x2;y1<=y2 0<=x1<=127 0<=y1<=63
//dot:0,清空;1,填充
void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot)
{
    uint8_t x, y;
    for (x = x1; x <= x2; x++)
    {
        for (y = y1; y <= y2; y++)
            OLED_DrawPoint(x, y, dot);
    }
    OLED_Refresh_Gram();
}

/*********************清屏函数***********************/
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void OLED_CLS(void)
{
    OLED_Fill(0, 0, 127, 63, 0x00);
    Delay_ms(200);
}

/*********************12864初始化***********************/
void OLED_Init(void)
{
    OLED_RST_Clr();
    Delay_ms(100);
    OLED_RST_Set();
    OLED_WrCmd(0xAE); //display off
    OLED_WrCmd(0x20); //Set Memory Addressing Mode
    OLED_WrCmd(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    OLED_WrCmd(0xb0); //Set Page Start Address for Page Addressing Mode,0-7
    OLED_WrCmd(0xc8); //Set COM Output Scan Direction
    OLED_WrCmd(0x00); //---set low column address
    OLED_WrCmd(0x10); //---set high column address
    OLED_WrCmd(0x40); //--set start line address
    OLED_WrCmd(0x81); //--set contrast control register
    OLED_WrCmd(0xff); //亮度调节 0x00~0xff
    OLED_WrCmd(0xa1); //--set segment re-map 0 to 127
    OLED_WrCmd(0xa6); //--set normal display
    OLED_WrCmd(0xa8); //--set multiplex ratio(1 to 64)
    OLED_WrCmd(0x3F); //
    OLED_WrCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    OLED_WrCmd(0xd3); //-set display offset
    OLED_WrCmd(0x00); //-not offset
    OLED_WrCmd(0xd5); //--set display clock divide ratio/oscillator frequency
    OLED_WrCmd(0xf0); //--set divide ratio
    OLED_WrCmd(0xd9); //--set pre-charge period
    OLED_WrCmd(0x22); //
    OLED_WrCmd(0xda); //--set com pins hardware configuration
    OLED_WrCmd(0x12);
    OLED_WrCmd(0xdb); //--set vcomh
    OLED_WrCmd(0x20); //0x20,0.77xVcc
    OLED_WrCmd(0x8d); //--set DC-DC enable
    OLED_WrCmd(0x14); //
    OLED_WrCmd(0xaf); //--turn on oled panel
}