#ifndef __OLED_I2C_H
#define	__OLED_I2C_H

#include "main.h"

extern uint8_t RevState;
extern unsigned char brightness;

#define OLED_ADDRESS	0x78 //通过调整0R电阻,屏可以0x78和0x7A两个地址 -- 默认0x78

void I2C_Configuration(void);
void I2C_WriteByte(uint8_t addr,uint8_t data);
void WriteCmd(unsigned char I2C_Command);
void WriteDat(unsigned char I2C_Data);
void OLED_Init(void);
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);
void OLED_ShowChar(unsigned char x, unsigned char y, unsigned char ch, unsigned char TextSize);
int  OLED_ShowInt(unsigned char x, unsigned char y, int val, unsigned char TextSize);
void OLED_ShowRevStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);
void OLED_ShowRevChar(unsigned char x, unsigned char y, unsigned char ch, unsigned char TextSize);
void OLED_SetBrightness(unsigned char val);
void OLED_Reverse(void);//反色显示
int  OLED_ShowFloat(unsigned char x, unsigned char y, double val, unsigned char TextSize);
void OLED_DrawPoint(unsigned char x,unsigned char y,unsigned char t);
void OLED_Scroll(uint8_t direction, uint8_t startPage, uint8_t endPage, uint8_t hSpeed, uint8_t vSpeed);
void OLED_Fill_Area(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned char fill_Data);
void BackGround(void);
void wave_show (int wave);
#endif
