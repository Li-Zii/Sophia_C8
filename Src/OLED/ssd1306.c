/**
  ******************************************************************************
  * @author  Huang.
  * @version V3.0
  * @date    2021-5-08
  * @brief   1.3’OLED配置C文件
  ******************************************************************************
  * @describe:引用：https://github.com/4ilo/ssd1306-stm32HAL
  *				1、修改了初始化的参数
  *				2、修改了原生的显示函数，增加了x，y参数
  *				3、增加了立即清屏、立即整屏全亮、屏幕开关的等函数，无需使用更新显示指令。
  *				4、增加了数字和中文的显示函数，中文需要fonts.c中的字库支持
  *				5、修复了一些原生的bug
  *				6、待增加BMP图片显示
  *
  ******************************************************************************
  */


#include "ssd1306.h"


// Screenbuffer
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8]={0};

// Screen object
static SSD1306_t SSD1306;


//
//  Send a byte to the command register
//
static uint8_t ssd1306_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t command)
{
    return HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x00, 1, &command, 1, 10);
}

//
//  Send a byte to the command register
//
static uint8_t ssd1306_WriteData(I2C_HandleTypeDef *hi2c, uint8_t data)
{
    return HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x40, 1, &data, 1, 10);
}


//
//  Initialize the oled screen
//
uint8_t ssd1306_Init(I2C_HandleTypeDef *hi2c)
{
    // Wait for the screen to boot
    HAL_Delay(200);
    int status = 0;

    // Init LCD
	status += ssd1306_WriteCommand(hi2c, 0xAE);   // Display off
	status += ssd1306_WriteCommand(hi2c, 0x02);   // Set low column address
	status += ssd1306_WriteCommand(hi2c, 0x10);   // Set high column address
	status += ssd1306_WriteCommand(hi2c, 0x40);   // Set start line address
	status += ssd1306_WriteCommand(hi2c, 0xB0);   // Set Page Start Address for Page Addressing Mode,0-7

	status += ssd1306_WriteCommand(hi2c, 0x81);   // set contrast control register
	status += ssd1306_WriteCommand(hi2c, 0xFF);	  // 128
	status += ssd1306_WriteCommand(hi2c, 0xA1);   // Set segment re-map 0 to 127
	status += ssd1306_WriteCommand(hi2c, 0xA6);   // Set normal display
	
	status += ssd1306_WriteCommand(hi2c, 0xA8);   // Set multiplex ratio(1 to 64)
	status += ssd1306_WriteCommand(hi2c, 0x3F);	  // 1/32 duty
	
	status += ssd1306_WriteCommand(hi2c, 0xC8);   // Set COM Output Scan Direction
	
	status += ssd1306_WriteCommand(hi2c, 0xD3);   // Set display offset
	status += ssd1306_WriteCommand(hi2c, 0x00);   // No offset
	
	status += ssd1306_WriteCommand(hi2c, 0xD5);   // Set display clock divide ratio/oscillator frequency
	status += ssd1306_WriteCommand(hi2c, 0x80);
	
	status += ssd1306_WriteCommand(hi2c, 0xD8);   // Set area color mode off
	status += ssd1306_WriteCommand(hi2c, 0x05);
	
	status += ssd1306_WriteCommand(hi2c, 0xD9);   // Set pre-charge period
	status += ssd1306_WriteCommand(hi2c, 0xF1);
	
	status += ssd1306_WriteCommand(hi2c, 0xDA);   // Set com pins hardware configuration
#ifdef SSD1306_COM_LR_REMAP
	status += ssd1306_WriteCommand(hi2c, 0x32);   // Enable COM left/right remap
#else
	status += ssd1306_WriteCommand(hi2c, 0x12);   // Do not use COM left/right remap
#endif // SSD1306_COM_LR_REMAP

    status += ssd1306_WriteCommand(hi2c, 0xDB);   // Set vcomh
    status += ssd1306_WriteCommand(hi2c, 0x30);   // 0x20,0.77xVcc
	
    status += ssd1306_WriteCommand(hi2c, 0x8D);   // Set DC-DC enable
    status += ssd1306_WriteCommand(hi2c, 0x14);   //
    status += ssd1306_WriteCommand(hi2c, 0xAF);   // Turn on SSD1306 panel

    if (status != 0) {
        return 1;
    }

    // Clear screen
    ssd1306_Fill(Black);
//	OLED_Clear(hi2c);

    // Flush buffer to screen
    ssd1306_UpdateScreen(hi2c);

    // Set default values for screen object
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;

    SSD1306.Initialized = 1;

    return 0;
}

//
//fill_Picture
//
void fill_picture(I2C_HandleTypeDef *hi2c,unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		ssd1306_WriteCommand(hi2c,0xB0+m);		//page0-page1
		ssd1306_WriteCommand(hi2c,0x02);		//low column start address
		ssd1306_WriteCommand(hi2c,0x10);		//high column start address
		for(n=0;n<SSD1306_WIDTH;n++)
			{
				ssd1306_WriteData(hi2c,fill_Data);
			}
	}
}

//
//开启OLED显示 
//
void ssd1306_Display_On(I2C_HandleTypeDef *hi2c)
{
	ssd1306_WriteCommand(hi2c,0X8D);  //SET DCDC命令
	ssd1306_WriteCommand(hi2c,0X14);  //DCDC ON
	ssd1306_WriteCommand(hi2c,0XAF);  //DISPLAY ON
}
//
//关闭OLED显示 
//
void ssd1306_Display_Off(I2C_HandleTypeDef *hi2c)
{
	ssd1306_WriteCommand(hi2c,0X8D);  //SET DCDC命令
	ssd1306_WriteCommand(hi2c,0X10);  //DCDC OFF
	ssd1306_WriteCommand(hi2c,0XAE);  //DISPLAY OFF
}	

//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void ssd306_Clear(I2C_HandleTypeDef *hi2c)
{  
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		ssd1306_WriteCommand (hi2c,0xB0+i);    //设置页地址（0~7）
		ssd1306_WriteCommand (hi2c,0x02);      //设置显示位置—列低地址
		ssd1306_WriteCommand (hi2c,0x10);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)
			ssd1306_WriteData(hi2c,0); 
	} //更新显示
}
void ssd1306_On(I2C_HandleTypeDef *hi2c)
{  
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		ssd1306_WriteCommand (hi2c,0xB0+i);    //设置页地址（0~7）
		ssd1306_WriteCommand (hi2c,0x02);      //设置显示位置—列低地址
		ssd1306_WriteCommand (hi2c,0x10);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)
			ssd1306_WriteData(hi2c,1); 
	} //更新显示
}


//  Fill the whole screen with the given color

void ssd1306_Fill(SSD1306_COLOR color)
{
    // Fill screenbuffer with a constant value (color)
    uint32_t i;

    for(i = 0; i < sizeof(SSD1306_Buffer); i++)
    {
        SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

//
//  Write the screenbuffer with changed to the screen
//
void ssd1306_UpdateScreen(I2C_HandleTypeDef *hi2c)
{
    uint8_t i;

    for (i = 0; i < 8; i++) 
	{
        ssd1306_WriteCommand(hi2c, 0xB0 + i);
        ssd1306_WriteCommand(hi2c, 0x02);
        ssd1306_WriteCommand(hi2c, 0x10);

        HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
    }
}

//
//  Draw one pixel in the screenbuffer
//  X => X Coordinate
//  Y => Y Coordinate
//  color => Pixel color
//
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        // Don't write outside the buffer
        return;
    }

    // Check if pixel should be inverted
    if (SSD1306.Inverted)
    {
        color = (SSD1306_COLOR)!color;
    }

    // Draw in the correct color
    if (color == White)
    {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    }
    else
    {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}


//
//  Draw 1 char to the screen buffer
//  ch      => Character to write
//  Font    => Font to use
//  color   => Black or White
//
char ssd1306_WriteChar(uint8_t x,uint8_t y,char ch, FontDef Font, SSD1306_COLOR color)
{
    uint32_t i, b, j;
	
	ssd1306_SetCursor(x,y);
	
    // Check remaining space on current line
    if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
        SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
    {
        // Not enough space on current line
        return 0;
    }

    // Translate font to screenbuffer
    for (i = 0; i < Font.FontHeight; i++)
    {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for (j = 0; j < Font.FontWidth; j++)
        {
            if ((b << j) & 0x8000)
            {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
            }
            else
            {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }

    // Return written char for validation
    return ch;
}

//m^n函数
uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//显示数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void ssd1306_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,FontDef Font, SSD1306_COLOR color)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				ssd1306_WriteChar(x+Font.FontWidth*t,y,' ',Font,color);
				continue;
			}else enshow=1; 
		 	 
		}
	 	ssd1306_WriteChar(x+Font.FontWidth*t,y,temp+'0',Font,color); 
	}
} 

//显示汉字
void ssd1306_ShowCHinese(uint8_t x,uint8_t y,uint8_t no,uint8_t color)
{      			    
	uint8_t t,j,b=0;
	
	ssd1306_SetCursor(x,y);	
    for(t=0;t<16;t++)
	{
		b = Hzk[2*no][t];
        for (j = 0; j < 8; j++)
        {
            if ((b >> j) & 0x01)
            {
                ssd1306_DrawPixel(SSD1306.CurrentX + t, (SSD1306.CurrentY + j), (SSD1306_COLOR) color);
            }
            else
            {
                ssd1306_DrawPixel(SSD1306.CurrentX + t, (SSD1306.CurrentY + j), (SSD1306_COLOR)!color);
            }
        }
    }	
	ssd1306_SetCursor(x,y+8);	
    for(t=0;t<16;t++)
	{	
		b = Hzk[2*no+1][t];
        for (j = 0; j < 8; j++)
        {
            if ((b >> j) & 0x01)
            {
                ssd1306_DrawPixel(SSD1306.CurrentX + t, (SSD1306.CurrentY + j), (SSD1306_COLOR) color);
            }
            else
            {
                ssd1306_DrawPixel(SSD1306.CurrentX + t, (SSD1306.CurrentY + j), (SSD1306_COLOR)!color);
            }
        }
     }					
}

///***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～64*****************/
//void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
//{ 	
// unsigned int j=0;
// unsigned char x,y;
//  
//  if(y1%8==0) y=y1/8;      
//  else y=y1/8+1;
//	for(y=y0;y<y1;y++)
//	{
//		OLED_Set_Pos(x0,y);
//    for(x=x0;x<x1;x++)
//	    {      
//	    	OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
//	    }
//	}
//} 

//
//  Write full string to screenbuffer
//
char ssd1306_WriteString(uint8_t x,uint8_t y,char* str, FontDef Font, SSD1306_COLOR color)
{
	
//	ssd1306_SetCursor(x,y);
    // Write until null-byte
    while (*str)
    {
        if (ssd1306_WriteChar(x,y,*str, Font, color) != *str)
        {
            // Char could not be written
            return *str;
        }

        // Next char
		// The current space is now taken
		x += Font.FontWidth;
        str++;
    }

    // Everything ok
    return *str;
}

//
//  Invert background/foreground colors
//
void ssd1306_InvertColors(void)
{
    SSD1306.Inverted = !SSD1306.Inverted;
}

//
//  Set cursor position
//
void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}
