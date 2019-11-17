/***********************************************************************************
*This program is a demo of clearing screen to display red,green,blue.
*This demo was made for LCD modules with 8bit or 16bit data port.
*This program don't need to rely on any libraries and Can run directly.
**********************************************************************************/
#include "Arduino.h"

#define LCD_RD   23
#define LCD_WR   32     
#define LCD_RS   33        
#define LCD_CS   5       
#define LCD_REST 4

// для уменьшения манипуляций с битами (для прямой записи в регистр GPIO как в шину данных дисплея за один раз) паяем шину последовательно номерам GPIO
// In order to minimize computational burden when operating the 8 pins, you will want these pins to correspond to consecutive GPIO numbers (e.g. GPI12 to GPIO19)
#define PARALLEL_0  12 // шина запаяна с 12 й ноги (GPIO12 to GPIO19)

void parallel_write(uint8_t value) {
  uint32_t output =
    (REG_READ(GPIO_OUT_REG) & ~(0xFF << PARALLEL_0)) | (((uint32_t)value) << PARALLEL_0);

  REG_WRITE(GPIO_OUT_REG, output);
}

 void Lcd_Writ_Bus(unsigned char d)
 {
  parallel_write(d); 
//  PORTD = (PORTD & B00000011) | ((d) & B11111100); 
//  PORTB = (PORTB & B11111100) | ((d) & B00000011); 
 *(portOutputRegister(digitalPinToPort(LCD_WR))) &=  ~digitalPinToBitMask(LCD_WR);
 *(portOutputRegister(digitalPinToPort(LCD_WR)))|=  digitalPinToBitMask(LCD_WR);
}


void Lcd_Write_Com(unsigned char VH)  
{   
  *(portOutputRegister(digitalPinToPort(LCD_RS))) &=  ~digitalPinToBitMask(LCD_RS);//LCD_RS=0;
  Lcd_Writ_Bus(VH);
}

void Lcd_Write_Data(unsigned char VH)
{
  *(portOutputRegister(digitalPinToPort(LCD_RS)))|=  digitalPinToBitMask(LCD_RS);//LCD_RS=1;
  Lcd_Writ_Bus(VH);
}

void Lcd_Write_Com_Data(unsigned char com,unsigned char dat)
{
  Lcd_Write_Com(com);
  Lcd_Write_Data(dat);
}

void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{
        Lcd_Write_Com(0x2a); // CASET (2Ah): Column Address Set
	Lcd_Write_Data(x1>>8);
	Lcd_Write_Data(x1);
	Lcd_Write_Data(x2>>8);
	Lcd_Write_Data(x2);
        Lcd_Write_Com(0x2b); // RASET (2Bh): Row Address Set
	Lcd_Write_Data(y1>>8);
	Lcd_Write_Data(y1);
	Lcd_Write_Data(y2>>8);
	Lcd_Write_Data(y2);
	Lcd_Write_Com(0x2c); // 2c - memory write							 
}

void Lcd_Init(void)
{
  digitalWrite(LCD_REST,HIGH);
  delay(50); 
  digitalWrite(LCD_REST,LOW);
  delay(150);
  digitalWrite(LCD_REST,HIGH);
  delay(150);

  digitalWrite(LCD_CS,HIGH);
  digitalWrite(LCD_WR,HIGH);
  digitalWrite(LCD_CS,LOW);  //CS

  Lcd_Write_Com(0xF0); // CSCON (F0h): Command Set Control
  Lcd_Write_Data(0xC3);// 0xC3 -> D[7:0] = C3h enable command 2 part I
  Lcd_Write_Com(0xF0); // CSCON (F0h): Command Set Control
  Lcd_Write_Data(0x96);// D[7:0] = 96h enable command 2 part II 
  
  Lcd_Write_Com(0x36); // MADCTL (36h): Memory Data Access Control
  Lcd_Write_Data(0xE8);// 0xE8 -> 1110 1000 (Row Address Order= 1, Column Address Order= 1, Row/Column Exchange= 1, Vertical Refresh Order ‘0’ = LCD vertical refresh Top to Bottom,
                       // RGB-BGR ORDER= ‘1’ -> BGR color filter panel, MH - Horizontal Refresh Order= ‘0’ = Left to Right, D1= 0 -> NOTHING, D0= 0 -> NOTHING)
  Lcd_Write_Com(0x3A); // COLMOD (3Ah): Interface Pixel Format
  Lcd_Write_Data(0x05);// 0x05= 0101 -> "101’ = 16bit/pixel

  Lcd_Write_Com(0xB0); // IFMODE (B0h): Interface Mode Control
  Lcd_Write_Data(0x80);// 0X80 -> SPI_EN = 1 (3/4 wire serial interface selection -> DIN/SDA pin is used for 3/4 wire serial interface and DOUT pin is not used.)
  Lcd_Write_Com(0xB6); // DFC(B6): Display Function Control
  Lcd_Write_Data(0x20); // 0x20 -> RM = 1 (RGB interface)
  Lcd_Write_Data(0x02); // 0x02 -> ICS[3:0] = [0010] (Set the scan cycle when PTG selects interval scan in non-display area drive period. The scan cycle is defined by n frame
                        // periods, where n is an odd number from 3 to 31. The polarity of liquid crystal drive voltage from the gate driver is inverted in the same
                        // timing as the interval scan cycle. 
  Lcd_Write_Com(0xB5);  // BPC(B5): Blanking Porch Control
  // VFP [7:0] / VBP[7:0]: The FP [7:0] and BP [7:0] bits specify the line number of vertical front and back porch period respectively
  Lcd_Write_Data(0x02); // 0x02 -> VFP[7:0] = ...10
  Lcd_Write_Data(0x03); // 0x03 -> VBP[7:0] = ...11
  Lcd_Write_Data(0x00); // 0x00 -> 3rd parameter always must be zero
  Lcd_Write_Data(0x04); //      -> HBP[7:0] = ..100 (Back porch of Number lines is 2) specify the dotclk number of horizontal back porch period
  Lcd_Write_Com(0xB1); 
  Lcd_Write_Data(0x80); // 0x80 -> FRS[3:0]=1000 (Sets the frame frequency of full color normal mode.),
                                // DIVA[1:0]=00 -> Fosc (division ratio for internal clocks when Normal mode)  
  Lcd_Write_Data(0x10); // 0x10= 1 0000 -> RTNA [4:0] = (1 0000) (RTNA[4:0] is used to set 1H (line) period of Normal mode at CPU interface.
                        // Frame rate = 10^7 / (168+RTNA[4:0] + 32 x (15-FRS[3:0])) (480 + VFP[7:0]+VBP[7:0]))
  Lcd_Write_Com(0xB4);  // DIC (B4): Display Inversion Control
  Lcd_Write_Data(0x00); // 0x00 -> DINV=00 -> Inversion mode = Column inversion
  Lcd_Write_Com(0xB7);  // EM(B7): Entry Mode Set
  Lcd_Write_Data(0xC6); // 0xC6 -> 1100 0110, EPF[1:0] = 11 -> r(0) = b(0) = G(0) (EPF[1:0] Set the data format when 16bbp (R,G,B) to 18 bbp (r, g, b) is stored in the internal GRAM)
                        // DSTB=0 (not Deep Standby mode), [GON,DTE] = [1,1] -> G1~G480 Gate Output = Normal display
  Lcd_Write_Com(0xC5);  // VCMPCTL(C5h): VCOM Control 
  Lcd_Write_Data(0x24); // 0x24 = 0010 0100 -> VCMP[5:0] = 10 0100, 44d -> VCOM= 1.4 (VCOM - A power supply for the TFT-LCD common electrode)
  
  Lcd_Write_Com(0xE4);
  Lcd_Write_Data(0x31);
  Lcd_Write_Com(0xE8);
  Lcd_Write_Data(0x40);
  Lcd_Write_Data(0x8A);
  Lcd_Write_Data(0x00);
  Lcd_Write_Data(0x00);
  Lcd_Write_Data(0x29);
  Lcd_Write_Data(0x19);
  Lcd_Write_Data(0xA5);
  Lcd_Write_Data(0x33);
  Lcd_Write_Com(0xC2);
  Lcd_Write_Com(0xA7);
  
  Lcd_Write_Com(0xE0);
  Lcd_Write_Data(0xF0);
  Lcd_Write_Data(0x09);
  Lcd_Write_Data(0x13);
  Lcd_Write_Data(0x12);
  Lcd_Write_Data(0x12);
  Lcd_Write_Data(0x2B);
  Lcd_Write_Data(0x3C);
  Lcd_Write_Data(0x44);
  Lcd_Write_Data(0x4B);
  Lcd_Write_Data(0x1B);
  Lcd_Write_Data(0x18);
  Lcd_Write_Data(0x17);
  Lcd_Write_Data(0x1D);
  Lcd_Write_Data(0x21);

  Lcd_Write_Com(0XE1);
  Lcd_Write_Data(0xF0);
  Lcd_Write_Data(0x09);
  Lcd_Write_Data(0x13);
  Lcd_Write_Data(0x0C);
  Lcd_Write_Data(0x0D);
  Lcd_Write_Data(0x27);
  Lcd_Write_Data(0x3B);
  Lcd_Write_Data(0x44);
  Lcd_Write_Data(0x4D);
  Lcd_Write_Data(0x0B);
  Lcd_Write_Data(0x17);
  Lcd_Write_Data(0x17);
  Lcd_Write_Data(0x1D);
  Lcd_Write_Data(0x21);

  Lcd_Write_Com(0X36);
  Lcd_Write_Data(0x08);
  Lcd_Write_Com(0xF0);
  Lcd_Write_Data(0xC3);
  Lcd_Write_Com(0xF0);
  Lcd_Write_Data(0x69);
  Lcd_Write_Com(0X13);
  Lcd_Write_Com(0X11);
  Lcd_Write_Com(0X29);
}

void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c)                   
{	
  unsigned int i,j;
  Lcd_Write_Com(0x02c); //write_memory_start
  digitalWrite(LCD_RS,HIGH);
  digitalWrite(LCD_CS,LOW);
  l=l+x;
  Address_set(x,y,l,y);
  j=l*2;
  for(i=1;i<=j;i++)
  {
    Lcd_Write_Data(c);
  }
  digitalWrite(LCD_CS,HIGH);   
}

void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c)                   
{	
  unsigned int i,j;
  Lcd_Write_Com(0x02c); //write_memory_start
  digitalWrite(LCD_RS,HIGH);
  digitalWrite(LCD_CS,LOW);
  l=l+y;
  Address_set(x,y,x,l);
  j=l*2;
  for(i=1;i<=j;i++)
  { 
    Lcd_Write_Data(c);
  }
  digitalWrite(LCD_CS,HIGH);   
}

void Rect(unsigned int x,unsigned int y,unsigned int w,unsigned int h,unsigned int c)
{
  H_line(x  , y  , w, c);
  H_line(x  , y+h, w, c);
  V_line(x  , y  , h, c);
  V_line(x+w, y  , h, c);
}

void Rectf(unsigned int x,unsigned int y,unsigned int w,unsigned int h,unsigned int c)
{
  unsigned int i;
  for(i=0;i<h;i++)
  {
    H_line(x  , y  , w, c);
    H_line(x  , y+i, w, c);
  }
}
int RGB(int r,int g,int b)
{return r << 16 | g << 8 | b;
}
void LCD_Clear(unsigned int j)                   
{	
  unsigned int i,m;
 Address_set(0,0,320,480);
  Lcd_Write_Com(0x02c); //write_memory_start
  digitalWrite(LCD_RS,HIGH);
  digitalWrite(LCD_CS,LOW);


  for(i=0;i<320;i++)
    for(m=0;m<480;m++)
    {
      Lcd_Write_Data(j>>8);
      Lcd_Write_Data(j);

    }
  digitalWrite(LCD_CS,HIGH);   
}

// set the 8 pins as outputs
  void parallel_set_outputs(void) {
  REG_WRITE(GPIO_ENABLE_W1TS_REG, 0xFF << PARALLEL_0);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup started.");
  // At initialization, you need to configure all 8 data display pins a GPIOs, e.g. by setting them all as inputs:
  for (int i = 0; i < 8; i++) {
    pinMode(PARALLEL_0 + i, INPUT);
  }
  // set the 8 pins as outputs
  parallel_set_outputs();
  
  pinMode(32,OUTPUT);
  pinMode(33,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(23,OUTPUT);
  digitalWrite(32, HIGH);
  digitalWrite(33, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(23, HIGH);
  Lcd_Init();
 LCD_Clear(0xf800);
}

void loop()
{  
  Serial.println("LOOP.");
  /*
  //parallel_write(0);Serial.println("0 setted to bus.");delay (20000);
  parallel_write    (0b00000001);
  Serial.println("D0 0b00000001 setted to bus.");
  delay (30000);
  
  parallel_write    (0b00000010);
  Serial.println("D1 0b00000010 setted to bus.");
  delay (30000);

  parallel_write    (0b00000100);
  Serial.println("D2 0b00000100 setted to bus.");
  delay (30000);

  parallel_write    (0b00001000);
  Serial.println("D3 0b00001000 setted to bus.");
  delay (30000);

  parallel_write    (0b00010000);
  Serial.println("D4 0b00010000 setted to bus.");
  delay (30000);

  parallel_write    (0b00100000);
  Serial.println("D5 0b00100000 setted to bus.");
  delay (30000);

  parallel_write    (0b01000000);
  Serial.println("D6 0b01000000 setted to bus.");
  delay (30000);

  parallel_write    (0b10000000);
  Serial.println("D7 0b10000000 setted to bus.");
  delay (30000);
*/
   LCD_Clear(0xf800);
   LCD_Clear(0x07E0);
   LCD_Clear(0x001F);
     
  for(int i=0;i<1000;i++)
  {
    Rect(random(300),random(300),random(300),random(300),random(65535)); // rectangle at x, y, with, hight, color
   // Serial.print("Random: ");
   // Serial.println (random(65535));
  }
  
  LCD_Clear(0xf800);
}
