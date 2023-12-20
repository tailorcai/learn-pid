#include <Arduino.h>
#include "mcpwm_motor.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int PWM=7000;          //PWM控制变量
int PWM_MAX = 7200;
int Step=500;   //速度渐变速率 相当于加速度
int MortorRun = 1;  //允许电机控制标志位

McpwmMotor motor;

void setup() {
  // put your setup code here, to run once:
  prepare();

  motor.attachMotor(0, 33, 25);
}

void loop() {
	 while(1)
	  {		
			delay(200);	//延迟1秒
			// LED=!LED;    //LED灯闪烁		
      // if(KEY_Scan())MortorRun=!MortorRun; //按下按键MortorRun取反
			
			if(MortorRun)
			{
				//速度循环变化 
				if(PWM<=-7000)Step=500;      //减速到反转最大速度后加速
				else if(PWM>=7000)Step=-500; //加速到正转最大速度后减速
				
				PWM=PWM+Step; //速度渐变
				motor.updateMotorSpeed(0, PWM*100.0 / PWM_MAX );  //设置PWM
			}
			else PWM=0,motor.updateMotorSpeed(0, PWM*100.0 / PWM_MAX ); //电机停止
			
			Oled_Show();  //OLED显示屏显示内容
	  }
}

void prepare(void) 
{
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
}

void drawStr(int x,int y, const char* t) {
  display.setCursor(x, y);
  display.write( t);
}
void Oled_Show(void)
{  
  char buf[100];

  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);  

  display.setCursor(20,0);
  display.write("VelocityDirect");
  display.setCursor(0, 20);
  display.write("PWM       :");
  if(PWM>=0)
  {
    drawStr(80,20,"+");
    drawStr(100,20, itoa(PWM,buf,10));
  }
  else
  {
    drawStr(80,20,"-");
    drawStr(100,20,itoa(-PWM,buf,10));
  }
       
  display.display();
}