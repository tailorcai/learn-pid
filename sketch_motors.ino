#include <Arduino.h>
#include "mcpwm_motor.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "common.h"

#include <OneButton.h>

#define BUTTON_PIN 32
OneButton btn = OneButton(
  BUTTON_PIN,  // Input pin for the button
  true,        // Button is active LOW
  true         // Enable internal pull-up resistor
);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// int PWM=5000;          //PWM控制变量
int PWM_MAX = 7200;
int Step=500;   //速度渐变速率 相当于加速度
MotorControl* mc = new MC_ClosedPosition();

McpwmMotor motor;

void handleClick() {
  mc->MotorRun = 1 - mc->MotorRun;
  Serial.print("MotorRun=");
  Serial.println(mc->MotorRun);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  prepare();

  motor.attachMotor(0, 33, 25);
  mc->MotorEncoder_Init();

  btn.attachClick(handleClick);
  pinMode( BUTTON_PIN, INPUT_PULLUP);

}

void MotorControl::setPWM(int pwm) {
  pwm = min(max(pwm,-7200),7200);
  motor.updateMotorSpeed(0, pwm*100.0 / PWM_MAX ); 
}

void MC_OpenVelocity::loop() {
  //速度循环变化 
  if(this->PWM<=-7000) this->Step=500;      //减速到反转最大速度后加速
  else if(this->PWM>=7000)  this->Step=-500; //加速到正转最大速度后减速
				
  this->PWM=this->PWM+this->Step; //速度渐变
  this->setPWM(this->PWM); //设置PWM
}

void MC_ClosedVelocity::loop() {
  if( this->Encoder_changed ) {
    int pwm=this->FeedbackControl(this->TargetVelocity, this->Encoder); //速度环闭环控制
      // Serial.print("PWM to be:");Serial.println(pwm);
    this->setPWM(pwm);
    this->PWM = pwm;
    this->Encoder_changed = false;
  }
}

void MC_ClosedPosition::loop() {
  if( this->changed ) {
    // mutex needed
    int p = this->CurrentPosition;
    this->changed = false;
    // mutex end

    int pwm=this->FeedbackControl(this->TargetCircle, p); //速度环闭环控制
      // Serial.print("PWM to be:");Serial.println(pwm);
    this->setPWM(pwm);
    this->PWM = pwm;
    
  }
}

void loop() {
  int cnt =0;
  // mc->setPWM(PWM);
  while(1)
  {		
    cnt ++;
    btn.tick();
    delay(10);	//延迟10ms
    // LED=!LED;    //LED灯闪烁		
    // if(KEY_Scan())MortorRun=!MortorRun; //按下按键MortorRun取反
    //  open
    if(mc->MotorRun)
    {
      // ctrl_open_speed();
      mc->loop();
    } 
    else {
      mc->PWM=0;
      mc->setPWM(0); //电机停止
    }
    
    if( cnt % 20 == 0)
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
 
  display.setTextColor(WHITE); // Draw white text

		//显示当前速度，即编码器读数，分正负
		drawStr(00,20,"Current_V:"); 		
		if(mc->Encoder>=0)
		{
			drawStr(80,20,"+");
			drawStr(90,20,itoa(mc->Encoder,buf,10));
		}
		else
		{
			drawStr(80,20,"-");
			drawStr(90,20,itoa(-mc->Encoder,buf,10));
		}
		
		//显示速度控制值，即PWM，分正负
		drawStr(00,30,"PWM      :"); 		
		if(mc->PWM>=0)
		{
			drawStr(80,30,"+");
			drawStr(90,30,itoa(mc->PWM,buf,10));
		}
		else
		{
			drawStr(80,30,"-");
			drawStr(90,30,itoa(mc->PWM,buf,10));
		}

    mc->show();

       
  display.display();
}
void MC_ClosedVelocity::show() {
  char buf[100];
   display.setCursor(0, 10);
  display.write("Target_V:");
 
 if(this->TargetVelocity>=0)
  {
    drawStr(80,10,"+");
    drawStr(100,10, itoa(this->TargetVelocity,buf,10));
  }
  else
  {
    drawStr(80,10,"-");
    drawStr(100,10,itoa(-this->TargetVelocity,buf,10));
  }
}

void MC_ClosedPosition::show() {
  char buf[100];
    display.setCursor(0, 10);
  display.write("Target_P:");

    drawStr(80,10, itoa(this->TargetCircle*1040*1.04,buf,10));
    display.setCursor(0, 40);
  display.write("Current_P:");

    drawStr(80,40, itoa(this->CurrentPosition,buf,10));

}