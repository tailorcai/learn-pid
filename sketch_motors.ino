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
MotorControl* mc = new MC_ClosedVP();

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
  if(PWM<=-7000) Step=500;      //减速到反转最大速度后加速
  else if(PWM>=7000)  Step=-500; //加速到正转最大速度后减速
				
  PWM=PWM+Step; //速度渐变
  setPWM(PWM); //设置PWM
}

void MC_ClosedVelocity::loop() {
  if( Encoder_changed ) {
    int pwm=FeedbackControl(TargetVelocity, Encoder); //速度环闭环控制
      // Serial.print("PWM to be:");Serial.println(pwm);
    setPWM(pwm);
    PWM = pwm;
    Encoder_changed = false;
  }
}

void MC_ClosedPosition::loop() {
  if( changed ) {
    // mutex needed
    int p = CurrentPosition;
    changed = false;
    // mutex end

    int pwm=FeedbackControl(TargetCircle, p); //速度环闭环控制
      // Serial.print("PWM to be:");Serial.println(pwm);
    setPWM(pwm);
    PWM = pwm;
    
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

void drawStr(int line, const char* name, int t) {
  char buf[100];

  display.setCursor(0, line*10);
  display.write( name );
  display.setCursor(80,line*10);
  if (t) display.write( t>0?"+":"-" );
  display.setCursor(100,line*10);
  display.write( itoa(t,buf,10) );
}
void Oled_Show(void)
{  
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);  

  display.setCursor(20,0);
  display.write("VelocityDirect");
 
  display.setTextColor(WHITE); // Draw white text

		//显示当前速度，即编码器读数，分正负

    mc->show();

       
  display.display();
}
void MC_ClosedVelocity::show() {
  drawStr(1, "Current_V:", Encoder); 		
  drawStr(2, "Target_V :", TargetVelocity); 		
  drawStr(3, "PWM      :", PWM); 		
}

void MC_ClosedPosition::show() {
  drawStr(1, "Current_V:", Encoder); 		
  // drawStr(2, "Target_V :", mc->TargetVelocity); 		
  drawStr(2, "PWM      :", PWM); 	
  drawStr(3, "Current_P:", CurrentPosition);
  drawStr(4, "Target_P :", TargetCircle*1040*1.04);
}

void MC_ClosedPosition2::show() {
  drawStr(1, "Current_V:", Encoder); 		
  drawStr(2, "PWM      :", PWM); 	
  drawStr(3, "Current_P:", CurrentPosition);
  drawStr(4, "Target_P :", TargetCircle*1040*1.04);
}

void MC_ClosedPosition2::loop() {
  static int TimeCount = 0;
  TimeCount += 1; // 10ms each
  if(TimeCount<=300) TargetCircle=2;      
	else if(300<TimeCount) TargetCircle=-1; 
  MC_ClosedPosition::loop();
}

void MC_ClosedVP::show() {
  drawStr(1, "Current_V:", Encoder); 		
  drawStr(2, "Target_V:", TargetVelocity); 		
  drawStr(2, "PWM      :", PWM); 	
  drawStr(3, "Current_P:", CurrentPosition);
  drawStr(4, "Target_P :", TargetCircle*1040*1.04);
}

void MC_ClosedVP::loop() {
  static int TimeCount = 0;
  TimeCount += 1; // 10ms each
  if(TimeCount<=600) TargetCircle=15,TargetVelocity=40;   
	else if(600<TimeCount) TargetCircle=30,TargetVelocity=20;

  int PWM_P=MC_ClosedPosition::FeedbackControl(TargetCircle,CurrentPosition); //位置闭环控制
  int PWM_V=PWM_P/76; //PWM 值转换为速度值 76 为转换参数
  
  // PWM_P=Velocity_Restrict(PWM_P,TargetVelocity); //限幅位置环输出的 PWM
  if (PWM_V>+TargetVelocity) 
    PWM_V=+TargetVelocity;
	else if(PWM_V<-TargetVelocity) 
    PWM_V=-TargetVelocity;
	// else PWM_V=PWM_V;

  PWM=MC_ClosedVelocity::FeedbackControl(PWM_V, Encoder); //速度环闭环控制 相当于位置环的输出为速度环的输入，形成串级 PID  
  setPWM(PWM); //设置PWM
}