#include <cstdio>
#include <Arduino.h>
// #include <driver/timer.h>
#include "common.h"

#define ENCODER_0_A 13 // CLK ENCODER 
#define ENCODER_0_B 14 // DT ENCODER 

// extern int   Encoder; //当前速度
// extern int   TargetVelocity, Encoder,PWM; //目标速度、编码器读数、PWM控制变量
// extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //相关速度PID参数
// extern int   MortorRun;  //允许电机控制标志位
// extern bool Encoder_changed;
extern MotorControl* mc;
hw_timer_t* timer_encoder = NULL;

int MotorControl::Read_Encoder(void)
{
	int Encoder_TIM = encoder.getCount();  // timer is 10ms 
  encoder.setCount(0);
	return Encoder_TIM; //返回值
} 

// void MC_ClosedVelocity::encoder_handler() {
//   Encoder = Read_Encoder();
//   Encoder_changed = true;
// }

void IRAM_ATTR EncoderRead_Func() {
  mc->encoder_handler();
  // Serial.print("Encoder is:");Serial.println(Encoder);
  // int pwm=Velocity_FeedbackControl(TargetVelocity, Encoder); //速度环闭环控制
  // // Serial.print("PWM to be:");Serial.println(pwm);
  // PWM = pwm;
	// SetPWM(pwm); //设置PWM
}

void MotorControl::MotorEncoder_Init() { 
  this->encoder.attachFullQuad ( ENCODER_0_A, ENCODER_0_B );
  this->encoder.setCount ( 0 );

  timer_encoder = timerBegin(0,80, true); // 1MHz
  timerAttachInterrupt(timer_encoder, &EncoderRead_Func, true);
  timerAlarmWrite(timer_encoder, 10000, true); // 100Hz
  timerAlarmEnable(timer_encoder);
}



int MC_ClosedVelocity::FeedbackControl(int TargetVelocity, int CurrentVelocity)
{
		int Bias;  //定义相关变量
		static int ControlVelocity =0, Last_bias =0; //静态变量，函数调用结束后其值依然存在
		
		Bias=TargetVelocity-CurrentVelocity; //求速度偏差
		
		ControlVelocity+=Velcity_Kp*(Bias-Last_bias)+Velcity_Ki*Bias;  //增量式PI控制器
                                                                   //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
	                                                                 //Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
		Last_bias=Bias;	
		return ControlVelocity; //返回速度控制值
}

int MC_ClosedPosition::FeedbackControl(float Circle, int CurrentPosition)
{
		float TargetPosition,Bias, ControlVelocity;     //定义相关变量
		static float Last_bias, Integral_Bias;          //静态变量，函数调用结束后其值依然存在
		
	  TargetPosition=Circle*1040*1.04; //目标位置=目标圈数*1040
	                                   //10ms读取一次编码器(即100HZ)，电机减速比为20，霍尔编码器精度13，AB双相组合得到4倍频，
	                                   //则转1圈编码器读数为20*13*4=1040，电机转速=Encoder*100/1040r/s 使用定时器2
	                                   //1.04是误差系数，电机本身存在误差，可根据实际情况调整该系数以提高控制精度
	
		Bias=TargetPosition-CurrentPosition; //求位置偏差
	  Integral_Bias+=Bias;
    if(Integral_Bias> 970) Integral_Bias= 970;	//积分限幅 防止到达目标位置后过冲
	  if(Integral_Bias<-970) Integral_Bias=-970;	//积分限幅 防止到达目标位置后过冲
	
		ControlVelocity=Position_Kp*Bias+Position_Ki*Integral_Bias+Position_Kd*(Bias-Last_bias);  //增量式PI控制器
	                                                                                            //Position_Kp*Bias 偏差越大速度越大
	                                                                                            //Position_Ki*Integral_Bias 减小稳态误差
	                                                                                            //Position_Kd*(Bias-Last_bias) 限制速度
    Serial.printf("target_p: %f, Bias: %f, v: %f\n", TargetPosition, Bias, ControlVelocity);

		Last_bias=Bias;	
		return ControlVelocity;    //返回速度控制值 
}