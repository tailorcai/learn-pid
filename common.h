#ifndef __COMMON__H
#define __COMMON__H
#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 

class MotorControl {
  public:
    int   PWM;
    int   MotorRun;                             //允许电机控制标志位
    volatile int Encoder;                       //目标速度、编码器读数、PWM控制变量
    ESP32Encoder encoder;

    MotorControl(): MotorRun(0),PWM(0) {
      
    }

    virtual void encoder_handler()=0;

    void MotorEncoder_Init();
    virtual void loop() = 0;
    void setPWM(int pwm);
    virtual void show()=0;

  protected:
    int Read_Encoder(void);
};

class MC_ClosedVelocity: public MotorControl {
  public:
    int   TargetVelocity;
    volatile bool Encoder_changed;
    float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //相关速度PID参数

    MC_ClosedVelocity():Encoder_changed(false) {
      this->Velcity_Kp = 20;
      this->Velcity_Ki = 5;
      this->TargetVelocity = 50;
    }
    virtual void encoder_handler() override {
      this->Encoder = this->Read_Encoder();
      this->Encoder_changed = true;
    }
    virtual void loop() override;
    virtual void show() override;
  protected:
    int FeedbackControl(int TargetVelocity, int CurrentVelocity);
};

class MC_ClosedPosition: public MotorControl {
  public:
    int CurrentPosition;
    volatile bool changed;

    int   TargetCircle;
    int Position_Kp, Position_Ki, Position_Kd;

    MC_ClosedPosition():CurrentPosition(0),Position_Kp(120), Position_Ki(0.1), Position_Kd(400),TargetCircle(10) {
    }
    virtual void encoder_handler() override {
      this->Encoder = this->Read_Encoder();
      this->CurrentPosition += this->Encoder;
      this->changed = true;
    }
    virtual void loop() override;
    virtual void show() override;
  protected:
    int FeedbackControl(float, int);

};
class MC_OpenVelocity: public MotorControl {
  public:
    int  Step;
    MC_OpenVelocity():Step(0) {
      // this->Velcity_Kp = 20;
      // this->Velcity_Ki = 5;
      // this->TargetVelocity = 50;
    }

    virtual void loop() override;
};
// extern void SetPWM(int);
// extern int Velocity_FeedbackControl(int,int);

#endif