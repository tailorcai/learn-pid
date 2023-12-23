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

class MC_ClosedVelocity: public virtual MotorControl {
  public:
    int   TargetVelocity;
    volatile bool Encoder_changed;
    float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //相关速度PID参数

    MC_ClosedVelocity():Encoder_changed(false) {
      Velcity_Kp = 20;
      Velcity_Ki = 5;
      TargetVelocity = 50;
    }
    virtual void encoder_handler() override {
      Encoder = Read_Encoder();
      Encoder_changed = true;
    }
    virtual void loop() override;
    virtual void show() override;
  protected:
    int FeedbackControl(int TargetVelocity, int CurrentVelocity);
};

class MC_ClosedPosition: public virtual MotorControl {
  public:
    int CurrentPosition;
    volatile bool changed;

    int   TargetCircle;
    int Position_Kp, Position_Ki, Position_Kd;

    MC_ClosedPosition():CurrentPosition(0),Position_Kp(120), Position_Ki(0.1), Position_Kd(400),TargetCircle(10) {
    }
    virtual void encoder_handler() override {
      Encoder = Read_Encoder();
      CurrentPosition += Encoder;
      changed = true;
    }
    virtual void loop() override;
    virtual void show() override;
  protected:
    int FeedbackControl(float, int);

};
class MC_OpenVelocity: public virtual MotorControl {
  public:
    int  Step;
    MC_OpenVelocity():Step(0) {

    }

    virtual void loop() override;
};

class MC_ClosedPosition2: public virtual MC_ClosedPosition {
  public:
    virtual void loop() override;
    virtual void show() override;
};

class MC_ClosedVP: public virtual MC_ClosedPosition, public virtual MC_ClosedVelocity {
  public:
    MC_ClosedVP() {
      TargetVelocity = 90;
      TargetCircle = 20;
      Velcity_Kp = 80;
      Velcity_Ki = 10;
    }
    virtual void loop() override;
    virtual void show() override;
    virtual void encoder_handler() override {
      MC_ClosedPosition::encoder_handler();
    }
};


// extern void SetPWM(int);
// extern int Velocity_FeedbackControl(int,int);

#endif