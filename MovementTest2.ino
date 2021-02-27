/*
电机按顺序分别标记为
0号电机（LF左前）   1号电机（RF右前）
2号电机（RB右后）   3号电机（LB左后）
所包含的值包括
IN1 IN2（IN1和IN2口共同控制电机转向） Pwm（控制电机速度） 
EncodeA EncodeB （中断接口，但2560只有6个中断，所以，每个电机使用其中一个）
*/

#include <MsTimer2.h>     //定时器库的头文件

int Motor_Pins[4][5] = {
    {6, 7, 29, 2, 28},
    {12, 13, 4, 21, 30},
    {10, 11, 5, 20, 26},
    {8, 9, 15, 3, 24},
};
float MotorCount = 0;
float Velocity[4] = {0};
int V[4] = {0};
int Pwm[4] = {0}; 

// 这个是要写在主程序里面的 
// 改变目标速度值

float Target_V[4] = {50, 50, 50, 50};  // 需要不同速度的时候在函数里修改数组的值

/***********************************
***********PID控制部分***开始********
************定义一些数据和函数*******
************************************/
typedef struct
{
  float Kp;                       //比例系数Proportional
  float Ki;                       //积分系数Integral
  float Kd;                       //微分系数Derivative
 
  float Ek;                       //当前误差
  float Ek1;                      //前一次误差 e(k-1)
  float Ek2;                      //再前一次误差 e(k-2)
}PID_IncTypeDef;

// 初始化pid结构体
 // 四套pid

PID_IncTypeDef pid0 = 
    {1, 0.5, 0.1, 0, 0, 0 }; 

PID_IncTypeDef pid1 = 
    {1, 0.5, 0.1, 0, 0, 0 }; 

PID_IncTypeDef pid2 = 
    {1, 0.5, 0.1, 0, 0, 0 }; 
PID_IncTypeDef pid3 = 
    {1, 0.5, 0.1, 0, 0, 0 }; 

PID_IncTypeDef pid[4] = 
    {pid0, pid1, pid2, pid3};


// 最终计算出需要变化的Pwm的值是多少
// 这个地方的PID值应该可以确定，但是四个电机有差距，决定采取四套PID数据

/************************************************************************
**********************函数功能：确定对应电机对应速度的PWM值*****************
*************************入口参数：电机编号、速度目标值*********************
*************************返回值：电机转动所需要的PWM值**********************
**************************************************************************/
float PwmControl(int n, float SetValue)
{
  float PIDInc, ActualValue, PWMControl;                                  
  
  PID_IncTypeDef *PID = &pid[n];

  
  ActualValue = Read_Motor_V(n);

  PID->Ek = SetValue - ActualValue;
  PIDInc = (PID->Kp * PID->Ek) - (PID->Ki * PID->Ek1) + (PID->Kd * PID->Ek2);
 
  PID->Ek2 = PID->Ek1;
  PID->Ek1 = PID->Ek;

  PWMControl += PIDInc;

  if(PWMControl < 0){
    PWMControl = 0;     
  }
  if(PWMControl > 250){
    PWMControl=250;  
  }

  return PWMControl;
}

// 定时器中断控制达到PWM值累加的效果，同时做一个简单的限幅， 使用MS Timer2库
/************************************************************************
***********函数功能：定时器中断的中断函数，同时限制PWM和速度的值*************
*************************入口参数：无*************************************
*************************返回值：无****************************************
**************************************************************************/
void OnTimer() {
    for(int i = 0; i < 4; i++) {
        Pwm[i] = PwmControl(i, Target_V[i]);
        // 给PWM和速度做一个限幅
        if (Pwm[i] > 255)
            Pwm[i] = 255;
        if (Pwm[i] < 0)
            Pwm[i] = 0;
        if (Target_V[i] >50)
            Target_V[i] = 50;
        if (Target_V[i] < -50)
            Target_V[i] = -50;
    }
}
 

/***********************************
***********PID控制部分***结束********
************************************/


/************************************************************************
**********************函数功能：电机初始化，设置所有接口********************
*******************************入口参数：无********************************
********************************返回值：无*********************************
**************************************************************************/
void Motor_Init() {
    for (int i = 0; i < 4; i++) {
        pinMode(Motor_Pins[i][0], OUTPUT);
        pinMode(Motor_Pins[i][1], OUTPUT);
        pinMode(Motor_Pins[i][2], OUTPUT);
        pinMode(Motor_Pins[i][3], INPUT);
        pinMode(Motor_Pins[i][4], INPUT);

        digitalWrite(Motor_Pins[i][0], LOW);
        digitalWrite(Motor_Pins[i][1], LOW);
        digitalWrite(Motor_Pins[i][2], LOW);
        }
}

/************************************************************************
**********************函数功能：获得对应电机的速度***************************
*************************入口参数：电机编号********************************
*********************返回值：电机转速的浮点型数据***************************
**************************************************************************/
float Read_Motor_V(int n) {
    // n是电机的序号
    MotorCount = 0;
    unsigned long nowtime = 0;
    nowtime = millis() + 50;
    attachInterrupt(digitalPinToInterrupt(Motor_Pins[n][3]), Read_Motor, RISING); // 2560 的中断2口对应编号0
    while(millis() < nowtime);
    detachInterrupt(digitalPinToInterrupt(Motor_Pins[n][3]));
    Velocity[n] = ((MotorCount/(11*56))*6.5*PI)/0.05;
    V[n] = Velocity[n];
    return Velocity[n];
}

/************************************************************************
**************函数功能：外部中断的中断函数，配合Read_Motor_V使用*************
*******************************入口参数：无********************************
********************************返回值：无*********************************
**************************************************************************/
void Read_Motor() {
    MotorCount++;
}

/************************************************************************
**************************函数功能：驱动对应电机***************************
***********************入口参数：电机编号、速度目标值***********************
**********************************返回值：无*******************************
**************************************************************************/
void Drive_motor (int n, int TargetV) {
    // IN q IN 2 的值是0还是1 控制轮子转的方向
    Target_V[n] = TargetV; 

    if(TargetV > 0) {
        Pwm[n] = PwmControl(n, TargetV);
        digitalWrite(Motor_Pins[n][0], LOW);
        digitalWrite(Motor_Pins[n][1], HIGH);
        analogWrite(Motor_Pins[n][2], Pwm[n]);
    }
    else {
        Pwm[n] = PwmControl(n, -TargetV);
        digitalWrite(Motor_Pins[n][0], HIGH);
        digitalWrite(Motor_Pins[n][1], LOW);
        analogWrite(Motor_Pins[n][2], Pwm[n]);
    }       
}

/*************************************************************************
***********函数功能：控制车前进（后退只需要令电机目标速度为负）***************
**************入口参数：速度目标值（所有电机保持同样的速度）******************
*****************************返回值：无************************************
**************************************************************************/
void MoveForward(int TargetV) {

    Drive_motor(0, TargetV);
    Drive_motor(1, TargetV);
    Drive_motor(2, TargetV);
    Drive_motor(3, TargetV);
}

/************************************************************************
***************************函数功能：控制小车左右转**************************
********************************入口参数：无******************************
*********************************返回值：无*******************************
**备注：转弯的时候前面的两个轮子保持相反50转速，后轮保持之前直行或者默认的转速**
**************************************************************************/
void MoveTurnLeft() {
    Target_V[0] = -50;
    Target_V[1] = 50;
    Drive_motor(0, Target_V[0]);
    Drive_motor(1, Target_V[1]);
    Drive_motor(2, Target_V[2]);
    Drive_motor(3, Target_V[3]);
    Target_V[0] = Target_V[2];
    Target_V[1] = Target_V[2];
}

void MoveTurnRight() {
    Target_V[0] = 50;
    Target_V[1] = -50;
    Drive_motor(0, Target_V[0]);
    Drive_motor(1, Target_V[1]);
    Drive_motor(2, Target_V[2]);
    Drive_motor(3, Target_V[3]);
    Target_V[0] = Target_V[2];
    Target_V[1] = Target_V[2];
}




void setup() {
    Motor_Init();  // 初始化
    Serial.begin(9600);  // 串口
}

void loop() {

    for(int i = 0; i < 4; i++) {
        while(!Serial);
        Serial.print("Motor");
        Serial.print(i);
        Serial.print(":");
        Serial.print(Read_Motor_V(i));
        Serial.print("  ");
        Serial.println(Pwm[i]);
    }

    MsTimer2::set(1000, OnTimer); //设置中断，每1000ms进入一次中断服务程序 OnTimer()
    MsTimer2::start(); //开始计时

    MoveForward(40);

    delay(1000);
}
