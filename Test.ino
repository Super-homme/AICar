/*
  电机按顺序分别标记为
  0号电机（LF左前）   1号电机（RF右前）
  2号电机（RB右后）   3号电机（LB左后）
  所包含的值包括
  IN1 IN2（IN1和IN2口共同控制电机转向） Pwm（控制电机速度）
  EncodeA EncodeB （中断接口，但2560只有6个中断，所以，每个电机使用其中一个）
  Serial 1: 19 (RX1蓝线) and 18 (TX1绿线);
*/
#include <SoftwareSerial.h>
#include <Wire.h>
#include <JY901.h>

SoftwareSerial BlueTooth(50, 51); // RX,TX

int Motor_Pins[4][5] = {
  {6, 17, 7, 2, 28},
  {12, 15, 4, 21, 30},
  {11, 16, 5, 20, 26},
  {23, 8, 13, 3, 24},
};
float Angle;
float MotorCount = 0;
float Velocity[4] = {0};
int V[4] = {0};
int Pwm[4] = {0};
float PwmControl[4] = {0};
unsigned long Time; // 记录每次更改版Pwm[i]的时间

// 这个是要写在主程序里面的
// 改变目标速度值

float Target_V[4] = {0}; // 需要不同速度的时候在函数里修改数组的值

/***********************************
***********PID控制部分***开始********
************定义一些数据和函数*******
************************************/
typedef struct
{
  float Kp; //比例系数Proportional
  float Ki; //积分系数Integral
  float Kd; //微分系数Derivative

  float Ek;  //当前误差
  float Ek1; //前一次误差 e(k-1)
  float Ek2; //再前一次误差 e(k-2)
} PID_IncTypeDef;

// 初始化pid结构体
// 四套pid

PID_IncTypeDef pid0 =
{2, 0.5, 0.1, 0, 0, 0};

PID_IncTypeDef pid1 =
{2, 0.5, 0.1, 0, 0, 0};

PID_IncTypeDef pid2 =
{2, 0.5, 0.1, 0, 0, 0};
PID_IncTypeDef pid3 =
{2, 0.5, 0.1, 0, 0, 0};

PID_IncTypeDef pid[4] =
{pid0, pid1, pid2, pid3};

// 最终计算出需要变化的Pwm的值是多少
// 这个地方的PID值应该可以确定，但是四个电机有差距，决定采取四套PID数据

/************************************************************************
**********************函数功能：确定对应电机对应速度的PWM值*****************
*************************入口参数：电机编号、速度目标值*********************
*************************返回值：电机转动所需要的PWM值**********************
**************************************************************************/
float PwmInc(int n, float SetValue)
{
  float PWMInc = 0;
  float ActualValue = 0;

  PID_IncTypeDef *PID = &pid[n];

  Target_V[n] = SetValue;
  ActualValue = Velocity[n];

  PID->Ek = SetValue - ActualValue;
  PWMInc = PID->Kp * (PID->Ek - PID->Ek1) + PID->Ki * PID->Ek;

  PID->Ek2 = PID->Ek1;
  PID->Ek1 = PID->Ek;

  //PWMControl[n] = PWMControl[n] + PIDInc;

  return PWMInc;
}

// 定时器中断控制达到PWM值累加的效果，同时做一个简单的限幅， 使用millis()函数
/************************************************************************
***********函数功能：定时器中断的中断函数，同时限制PWM和速度的值*************
*************************入口参数：电机序号*************************************
*************************返回值：无****************************************
**************************************************************************/
void PwmSet()
{
  for (int i = 0; i < 4; i++)
  {
    Pwm[i] = PwmControl[i];
  }
}

void AngleSet()
{
  Angle = (float)JY901.stcAngle.Angle[2] / 32768 * 180;
  delay(1);
  while (Serial1.available())

  {

    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }
}
// 每次loop循环开始的时候检测并定时更改Pwm值
/************************************************************************
***********函数功能：检测是否时间到了更改Pwm的时候*************
*************************入口参数：电机编号*************************************
*************************返回值：无****************************************
**************************************************************************/
void PwmCheck()
{
  if (millis() - Time < 20)
  {
    return;
  }
  Time = millis();
  for (int i = 0; i < 4; i++)
  {
    PwmSet();
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
void Motor_Init()
{
  for (int i = 0; i < 4; i++)
  {
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
float Read_Motor_V(int n)
{
  // n是电机的序号
  MotorCount = 0;
  unsigned long nowtime = 0;
  nowtime = millis() + 50;
  attachInterrupt(digitalPinToInterrupt(Motor_Pins[n][3]), Read_Motor, RISING); // 2560 的中断2口对应编号0
  while (millis() < nowtime)
    ;
  detachInterrupt(digitalPinToInterrupt(Motor_Pins[n][3]));
  Velocity[n] = ((MotorCount / (11 * 56)) * 6.5 * PI) / 0.05;
  V[n] = Velocity[n];
  return Velocity[n];
}

/************************************************************************
**************函数功能：外部中断的中断函数，配合Read_Motor_V使用*************
*******************************入口参数：无********************************
********************************返回值：无*********************************
**************************************************************************/
void Read_Motor()
{
  MotorCount++;
}

/************************************************************************
**************************函数功能：驱动对应电机***************************
***********************入口参数：电机编号、速度目标值***********************
**********************************返回值：无*******************************
**************************************************************************/
void Drive_motor(int n, int TargetV)
{
  // IN q IN 2 的值是0还是1 控制轮子转的方向
  Target_V[n] = TargetV;
  //PwmSet(n);
  digitalWrite(Motor_Pins[n][0], LOW);
  digitalWrite(Motor_Pins[n][1], HIGH);
  analogWrite(Motor_Pins[n][2], Pwm[n]);
}

void Drive_motor_1(int n, int TargetV)
{
  Target_V[n] = TargetV;
  digitalWrite(Motor_Pins[n][0], HIGH);
  digitalWrite(Motor_Pins[n][1], LOW);
  analogWrite(Motor_Pins[n][2], Pwm[n]);
}

/*************************************************************************
***********函数功能：控制车前进（后退只需要令电机目标速度为负）***************
**************入口参数：速度目标值（所有电机保持同样的速度）******************
*****************************返回值：无************************************
**************************************************************************/
void Stop()
{
  Target_V[4] = {0};
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(Motor_Pins[i][0], HIGH);
    digitalWrite(Motor_Pins[i][1], LOW);
    analogWrite(Motor_Pins[i][2], 30);
  }
}
void GoForward(int TargetV)
{

  Drive_motor(0, TargetV);
  Drive_motor(1, TargetV);
  Drive_motor(2, TargetV);
  Drive_motor(3, TargetV);
}

void GoBackward(int TargetV)
{

  Drive_motor_1(0, TargetV);
  Drive_motor_1(1, TargetV);
  Drive_motor_1(2, TargetV);
  Drive_motor_1(3, TargetV);
}
/************************************************************************
***************************函数功能：控制小车左右转**************************
********************************入口参数：无******************************
*********************************返回值：无*******************************
**备注：转弯的时候前面的两个轮子保持相反50转速，后轮保持之前直行或者默认的转速**
**************************************************************************/
void TurnLeft()
{
  Target_V[0] = 50;
  Target_V[1] = 50;
  Drive_motor_1(0, Target_V[0]);
  Drive_motor(1, Target_V[1]);
  Drive_motor(2, Target_V[2]);
  Drive_motor_1(3, Target_V[3]);
  //Target_V[0] = Target_V[2];
  //Target_V[1] = Target_V[2];
}

void TurnRight()
{
  Target_V[0] = 50;
  Target_V[1] = 50;
  Drive_motor(0, Target_V[0]);
  Drive_motor_1(1, Target_V[1]);
  Drive_motor_1(2, Target_V[2]);
  Drive_motor(3, Target_V[3]);
  //Target_V[0] = Target_V[2];
  //Target_V[1] = Target_V[2];
}

/************************************************************************
****************函数功能：在串口监视窗口显示所有电机实时转速***************
****************************入口参数：无******************************
****************************返回值：无*******************************
**************************************************************************/
void PrintV()
{
  for (int i = 0; i < 4; i++)
  {
    Serial.print("Motor");
    Serial.print(i);
    Serial.print(":");
    Serial.print(Read_Motor_V(i));
    Serial.print("  ");
  }
  Serial.println();
}
/************************************************************************
****************函数功能：在串口监视窗口显示所有电机实时PWM值***************
****************************入口参数：无******************************
****************************返回值：无*******************************
**************************************************************************/
void PrintPwm()
{
  for (int i = 0; i < 4; i++)
  {
    Serial.print("PWM");
    Serial.print(i);
    Serial.print(":");
    Serial.print(Pwm[i]);
    Serial.print("  ");
    delay(10);
  }
  Serial.println();
}

void PrintAngle()
{
  Serial.print("Angle:");
  Serial.println(Angle);
}

void setup()
{
  Motor_Init();       // 初始化
  Serial.begin(9600); // 串口
  BlueTooth.begin(9600);
  Serial1.begin(115200);
  for (int i = 0; i < 4; i++)
  {
    PwmSet();
  }
  Time = millis();
}

void loop()
{
  for (int i = 0; i < 4; i++)
  {
    PwmControl[i] = PwmControl[i] + PwmInc(i, Target_V[i]);
  }
  PwmCheck();
  AngleSet();

  char ch = BlueTooth.read(); // 传递信息
  //delay(10);

  switch (ch)
  {
    case 'S':
      Motor_Init();
      break;

    case 'F':
      GoForward(40);
      break;

    case 'B':
      GoBackward(40);
      break;
    case 'L':

      TurnLeft();
      break;
    case 'R':

      TurnRight();
      break;
  }
  //delay(100);
  //PrintV();
  //PrintAngle();

}
