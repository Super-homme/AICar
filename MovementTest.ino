
# define MotorLF_IN1 6   // 左前方电机的pin1——左侧L298N的IN1
# define MotorLF_IN2 7   // 左前方电机的pin2——左侧L298N的IN2
# define MotorLF_Pwm 29    // 左前方电机的PWM口，每个电机转速要分别控制，接L298N的ENA口
# define MotorLF_EncodeA 3 // 左前方电机的中断0 AB相电机 (3是中断口，与原定的PWM口换位置了）
# define MotorLF_EncodeB 28 // 左前方电机的中断1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        

volatile float MotorCount = 0; // 中断变量，轮子脉冲计数
float LF_Velocity = 0; // 轮子转速，单位cm
int LF_V = 0; // 速度变为整数
int LF_Pwm = 0; // Pwm值

void Motor_Init() {
    pinMode(MotorLF_IN1, OUTPUT);
    pinMode(MotorLF_IN2, OUTPUT);
    pinMode(MotorLF_Pwm, OUTPUT);
    pinMode(MotorLF_EncodeA, INPUT);
    pinMode(MotorLF_EncodeB, INPUT);

    digitalWrite(MotorLF_IN1, LOW);
    digitalWrite(MotorLF_IN2, LOW);
    digitalWrite(MotorLF_Pwm, LOW);
}

// 电机实际速度计算
void Read_Motor_V() {
    unsigned long nowtime = 0;
    nowtime = millis() + 50;
    attachInterrupt(1, Read_Motor, RISING); // 2560 的中断3口对应编号1
    // 这个中断端口直接写1表示3号口 也可以写成digitalPinToInterrupt(MotorLF_EncodeA)
    while(millis() < nowtime);
    detachInterrupt(0);
    LF_Velocity = ((MotorCount/(11*56))*6.5*PI)/0.05;
    LF_V = LF_Velocity;
}

// 中断函数
void Read_Motor() {
    MotorCount++;
}

void setup() {
    Motor_Init();  // 初始化
    Serial.begin(9600);  // 串口
}

void loop() {
    Read_Motor_V();
    while(!Serial);
    Serial.println(LF_Velocity);
    MotorCount = 0;
    // for (int i = 0; i< 255; i++){
    digitalWrite(MotorLF_IN1,LOW);
    digitalWrite(MotorLF_IN2,HIGH);
    analogWrite(MotorLF_Pwm,255);
    delay(50);
     //}
    
}
