//传感器
#include "Arduino.h"
#include "PID_v1.h"
#include "math.h"
#define sensor_0 A3
#define sensor_r1 A4
#define sensor_r2 A5
#define sensor_r3 7
#define sensor_l1 A2
#define sensor_l2 A1
#define sensor_l3 A0


//电机与电机反转
#define motor_r 11
#define motor_l 6
#define motor_l_f 9
#define motor_r_f 10
#define motor_turn 40
#define motor_turn_go 50

//舵机
#define direc 3


//积分周期
#define Cycle 10

//以下为直行的参数们
#define front_motor 90

//以下为关于直角转弯神秘参数们
#define turn_front 160
#define turn_back 160//直角转弯时的电机差速
#define direc_r 0//左转时的舵机方向（我认为是0
#define direc_l 255 //直角右转的舵机方向
#define turn_time 1000//直角转向的具体时间
//以下为关于锐角转弯的神秘参数
#define sharp_turn_front 255
#define sharp_turn_back 0//锐角转弯时的电机差速
#define sharp_direc_r 0//左转时的舵机方向（我认为是0
#define sharp_direc_l 255 //锐角角右转的舵机方向
#define sharp_turn_time 500//锐角转向的具体时间
#define test 30

//不思进取，只等开源（调用现成的库香到爆
double door_ji = 0;//舵机的预期值
double fina;//误差值
double direc_control = 128;
double p_pid = 15;//28;//就是这个值可以让舵机在最左边传感器一个为黑的时候输出最左边值
double i_pid = 0;//关掉！关掉！一定要关掉！
double d_pid = 5;
PID myPID(&fina, &direc_control, &door_ji, p_pid, i_pid, d_pid, REVERSE);

//锐角转弯函数
void sharp_go_left() {
    analogWrite(motor_l_f,sharp_turn_back);
    analogWrite(motor_r,sharp_turn_front);//左电机反转，右边电机正转
    analogWrite(direc,sharp_direc_r);//舵机控制
    delay(sharp_turn_time);//执行时间
}
//锐角转弯函数
void sharp_go_right() {
    analogWrite(motor_l,sharp_turn_front);
    analogWrite(motor_r_f,sharp_turn_back);//右电机反转，左电机正转
    analogWrite(direc,sharp_direc_l);//舵机控制
    delay(sharp_turn_time);//执行时间
}



//直角转弯函数
void go_left() {
    analogWrite(motor_l_f,turn_back);
    analogWrite(motor_r,turn_front);//左电机反转，右边电机正转
    analogWrite(direc,direc_r);//舵机控制
    delay(turn_time);//执行时间
}
//直角转弯函数
void go_right() {
    analogWrite(motor_l,turn_front);
    analogWrite(motor_r_f,turn_back);//右电机反转，左电机正转
    analogWrite(direc,direc_l);//舵机控制
    delay(turn_time);//执行时间
}





//把整个数组里的数加起来   积分数组
int i_count(float values[Cycle]) {
    int res = 0;
    for (int i = 0; i < Cycle; i++) {
        res = res + values[i];
        //Serial.println(values[i]);
    }
    Serial.print("icount:");
    Serial.println(res);
    return res;
}
//计算pid算法所需偏差值
double last_value = 128;

double Value_count(int *value) {
    double res = 0;
    int count = 0;
    double return_value;
    for (int i = -3; i < 4; i++) {
        //Serial.print(i);
        //Serial.print(":");
        //Serial.println(value[i + 3]);
        res = res + i * value[i + 3];  //检测到是1，没检测到是0
        count = count + value[i + 3];
    }
    if (count == 0){
        res = last_value;
        count = 1;
    }
    else{
        last_value = res;
    }
    Serial.print("now value count(with out /):");
    Serial.println(res);
    Serial.print("count:");
    Serial.println(count);
    return_value = (res / count);
    return return_value;
}

//pid控制   参数：   传感器数组
void Pid_control(int *value) {
    static int cycle;  //周期
    static int last_cycle = Cycle - 1;
    static float values[Cycle];  //存积分值
    float directions = 0;        //舵机方向

    //记录积分值
    values[cycle] = Value_count(value);
    //pid计算
    directions = values[cycle] * p_pid + (values[cycle] - values[last_cycle]) * d_pid + i_count(values) * i_pid;
    //操作
    if (directions >= 3) {
        directions = 3;
    } else if (directions <= -3) {
        directions = -3;
    }
    analogWrite(direc,
                (directions, -3, 3, 0, 255));
    Serial.print("now value count:");
    Serial.println(values[cycle]);
    Serial.print("direction:");
    Serial.println(directions);
    Serial.print("cycle:");
    Serial.println(cycle);


    //编辑循环
    last_cycle = cycle;
    cycle++;
    //重置循环
    if (cycle > (Cycle - 1)) {
        cycle = 0;
    }
}




//设置电机转速
void Run_motor(int speed_l, int speed_r) {
    if(speed_l >= 0 && speed_r >= 0){
        if(speed_l > 210){
            speed_l =210;
        }
        else if(speed_l < 45){
            speed_l = 45;
        }
        if(speed_r > 240){
            speed_r =240;
        }
        else if(speed_r < 45){
            speed_r = 45;
        }
        analogWrite(motor_l, speed_l);
        analogWrite(motor_r, speed_r);
        analogWrite(motor_l_f, 0);
        analogWrite(motor_r_f, 0);
    }
    else if(speed_l < 0 && speed_r >= 0){
        speed_l = -speed_l;
        if(speed_l > 210){
            speed_l =210;
        }
        else if(speed_l < 45){
            speed_l = 45;
        }
        if(speed_r > 240){
            speed_r =240;
        }
        else if(speed_r < 45){
            speed_r = 45;
        }
        analogWrite(motor_l_f, speed_l);
        analogWrite(motor_r, speed_r);
        analogWrite(motor_l, 0);
        analogWrite(motor_r_f, 0);
    }
    else if(speed_l >= 0 && speed_r < 0){
        speed_r = - speed_r;
        if(speed_l > 210){
            speed_l =210;
        }
        else if(speed_l < 45){
            speed_l = 45;
        }
        if(speed_r > 240){
            speed_r =240;
        }
        else if(speed_r < 45){
            speed_r = 45;
        }
        analogWrite(motor_l, speed_l);
        analogWrite(motor_r_f, speed_r);
        analogWrite(motor_l_f, 0);
        analogWrite(motor_r, 0);
    }
    else if(speed_l < 0 && speed_r < 0){
        speed_l = -speed_l;
        speed_r = - speed_r;
        if(speed_l > 210){
            speed_l =210;
        }
        else if(speed_l < 45){
            speed_l = 45;
        }
        if(speed_r > 240){
            speed_r =240;
        }
        else if(speed_r < 45){
            speed_r = 45;
        }
        analogWrite(motor_l_f, speed_l);
        analogWrite(motor_r_f, speed_r);
        analogWrite(motor_l, 0);
        analogWrite(motor_r, 0);
    }
}



void Run_direct(double pwm){
    analogWrite(direc, pwm);
}

//获取传感器的值
int *Get_value() {
    int *res;
    res = (int *)malloc(sizeof(int) * 7);
    res[0] = digitalRead(sensor_l3);
    res[1] = digitalRead(sensor_l2);
    res[2] = digitalRead(sensor_l1);
    res[3] = digitalRead(sensor_0);
    res[4] = digitalRead(sensor_r1);
    res[5] = digitalRead(sensor_r2);
    res[6] = digitalRead(sensor_r3);
    //for(int i = 0;i<6;i++)
    //{
    //  Serial.println(res[i]);
    //}
    return res;
}

void go_front(){
    //int err;
    //Serial.println("----------------------");
    //emmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm这下有得忙了
    int left_less = 0;
    int right_less = 0;
    if(direc_control >= 170){
        right_less = ((direc_control-170) *motor_turn);
        direc_control = 210;
        Run_motor(-(motor_turn_go + right_less-test),motor_turn_go + right_less);
    }
    else if(direc_control <= 86){
        left_less =((86 - direc_control)*motor_turn);
        direc_control = 45;
        Run_motor(motor_turn_go + left_less,-(motor_turn_go + left_less-test));
    }
    else{
        // Run_motor(front_motor - right_less + left_less,front_motor -left_less + right_less);
        Run_motor(front_motor,front_motor);
    }
    Run_direct(direc_control);

}


int left_out_of_control = 0;
int right_out_of_control = 0;
void where_are_you_pid(double error){
    if(error >= 254&&right_out_of_control<=10){
        right_out_of_control++;
        left_out_of_control = 0;

    }
    else if(error <= 1 &&left_out_of_control<=10){
        left_out_of_control++;
        right_out_of_control = 0;

    }
    else if(error >= 254 && right_out_of_control >10){
        right_out_of_control = 0;
        go_right();
    }
    else if(error <= 1 && left_out_of_control >10){
        left_out_of_control = 0;
        go_left();
    }
}
void setup() {
    Serial.begin(57600);
    //传感器
    pinMode(sensor_0, INPUT);
    pinMode(sensor_r1, INPUT);
    pinMode(sensor_r2, INPUT);
    pinMode(sensor_r3, INPUT);
    pinMode(sensor_l1, INPUT);
    pinMode(sensor_l2, INPUT);
    pinMode(sensor_l3, INPUT);

    //舵机，左右电机
    pinMode(direc, OUTPUT);
    pinMode(motor_l, OUTPUT);
    pinMode(motor_r, OUTPUT);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(45,210);
    myPID.SetSampleTime(50);
    // put your setup code here, to run once:
}
void loop() {
    int *value = NULL;
    value = Get_value();
    fina = Value_count(value);
    myPID.Compute();
    //Pid_control(value);
    Serial.print("fina:");
    Serial.println(fina);
    Serial.print("direct_conrtol:");
    Serial.println(direc_control);
    free(value);
    go_front();
    delay(49);
    // put your main code here, to run repeatedly:
}