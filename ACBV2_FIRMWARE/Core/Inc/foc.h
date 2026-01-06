#include"masterFOC.h"
#include"main.h"
#define BUFFER_SIZE 1024
typedef struct {

    float position;
    float velocity;
    float torque;
    float i_d;
    float i_q;
    float v_d;
    float v_q;
    float pid_pos_P;
    float pid_pos_I;
    float pid_pos_D;
    float pid_vel_P;
    float pid_vel_I;
    float pid_vel_D;
    float pid_cur_P;
    float pid_cur_I;
    float pid_cur_D;
    float min_angle;
    float max_angle;
    float temprature;
    float bus_voltage;
    float internal_tempreture;
} FOC_Parameter;


void Motor_Init(void);
