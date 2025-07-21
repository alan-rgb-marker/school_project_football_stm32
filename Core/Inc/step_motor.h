#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H
#include <main.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

extern const double MY_PI;

extern const int man_range1;
extern const int man_range2;

//一部距離
extern const float one_step;

typedef struct
{
    int x;
    int y;
} Coord;

typedef struct
{
    // 滑軌tim
    TIM_HandleTypeDef *Slide_htim;
    uint32_t Slide_Channel;
    // 滑軌方向
    GPIO_TypeDef *Slide_Dir_GPIOx;
    uint16_t Slide_Dir_GPIO_Pin;
    int Slide_Dir;

    // 踢球tim
    TIM_HandleTypeDef *Ball_htim;
    uint32_t Ball_Channel;

    float man[3];

} Step_motor;

typedef struct{
    float Kp;
    float Ki;

    float integral;

    float last_time;

    float DT_SEC;
    float MAX_FREQ;
    float DEAD_ZONE;
} PIDController;

void init_pid(PIDController *pid);

float PI_Update(PIDController *pid, float error);
void Set_Step_Frequency(Step_motor *man, float freq);

void step_motor_init(Step_motor *step_motor, TIM_HandleTypeDef *Slide_htim, uint32_t Slide_Channel,GPIO_TypeDef *Slide_Dir_GPIOx, uint16_t Slide_Dir_GPIO_Pin, int Slide_Dir,TIM_HandleTypeDef *Ball_htim, uint32_t Ball_Channel);

void start_step_motor(Step_motor *step_motor);

void stop_step_motor(Step_motor *step_motor);

void dir_and_move_step_motor(Step_motor* step_motor, int dir);

float *which_man_range(Coord *ball_coord, Step_motor *step_motor);
Step_motor* which_step_motor(Coord *ball_coord, Step_motor* one, Step_motor* two, Step_motor* three);

void compute_man_location(Step_motor* step_motor);

#endif // STEP_MOTOR_H