#include "step_motor.h"
#include <math.h> // For fabs

const double MY_PI = 3.14159265358979323846;

// Y-axis boundaries for player selection on a rod
const int man_range1 = 100;
const int man_range2 = 200;

const float one_step = 31 * MY_PI / 800;

// X-axis boundaries for rod selection
const int ROD_ONE_BOUNDARY_X = 440;
const int ROD_TWO_BOUNDARY_X = 240;


/* --------------------------------------------step_motor----------------------------------------------- */
void step_motor_init(Step_motor *step_motor, TIM_HandleTypeDef *Slide_htim, uint32_t Slide_Channel, GPIO_TypeDef *Slide_Dir_GPIOx, uint16_t Slide_Dir_GPIO_Pin, int Slide_Dir, TIM_HandleTypeDef *Ball_htim, uint32_t Ball_Channel, int min_range, int max_range)
{
    step_motor->Slide_htim = Slide_htim;
    step_motor->Slide_Channel = Slide_Channel;
    step_motor->Slide_Dir_GPIOx = Slide_Dir_GPIOx;
    step_motor->Slide_Dir_GPIO_Pin = Slide_Dir_GPIO_Pin;
    step_motor->Slide_Dir = Slide_Dir;
    step_motor->Ball_htim = Ball_htim;
    step_motor->Ball_Channel = Ball_Channel;
    step_motor->man[0] = 0;
    step_motor->man[1] = 100;
    step_motor->man[2] = 200;

    step_motor->kick_step = 0;


    step_motor->ball_x_min_range = min_range;
    step_motor->ball_x_max_range = max_range;

    init_pid(&step_motor->pid);
}

void start_step_motor(Step_motor *step_motor)
{
    if (step_motor != NULL)
    {
        HAL_TIM_Base_Start_IT(step_motor->Slide_htim);
        HAL_TIM_PWM_Start(step_motor->Slide_htim, step_motor->Slide_Channel);
    }
}

void stop_step_motor(Step_motor *step_motor)
{
    if (step_motor != NULL)
    {

        HAL_TIM_Base_Stop_IT(step_motor->Slide_htim);
        HAL_TIM_PWM_Stop(step_motor->Slide_htim, step_motor->Slide_Channel);
    }
}

void dir_and_move_step_motor(Step_motor *step_motor, int dir)
{
    if (step_motor != NULL)
    {
        start_step_motor(step_motor);
        step_motor->Slide_Dir = dir;
        HAL_GPIO_WritePin(step_motor->Slide_Dir_GPIOx, step_motor->Slide_Dir_GPIO_Pin, dir == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void kick_start_step_motor(Step_motor *step_motor)
{
    if (step_motor != NULL)
    {
        HAL_TIM_Base_Start_IT(step_motor->Ball_htim);
        HAL_TIM_PWM_Start(step_motor->Ball_htim, step_motor->Ball_Channel);
    }
}

void kick_stop_step_motor(Step_motor *step_motor)
{
    if (step_motor != NULL)
    {
        HAL_TIM_Base_Stop_IT(step_motor->Ball_htim);
        HAL_TIM_PWM_Stop(step_motor->Ball_htim, step_motor->Ball_Channel);
    }
}
/* --------------------------------------------step_motor----------------------------------------------- */


/* ------------------------------------------判斷球在小人的哪個範圍--------------------------------------------- */
float *which_man_range(Coord *ball_coord, Step_motor *step_motor)
{
    if (ball_coord != NULL)
    {
        if (ball_coord->y > man_range2)
        {
            return &step_motor->man[2];
        }
        else if (ball_coord->y > man_range1)
        {
            return &step_motor->man[1];
        }
        else
        {
            return &step_motor->man[0];
        }
    }
    else
    {
        return NULL;
    }
}

Step_motor *which_step_motor(Coord *ball_coord, Step_motor *one, Step_motor *two, Step_motor *three)
{
    if (ball_coord->x > ROD_ONE_BOUNDARY_X)
    {
        // 停止只保留一個馬達在場上動
        stop_step_motor(two);
        stop_step_motor(three);
        return one;
    }
    else if (ball_coord->x > ROD_TWO_BOUNDARY_X)
    {
        stop_step_motor(one);
        stop_step_motor(three);
        return two;
    }
    else if (ball_coord->x > 0)
    {
        stop_step_motor(one);
        stop_step_motor(two);
        return three;
    }
    else
    {
        // x <= 0 的情況：全部停
        stop_step_motor(one);
        stop_step_motor(two);
        stop_step_motor(three);
        return NULL;
    }
}

/* ------------------------------------------判斷球在小人的哪個範圍--------------------------------------------- */

/* ---------------------------------------------tim小人位置計算----------------------------------------------- */
void compute_man_location(Step_motor *step_motor)
{
    if (step_motor != NULL)
    {
        step_motor->man[0] += step_motor->Slide_Dir * one_step;
        step_motor->man[1] += step_motor->Slide_Dir * one_step;
        step_motor->man[2] += step_motor->Slide_Dir * one_step;
    }
}
void count_kick_step(Step_motor *step_motor)
{
    if (step_motor != NULL)
    {
        step_motor->kick_step++;
    }   
}
/* ---------------------------------------------pi----------------------------------------------- */

void init_pid(PIDController *pid)
{
    if (pid != NULL)
    {
        pid->Ki = 3.0f;
        pid->Kp = 40.0f;
        pid->DT_SEC = 0.02f;
        pid->MAX_FREQ = 2000.0f;
        pid->DEAD_ZONE = 150.0f;
        pid->integral = 0.0f;
        pid->last_time = HAL_GetTick();
    }
}

float PI_Update(PIDController *pid, float error)
{
    // 使用固定的時間差，讓 PI 控制器行為更穩定
    float dt = pid->DT_SEC;

    // 積分項累積誤差
    pid->integral += error * dt;

    // PI 控制輸出
    float output = pid->Kp * error + pid->Ki * pid->integral;

    // 安全限制
    if (output > pid->MAX_FREQ)
        output = pid->MAX_FREQ;
    if (output < -pid->MAX_FREQ)
        output = -pid->MAX_FREQ;
    if (fabs(output) < pid->DEAD_ZONE)
        output = pid->DEAD_ZONE;

    return output;
}

void Set_Step_Frequency(Step_motor *man, float freq)
{
    if (man == NULL)
    {
        // printf("Error: ball_in_man_range is NULL\n");
        return; // 或其他默認值
    }
    float abs_freq = fabs(freq);
    uint32_t arr;

    if (abs_freq == 0)
    {
        // 關閉 PWM 輸出
        __HAL_TIM_SET_COMPARE(man->Slide_htim, man->Slide_Channel, 0);
        return;
    }

    // 計算對應 ARR（假設 TIM2 用 1MHz）
    arr = 1000000 / abs_freq;
    __HAL_TIM_SET_AUTORELOAD(man->Slide_htim, arr);
    __HAL_TIM_SET_COMPARE(man->Slide_htim, man->Slide_Channel, arr / 2);
    __HAL_TIM_SET_COUNTER(man->Slide_htim, 0);
}
