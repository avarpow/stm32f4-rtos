#include "main.h"

typedef struct pid
{
    float p, i, d, summax;
    float input;
    float cur;
    //internal value
    float output, lasterr, sum;
} pid;

void pid_init_set(struct pid *t, float p, float i, float d, float summax);

void pid_set(struct pid *t, float p, float i, float d, float summax);

void pid_request(struct pid *t, float input);

void pid_update(struct pid *t, float cur);

void pid_go(struct pid *t, float input);

void pid_cacl(struct pid *t);

float pid_get(struct pid *t);

typedef struct motor
{
    struct pid pos;
    struct pid speed;
    int counter_min;
    int counter_max;
    float angle_min;
    float angle_max;
    TIM_HandleTypeDef tim;
    int tim_channel;
    //private
    int output_pluse;
} motor;
void motor_init(struct motor *m, float speed_p, float speed_i, float speed_d, float speed_summax,
                float pos_p, float pos_i, float pos_d, float pos_summax,
                int counter_min, int counter_max, int angle_min, int angle_max,
                TIM_HandleTypeDef tim, int tim_channel);

void motor_move_angle(struct motor *t, float angle);
