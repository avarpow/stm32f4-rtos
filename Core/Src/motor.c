#include "motor.h"
void pid_init_set(struct pid *t, float p, float i, float d, float summax)
{
    t->output = 0;
    t->lasterr = 0;
    t->sum = 0;
    t->output = 0;
    t->p = p;
    t->i = i;
    t->d = d;
    t->summax = summax;
}
void pid_set(struct pid *t, float p, float i, float d, float summax)
{
    t->p = p;
    t->i = i;
    t->d = d;
    t->summax = summax;
}
void pid_request(struct pid *t, float input)
{
    t->input = input;
}
void pid_update(struct pid *t, float cur)
{
    t->cur = cur;
}
void pid_go(struct pid *t, float input)
{
    t->input = input;
}
void pid_cacl(struct pid *t)
{
    //i
    float ierr, derr;
    ierr = t->input - t->cur;
    t->sum += ierr;
    if (t->sum > t->summax)
    {
        t->sum = t->summax;
    }
    if (t->sum < -t->summax)
    {
        t->sum = t->summax;
    }
    //d
    derr = ierr - t->lasterr;
    t->lasterr = derr;
    int temp = (t->p * ierr + t->i * t->sum + t->d * derr);
    t->output = temp;
}
float pid_get(struct pid *t)
{
    return t->output;
}

void motor_init(struct motor *m, float speed_p, float speed_i, float speed_d, float speed_summax,
                float pos_p, float pos_i, float pos_d, float pos_summax,
                int counter_min, int counter_max, int angle_min, int angle_max,
                TIM_HandleTypeDef tim, int tim_channel)
{
    m->pos.p = pos_p;
    m->pos.i = pos_i;
    m->pos.d = pos_d;
    m->speed.p = speed_p;
    m->speed.i = speed_i;
    m->speed.d = speed_d;
    m->counter_min = counter_min;
    m->counter_max = counter_max;
    m->angle_min = angle_min;
    m->angle_max = angle_max;
    m->tim = tim;
    m->tim_channel = tim_channel;
    pid_init_set(&m->pos, pos_p, pos_i, pos_d, pos_summax);
    pid_init_set(&m->speed, speed_p, speed_i, speed_d, speed_summax);
}
void motor_move_angle(struct motor *t, float angle)
{
    if (angle > t->angle_max || angle < t->angle_min)
    {
        return;
    }
    int pwm_counter;
    pwm_counter = (angle - t->angle_min) / (t->angle_max - t->angle_min) * (t->counter_max - t->counter_min) + t->counter_min;
    __HAL_TIM_SET_COMPARE(&t->tim, t->tim_channel, pwm_counter);
}
