#include "algorithm/pid.h"

/* 这个文件里没有任何全局/静态变量 —— 状态全在调用者的 Pid_t 里，
 * 所以它是可重入的：中断里在跑直立环的同时，别处用另一个 Pid_t 跑转向环也不会打架。 */

/* 对称限幅：把 v 压进 [-lim, +lim]。lim <= 0 视为"不限幅"（调用方保证不这么用） */
static float pid_clamp(float v, float lim)
{
    if (lim <= 0.0f)
    {
        return v;
    }
    if (v > lim)
    {
        return lim;
    }
    if (v < -lim)
    {
        return -lim;
    }
    return v;
}

void Pid_Reset(Pid_t *pid)
{
    if (pid == 0)
    {
        return;
    }
    pid->integral      = 0.0f;
    pid->prev_measure  = 0.0f;
    pid->prev_setpoint = 0.0f;
    pid->primed        = 0U;
}

void Pid_Init(Pid_t *pid, float kp, float ki, float kd,
              float out_max, float integ_max, uint8_t d_on_measure)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    /* 限幅值只接受正数：传负数/0 时统一按"不限幅"处理，避免写出 lim<0 的诡异分支 */
    pid->out_max   = (out_max   > 0.0f) ? out_max   : 0.0f;
    pid->integ_max = (integ_max > 0.0f) ? integ_max : 0.0f;

    pid->d_on_measure = (d_on_measure != 0U) ? 1U : 0U;

    Pid_Reset(pid);
}

float Pid_Update(Pid_t *pid, float setpoint, float measure, float dt)
{
    float rate = 0.0f;

    if (pid == 0)
    {
        return 0.0f;
    }

    /* 首拍没有"上一拍"，差不出速率：给 0，让核心函数本拍不出微分项。
     * dt 不合法时同样给 0 —— 除以 0 会得到 inf，喂进 D 项就是满占空比的一脚。 */
    if ((pid->primed != 0U) && (dt > 0.0f) && (dt < PID_DT_MAX_S))
    {
        rate = (measure - pid->prev_measure) / dt;
    }

    return Pid_UpdateWithRate(pid, setpoint, measure, rate, dt);
}

float Pid_UpdateWithRate(Pid_t *pid, float setpoint, float measure, float rate, float dt)
{
    float err;
    float p;
    float d;
    float i_keep;
    float out;
    uint8_t dt_ok;

    if (pid == 0)
    {
        return 0.0f;
    }

    err = setpoint - measure;

    /* ① 比例项：唯一的"当前信息"，永远参与 */
    p = pid->kp * err;

    /* dt 合法性：<=0 不能除也不能积；>= PID_DT_MAX_S 说明节拍早就丢了，
     * 用这种 dt 算出来的微分/积分都是假数。两种都只发 P 项。 */
    dt_ok = ((dt > 0.0f) && (dt < PID_DT_MAX_S)) ? 1U : 0U;

    /* ② 微分项：首拍不出（没有历史就没法微分），dt 非法也不出 */
    d = 0.0f;
    if ((pid->primed != 0U) && (dt_ok != 0U))
    {
        if (pid->d_on_measure != 0U)
        {
            /* 设定值变化不进入 D 项 → 没有 derivative kick。
             * 负号：测量值上升时 D 项要"往回拉"，符号必须和 P 项相反才叫阻尼。 */
            d = -pid->kd * rate;
        }
        else
        {
            /* 对误差差分 = 设定值变化率 - 测量值变化率。设定值常量的场合，
             * 两种写法等价；设定值会跳时，这一种会把跳变算进 D 项。 */
            float setpoint_rate = (setpoint - pid->prev_setpoint) / dt;
            d = pid->kd * (setpoint_rate - rate);
        }
    }

    /* ③ 积分项：clamp 抗饱和 + 条件积分 */
    i_keep = pid->integral;
    if ((dt_ok != 0U) && (pid->ki != 0.0f))
    {
        float i_next;
        float out_next;

        i_next = pid->integral + pid->ki * err * dt;

        /* 先钳位（integ_max 是物理上限，不是"建议值"） */
        if (pid->integ_max > 0.0f)
        {
            i_next = pid_clamp(i_next, pid->integ_max);
        }

        out_next = p + d + i_next;

        /* 条件积分：这一拍的总输出已经顶到上限，而且误差还在往同一边推
         * → 本拍不接受累积，直接沿用原积分。
         * 注意判据是"同一边"，不是"只要饱和就停"：饱和时如果误差反了，
         * 积分必须能反向退出来，否则会长时间卡在饱和里。 */
        if ((pid->out_max > 0.0f) &&
            (((out_next >  pid->out_max) && (err > 0.0f)) ||
             ((out_next < -pid->out_max) && (err < 0.0f))))
        {
            i_next = pid->integral;
        }

        i_keep = i_next;
    }
    pid->integral = i_keep;

    /* ④ 求和 + 输出限幅 */
    out = p + d + i_keep;
    out = pid_clamp(out, pid->out_max);

    /* 只在拿到有效 dt 时才记录历史：dt 非法的那一拍不更新，
     * 免得下一拍拿着"跨了两拍的时间"去算速率，把 D 项算小一半。 */
    if (dt_ok != 0U)
    {
        pid->prev_measure  = measure;
        pid->prev_setpoint = setpoint;
        pid->primed        = 1U;
    }

    return out;
}
