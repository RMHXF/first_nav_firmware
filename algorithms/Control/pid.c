#include "pid.h"
//#include "common.h"
/**
***********************************************************************
* @brief:      pid_para_init()
* @param[in]:	 pid_config  指向pid_para_t结构体的指针
* @param[in]:	 kp  比例系数
* @param[in]:	 ki  积分系数
* @param[in]:	 kd  微分系数
* @retval:     void
* @details:    初始化PID控制参数
***********************************************************************
**/
void pid_para_init(pid_para_t *pid_config)
{
	pid_config->kp = 0.0f;
	pid_config->ki = 0.0f;
	pid_config->kd = 0.0f;

	pid_config->p_term = 0.0f;
	pid_config->i_term = 0.0f;
	pid_config->d_term = 0.0f;
	pid_config->ref_value = 0.0f;
	pid_config->fback_value = 0.0f;
	pid_config->error = 0.0f;
	pid_config->pre_err = 0.0f;
	pid_config->out_value = 0.0f;
}
/**
***********************************************************************
* @brief:      pid_limit_init
* @param[in]:	 pid_config  指向pid_para_t结构体的指针
* @param[in]:	 i_term_max  积分项最大值
* @param[in]:	 i_term_min  积分项最小值
* @param[in]:	 out_max  输出最大值
* @param[in]:	 out_min  输出最小值
* @retval:     void
* @details:    设置PID控制器的输出限制范围
***********************************************************************
**/
void pid_limit_init(pid_para_t *pid_config, float i_term_max, float i_term_min,float out_max, float out_min)
{
	pid_config->i_term_max = i_term_max;
	pid_config->i_term_min = i_term_min;
	pid_config->out_max = out_max;
	pid_config->out_min = out_min;
}
/**
***********************************************************************
* @brief:      pid_clear
* @param[in]:	 pid_clear  指向pid_para_t结构体的指针
* @retval:     void
* @details:    清除PID控制器的历史状态
***********************************************************************
**/
void pid_clear(pid_para_t *pid_clear)
{
	pid_clear->p_term = 0.0f;
	pid_clear->i_term = 0.0f;
	pid_clear->d_term = 0.0f;
	pid_clear->error = 0.0f;
	pid_clear->pre_err = 0.0f;
	pid_clear->out_value = 0.0f;
}
/**
***********************************************************************
* @brief:      pid_reset()
* @param[in]:	 pid_config  指向pid_para_t结构体的指针
* @param[in]:	 kp  比例系数
* @param[in]:	 ki  积分系数
* @param[in]:	 kd  微分系数
* @retval:     void
* @details:    PID参数在线修改
***********************************************************************
**/
void pid_reset(pid_para_t *pid_config, float kp, float ki, float kd)
{
	pid_config->kp = kp;
	pid_config->ki = ki;
	pid_config->kd = kd;
}
/**
***********************************************************************
* @brief:     	pid_ctrl
* @param[in]:	pid  指向pid_para_t结构体的指针
* @param[in]:	ref_value  目标值
* @param[in]:	fback_value  实际值
* @retval:   	float  PID控制器的输出值
* @details:  	实现PID控制器的计算逻辑
***********************************************************************
**/
float parallel_pid_ctrl(pid_para_t *pid, float ref_value, float fdback_value)
{
	pid->ref_value = ref_value;
	pid->fback_value = fdback_value;

	pid->error = pid->ref_value - pid->fback_value;

	// 计算比例项
	pid->p_term = pid->kp * pid->error;

	// 计算积分项
	pid->i_term += pid->ki * pid->error;

	// 限制积分项范围
	if (pid->i_term > pid->i_term_max)
		pid->i_term = pid->i_term_max;
	else if (pid->i_term < pid->i_term_min)
		pid->i_term = pid->i_term_min;

	// 计算微分项
	pid->d_term = pid->kd * (pid->error - pid->pre_err);
	pid->pre_err = pid->error;

	// 计算总输出（P、I、D项）
	pid->out_value = pid->p_term + pid->i_term + pid->d_term;

	// 限制输出范围
	if (pid->out_value > pid->out_max)
		pid->out_value = pid->out_max;
	else if (pid->out_value < pid->out_min)
		pid->out_value = pid->out_min;

	return pid->out_value;
}


float serial_pid_ctrl(pid_para_t *pid, float ref_value, float fdback_value)
{
	pid->ref_value = ref_value;
	pid->fback_value = fdback_value;

	pid->error = pid->ref_value - pid->fback_value;

	pid->p_term = pid->kp * pid->error;

	pid->i_term += pid->ki * pid->p_term;

	if (pid->i_term > pid->i_term_max)
		pid->i_term = pid->i_term_max;
	else if (pid->i_term < pid->i_term_min)
		pid->i_term = pid->i_term_min;

	pid->out_value = pid->p_term + pid->i_term;


	if (pid->out_value > pid->out_max)
		pid->out_value = pid->out_max;
	else if (pid->out_value < pid->out_min)
		pid->out_value = pid->out_min;

	return pid->out_value;
}
