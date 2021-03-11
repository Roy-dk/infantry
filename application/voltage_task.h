/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       voltage_task.c/h
  *             percentage.24电源电压ADC任务,获取电压并且计算电量百分比.
  *             当电源不直连开发板,请修改VOLTAGE_DROP
  */
#ifndef VOLTAGE_TASK_H
#define VOLTAGE_TASK_H
#include "struct_typedef.h"

/**
  * @brief          电源采样和计算电源百分比
  * @param[in]      pvParameters: NULL
  */
extern void battery_voltage_task(void const *argument);

/**
  * @brief          获取电量
  * @param[in]      void
  * @retval         电量, 单位 1, 1 = 1%
  */
extern uint16_t get_battery_percentage(void);
#endif
