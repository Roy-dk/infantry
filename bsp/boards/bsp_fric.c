#include "bsp_fric.h"
#include "main.h"
extern TIM_HandleTypeDef htim1;
int16_t fric_speed_pwm = 1320;

extern int16_t *get_fric_speed_pwm_lp()
{
    return &fric_speed_pwm;
}

void fric_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, FRIC_OFF);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, FRIC_OFF);
}
void fric1_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, cmd);
}
void fric2_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, cmd);
}

