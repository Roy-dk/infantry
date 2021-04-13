#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#define FRIC_UP 1400
#define FRIC_DOWN 1320//测速
#define FRIC_OFF 1000

extern int16_t* get_fric_speed_pwm_lp();
extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
