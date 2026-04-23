#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include <stdint.h>

void RobotApp_EnableCycleCounter(void);
void RobotApp_Init(void);
void RobotApp_Tick(void);
void RobotApp_OnGpioExti(uint16_t gpio_pin);

#endif /* ROBOT_APP_H */
