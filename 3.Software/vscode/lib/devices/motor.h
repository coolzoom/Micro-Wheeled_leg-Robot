#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "SimpleFOC.h"
#include "bus/i2c_bus.h"

extern BLDCMotor left_motor;
extern BLDCMotor right_motor;

void motor_init(void);

#endif
