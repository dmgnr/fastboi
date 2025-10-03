#pragma once
#include <Arduino.h>
#include <Wire.h>

#ifndef POP64
#define POP64

#include "Extra.h"

#define a(pin) analogRead(pin)

// Motor control
#ifndef NO_MOTOR
void initMotor();
void motor(int powl, int powr);
void motor(int pow);
void motor();
#endif

// Utility
void OK();
void beep(int time);
void beep();

// Variant init
void initVariant();

#endif // POP64
