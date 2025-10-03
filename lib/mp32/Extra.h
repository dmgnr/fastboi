#pragma once

#ifdef EXTRA_MOTORS
#ifndef NO_MOTOR
#error "Extra motors module cannot be used when motor control is disabled"
#endif

#define M3DIR PB13
#define M4DIR PB12
#define M3EN  PA10
#define M4EN  PB4

void ao();
void m(int num, int pow);

#endif // EXTRA_MOTORS


#ifdef EXTRA_ALIASES
#define a   analogRead
#define d   digitalRead
#define p   pinMode
#define o   digitalWrite
#define b   beep
#define sl(x) motor(-x, x)
#define sr(x) motor(x, -x)
#define tl(x) motor(0, x)
#define tr(x) motor(x, 0)
#define fd   motor
#define fd2  motor
#define bk(x) motor(-x, -x)
#endif


#ifndef NO_EXTRA_BUTTONS
#define ok()     (analogRead(9) < 16)
#define sw_A()   !digitalRead(PC13)
#define sw_B()    digitalRead(PB2)

#define wait_OK()  while (!ok())   { delay(1); }
#define wait_A()   while (!sw_A()) { delay(1); }
#define wait_B()   while (!sw_B()) { delay(1); }
#endif
