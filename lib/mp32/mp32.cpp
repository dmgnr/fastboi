#include "mp32.h"

#ifndef NO_MOTOR
#define __M1DIR_OUT0 GPIOB->ODR &= 0x7FFF
#define __M1DIR_OUT1 GPIOB->ODR |= 0x8000
#define __M2DIR_OUT0 GPIOB->ODR &= 0xBFFF
#define __M2DIR_OUT1 GPIOB->ODR |= 0x4000
#define M1DIR PB15
#define M2DIR PB14
#define M1EN PA9
#define M2EN PA8

static bool motorInitialized = false;

void initMotor() {
  motorInitialized = true;
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M1EN, OUTPUT);
  pinMode(M2EN, OUTPUT);
  digitalWrite(M1EN, LOW);
  digitalWrite(M2EN, LOW);
  __M1DIR_OUT0;
  __M2DIR_OUT0;
}

void motor(int powl, int powr) {
  if (!motorInitialized) initMotor();
  if (powl >= 0) __M1DIR_OUT0; else __M1DIR_OUT1;
  if (powr >= 0) __M2DIR_OUT0; else __M2DIR_OUT1;
  analogWrite(M1EN, min((abs(powl) * 255) / 100, 255));
  analogWrite(M2EN, min((abs(powr) * 255) / 100, 255));
}

void motor(int pow) { motor(pow, pow); }
void motor() { motor(0, 0); }
#endif // NO_MOTOR


void OK() {
  if (millis() < 200) delay(200 - millis());
  while (analogRead(9) > 16);
  while (analogRead(9) <= 16);
}

void beep(int time) {
  analogWrite(PB5, 100);
  delay(time);
  analogWrite(PB5, 0);
}
void beep() { beep(100); }

// Optional OLED
#if __has_include(<OLED_I2C_SSD1309.h>) && !defined(NO_OLED)
#include <OLED_I2C_SSD1309.h>
#define OLED_RESET -1
static OLED_I2C_SSD1309 oled(OLED_RESET);
#define OLED
#else
#warning "OLED_I2C_SSD1309.h not found, oled will not be initialized"
#endif

void initVariant() {
#ifndef NO_MOTOR
  initMotor();
#endif
  analogReadResolution(12);
  pinMode(PB5, OUTPUT);

#ifdef OLED
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();
  oled.begin(SSD1309_SWITCHCAPVCC, 0x3C);
  oled.clear();
  oled.show();
#endif

#ifdef EXTRA_MOTORS
  pinMode(9, INPUT);
  pinMode(M3DIR, OUTPUT);
  pinMode(M4DIR, OUTPUT);
  pinMode(M3EN, OUTPUT);
  pinMode(M4EN, OUTPUT);
  digitalWrite(M3EN, LOW);
  digitalWrite(M4EN, LOW);
#endif

#ifndef NO_EXTRA_BUTTONS
  pinMode(PC13, INPUT_PULLUP);
  pinMode(PB2, INPUT_PULLDOWN);
#endif
}
