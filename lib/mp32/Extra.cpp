#include "mp32.h"
#include "Extra.h"

#ifdef EXTRA_MOTORS
const int _OUT0[4] = { 0x7FFF, 0xBFFF, 0xDFFF, 0xEFFF };
const int _OUT1[4] = { 0x8000, 0x4000, 0x2000, 0x1000 };
const int _EN[4]   = { M1EN, M2EN, M3EN, M4EN };

void ao() {
  m(1, 0);
  m(2, 0);
  m(3, 0);
  m(4, 0);
}

void m(int num, int pow) {
  int id = num - 1;
  if (pow >= 0)
    GPIOB->ODR &= _OUT0[id];
  else
    GPIOB->ODR |= _OUT1[id];
  analogWrite(_EN[id], min((abs(pow) * 255) / 100, 255));
}
#endif
