#include <Arduino.h>
#ifndef FILTER_H
#define FILTER_H

class PT1 {
public:
    PT1(float tau = 0.02f) : tau(tau), y(0), initialized(false), lastMicros(0) {}

    float filter(float x) {
        unsigned long now = micros();
        float dt;

        if (!initialized) {
            initialized = true;
            y = x;
            lastMicros = now;
            return y;
        }

        dt = (now - lastMicros) * 1e-6f;
        lastMicros = now;

        float a = dt / (tau + dt);
        y += a * (x - y);
        return y;
    }

private:
    float tau;  // smaller = faster response
    float y;
    bool initialized;
    unsigned long lastMicros;
};


#endif
