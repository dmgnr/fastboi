#include <mp32.h>
#include <stdlib.h>
#include <vector>
#include <numeric>
using namespace std;

class Sensor {
    short pin;
    short value;
    short ref;

    long cacheTime = 0;
    short upperRef = 0;
    short lowerRef = 0;

    void setRef() {
        if(upperRef == 0 || lowerRef == 0) return; // No reference set
        ref = (upperRef + lowerRef) / 2;
    }
    void store() {
        if (micros() < cacheTime) return; // Avoid reading too frequently
        value = a(pin);
        cacheTime = micros() + 200;
    }

public:
    Sensor(short p, short ref) : pin(p), value(0), ref(ref) {
        store();
    }
    short get() {
        store();
        return value;
    }
    bool on() {
        store();
        return value > ref || value < 100; // Threshold for "on" state OR robot being picked up
    }
    void setUpperRef() {
        store();
        upperRef = value;
        setRef();
    }
    void setLowerRef() {
        store();
        lowerRef = value;
        setRef();
    }
};

#ifndef LINE_FN
#define LINE_FN

// Weighted average
static float line(vector<Sensor> s, float inp, bool flipped, int ambiguity) {
    const int count = s.size();
    vector<int> active;
    for (int i = 0; i < count; i++) if(flipped - s.at(i).on())
            active.push_back(max(i * 2 - 1, 0));
    // If robot is on black line, return flag 1
    if(active.size() == count)
        return 101;
    // If it's potentially ambiguous, don't do anything
    if(active.size() > ambiguity || active.size() == 0)
        return inp;
    inp = accumulate(active.begin(), active.end(), 0.0) / active.size();
    return inp;
}

#endif