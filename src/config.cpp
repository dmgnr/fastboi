#include <./Sensor.cpp>

// PID Controller
#define SPEED 75 // Base speed
#define KP 20.0 // Proportional gain
#define KI 1.0 // Integral gain
#define KD 3.0 // Derivative gain
#define RIGHT_OFFSET 0.91 // Motor voltage imbalance

// Game
#define LAP_COUNT 9 // Number of laps

// Line calculation
#define FLIPPED true // true = white field
#define AMBIGUITY 3 // Line ambiguity

// Voltage
#define VOLTMETER_PIN PB0 // Voltage meter pin
#define VOLT_BASELINE 500 // Voltage in integer

// Sensors
static vector<Sensor> s = {
    Sensor(A0, (2508 + 782) / 2),
    Sensor(A1, (1777 + 587) / 2),
    Sensor(A2, (1436 + 501) / 2),
    Sensor(A3, (1568 + 522) / 2),
    Sensor(A4, (1738 + 394) / 2),
    Sensor(A5, (2255 + 583) / 2),
    Sensor(A6, (1669 + 286) / 2),
    Sensor(A7, (1384 + 359) / 2)
};