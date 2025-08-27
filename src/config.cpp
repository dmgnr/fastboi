#include <./Sensor.cpp>

// PID Controller
#define SPEED 100 // Base speed
#define KP 14.5 // Proportional gain
#define KI 0.5 // Integral gain
#define KD 0.6 // Derivative gain
#define RIGHT_OFFSET 0.91 // Motor voltage imbalance

// Game
#define LAP_COUNT 9 // Number of laps

// Voltage
#define VOLTMETER_PIN PB0 // Voltage meter pin
#define VOLT_BASELINE 762 // Voltage in integer

// Sensors
Sensor s[] = {
    Sensor(A0, (1666 + 409) / 2),
    Sensor(A1, (1687 + 418) / 2),
    Sensor(A2, (1465 + 371) / 2),
    Sensor(A3, (1510 + 394) / 2),
    Sensor(A4, (1498 + 412) / 2),
    Sensor(A5, (1517 + 414) / 2),
    Sensor(A6, (1425 + 431) / 2),
    Sensor(A7, (1260 + 368) / 2)
};