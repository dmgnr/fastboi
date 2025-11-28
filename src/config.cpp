#include <./Sensor.cpp>

// PID Controller
// #define SPEED 40 // Base speed
// #define KP 5.0 // Proportional gain
// #define KI 0.1 // Integral gain
// #define KD 0.0 // Derivative gain
// #define RIGHT_OFFSET 1 // Motor voltage imbalance
// #define ACCELERATION 0.005 // Acceleration rate
// #define BRAKE_INTENSITY 1.0 // Acceleration braking intensity
// #define BRAKE_LIMIT 0.3 // Minimum speed factor when braking
#define SPEED 75 // Base speed
#define KP 12.0 // Proportional gain
#define KI 0.1 // Integral gain
#define KD 0.5 // Derivative gain
#define RIGHT_OFFSET 1 // Motor voltage imbalance
#define ACCELERATION 0.005 // Acceleration rate
#define BRAKE_INTENSITY 1.0 // Acceleration braking intensity
#define BRAKE_LIMIT 0.5 // Minimum speed factor when braking

// Game
// This lap count algorithm counts everytime
//  the robot crosses a large blind junction
// 1, 2 = start
// 3, 4 = lap 2
// 5, 6 = lap 3
// 7, 8 = lap 4
// 9    = stop
#define LAP_COUNT 9 // Number of laps
#define PICK_STOP true // Stop on pick up

// Line calculation
#define FLIPPED true // true = white field
#define AMBIGUITY 3 // Line ambiguity
#define JC_TIMEOUT 200 // Junction confirmation time in ms
#define DEADZONE 1 // Deadzone around center position

// Voltage
#define VOLTMETER_PIN PB0 // Voltage meter pin
#define VOLT_BASELINE 740 // Voltage in integer

// Sensors
static vector<Sensor> s = {
    Sensor(A0, (1871 + 263) / 2),
    Sensor(A1, (1733 + 265) / 2),
    Sensor(A2, (1552 + 155) / 2),
    Sensor(A3, (1624 + 218) / 2),
    Sensor(A4, (1648 + 255) / 2),
    Sensor(A5, (1635 + 255) / 2),
    Sensor(A6, (1594 + 337) / 2),
    Sensor(A7, (1517 + 254) / 2)
};