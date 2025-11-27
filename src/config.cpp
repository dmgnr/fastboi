#include <./Sensor.cpp>

// PID Controller
#define SPEED 75 // Base speed
#define KP 12.0 // Proportional gain
#define KI 0.1 // Integral gain
#define KD 0.5 // Derivative gain
#define RIGHT_OFFSET 1 // Motor voltage imbalance
#define ACCELERATION 0.002 // Acceleration rate
#define BRAKE_INTENSITY 1.0 // Acceleration braking intensity

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
    Sensor(A0, (1796 + 279) / 2),
    Sensor(A1, (1723 + 363) / 2),
    Sensor(A2, (1628 + 278) / 2),
    Sensor(A3, (1706 + 303) / 2),
    Sensor(A4, (1845 + 298) / 2),
    Sensor(A5, (1836 + 311) / 2),
    Sensor(A6, (1865 + 304) / 2),
    Sensor(A7, (1639 + 253) / 2)
};