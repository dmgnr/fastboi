#include <./Sensor.cpp>

// PID Controller
#define SPEED 75 // Base speed
#define KP 10.0 // Proportional gain
#define KI 0.1 // Integral gain
#define KD 1.0 // Derivative gain
#define RIGHT_OFFSET 1 // Motor voltage imbalance

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

// Voltage
#define VOLTMETER_PIN PB0 // Voltage meter pin
#define VOLT_BASELINE 740 // Voltage in integer

// Sensors
static vector<Sensor> s = {
    Sensor(A0, (1255 + 310) / 2),
    Sensor(A1, (1310 + 371) / 2),
    Sensor(A2, (1190 + 287) / 2),
    Sensor(A3, (1327 + 335) / 2),
    Sensor(A4, (1360 + 347) / 2),
    Sensor(A5, (1343 + 352) / 2),
    Sensor(A6, (1370 + 362) / 2),
    Sensor(A7, (1223 + 344) / 2)
};