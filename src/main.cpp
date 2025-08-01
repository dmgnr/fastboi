asm(".global _printf_float");

int jccount;
#include <MicroPOP32.cpp>
#include <./Logger.cpp>
#include <QuickPID.h>

#define VOLTMETER_PIN PB0
#define SPEED 80
#define KP 1.0
#define KI 0.1
#define KD 0.5

SensorStripLogger sensorLogger;

float input, output, setpoint = 7, leftmotor, rightmotor;
QuickPID pid = QuickPID(&input, &output, &setpoint, KP, KI, KD, QuickPID::Action::reverse);

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
        return value > ref; // Threshold for "on" state
    }
    void setUpperRef(short upper) {
        upperRef = upper;
        setRef();
    }
    void setLowerRef(short lower) {
        lowerRef = lower;
        setRef();
    }
};

short voltage()
{
    return a(VOLTMETER_PIN) / 2.5;
}

Sensor sensors[] = {
    Sensor(A0, 2750),
    Sensor(A1, 2874),
    Sensor(A2, 2245),
    Sensor(A3, 2996),
    Sensor(A4, 2146),
    Sensor(A5, 3036),
    Sensor(A6, 2559),
    Sensor(A7, 3047)
};

const int sensorCount = sizeof(sensors) / sizeof(Sensor);

float line() {
    bool s0 = sensors[0].on(), s1 = sensors[1].on(), s2 = sensors[2].on(), s3 = sensors[3].on(), s4 = sensors[4].on(), s5 = sensors[5].on(), s6 = sensors[6].on(), s7 = sensors[7].on();
    if(false);
    else if(!s0 &&  s1 &&  s2 &&  s3 &&  s4 &&  s5 &&  s6 &&  s7) input =   0;
    else if(!s0 && !s1 &&  s2 &&  s3 &&  s4 &&  s5 &&  s6 &&  s7) input =   1;
    else if( s0 && !s1 &&  s2 &&  s3 &&  s4 &&  s5 &&  s6 &&  s7) input =   2;
    else if( s0 && !s1 && !s2 &&  s3 &&  s4 &&  s5 &&  s6 &&  s7) input =   3;
    else if( s0 &&  s1 && !s2 &&  s3 &&  s4 &&  s5 &&  s6 &&  s7) input =   4;
    else if( s0 &&  s1 && !s2 && !s3 &&  s4 &&  s5 &&  s6 &&  s7) input =   5;
    else if( s0 &&  s1 &&  s2 && !s3 &&  s4 &&  s5 &&  s6 &&  s7) input =   6;
    else if( s0 &&  s1 &&  s2 && !s3 && !s4 &&  s5 &&  s6 &&  s7) input =   7;
    else if( s0 &&  s1 &&  s2 &&  s3 && !s4 &&  s5 &&  s6 &&  s7) input =   8;
    else if( s0 &&  s1 &&  s2 &&  s3 && !s4 && !s5 &&  s6 &&  s7) input =   9;
    else if( s0	&&  s1 &&  s2 &&  s3 &&  s4	&& !s5 &&  s6 &&  s7) input =  10;
    else if( s0	&&  s1 &&  s2 &&  s3 &&  s4	&& !s5 && !s6 &&  s7) input =  11;
    else if( s0	&&  s1 &&  s2 &&  s3 &&  s4	&&  s5 && !s6 &&  s7) input =  12;
    else if( s0	&&  s1 &&  s2 &&  s3 &&  s4	&&  s5 && !s6 && !s7) input =  13;
    else if( s0	&&  s1 &&  s2 &&  s3 &&  s4	&&  s5 &&  s6 && !s7) input =  14;
    else if(!s0	&& !s1 && !s2 && !s3 && !s4	&& !s5 && !s6 && !s7) input = 101;
    // Last known position = don't set anything
    return input;
}

void Track(int time) {
    const int end = millis() + time;
    while (millis() < end) {
        line();
        // Straight on junction
        if(input == 101) {
            motor(SPEED);
            continue;
        }
        pid.Compute();
        // Use the PID output to control the motors
        leftmotor = SPEED + output;  // Adjust left motor speed
        rightmotor = SPEED - output; // Adjust right motor speed
        motor(leftmotor, rightmotor);
    }
}

void setup() {
    Serial.begin(115200);
    pid.SetMode(QuickPID::Control::automatic);
    pid.SetSampleTimeUs(20000); // Set sample time to 20ms
    pid.SetOutputLimits(-SPEED, SPEED); // Set output limits to motor speed range
    if(sw_A()) {
        beep(1000);
        while(sw_A())
            ;
        while (!sw_A())
        {
            if(ok()) {
                short sensorVals[sensorCount];
                for (int i = 0; i < sensorCount; i++) {
                    sensorVals[i] = a(i);
                }
                sensorLogger.log(sensorVals);
                beep(100);
                while(ok())
                    ;
            }
        }
        while(ok());
        beep(500);
        while (1)
        {
            while (!ok())
                delay(20);
            beep();
            Serial.print("\u001bcSensor Dump:\n\n");
            sensorLogger.dump();
            while (ok())
                delay(20);
        }
    }
    if (ok())
    {
        beep(200);
        delay(100);
        beep(200);
        while (1)
        {
            line();
            if(input != 101) pid.Compute();
            Serial.printf("\u001bcPow: %.2fcV\tLine: %d\nOut: %.2f\n\u001b[36mPin\tRaw\tStt\u001b[0m", voltage()/100.0, (int)input, output);
            for (int i = 0; i < sensorCount; i++)
            {
                Serial.printf("\nA\u001b[33m%d\u001b[0m\t%d\t%s", i, sensors[i].get(), sensors[i].on()?"ON":"\u001b[2mOFF\u001b[22m");
            }
            delayMicroseconds(33333); // 30 FPS
        }
    }
    beep();
    while(!ok());
    beep();
    while (ok());
    delay(1000);
    Track(2000);
    motor(0, 0);
}

void loop() {
}