asm(".global _printf_float");

int jccount;
#include <MicroPOP32.cpp>
#include <./Logger.cpp>
#include <QuickPID.h>
#include <pidautotuner.h>

PIDLogger logger;

// TODO: ACceleration based on ac/deceleration of error
// TODO: Scale output WITH the speed
// TODO: Momentum acceleration
// TODO: Stopper

#define VOLTMETER_PIN PB0
#define SPEED 80
#define KP 6.5
#define KI 0.02
#define KD 0.25
#define VOLT_BASELINE 762

SensorStripLogger sensorLogger;

float input = 7, output, setpoint = 7, leftmotor, rightmotor;
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

short voltage()
{
    return a(VOLTMETER_PIN) / 2.5;
}

Sensor sensors[] = {
    Sensor(A0, (1666 + 409) / 2),
    Sensor(A1, (1687 + 418) / 2),
    Sensor(A2, (1465 + 371) / 2),
    Sensor(A3, (1510 + 394) / 2),
    Sensor(A4, (1498 + 412) / 2),
    Sensor(A5, (1517 + 414) / 2),
    Sensor(A6, (1425 + 431) / 2),
    Sensor(A7, (1260 + 368) / 2)
};

const int sensorCount = sizeof(sensors) / sizeof(Sensor);

// Temporary solution
float line() {
    bool s0 = sensors[0].on(), s1 = sensors[1].on(), s2 = sensors[2].on(), s3 = sensors[3].on(), s4 = sensors[4].on(), s5 = sensors[5].on(), s6 = sensors[6].on(), s7 = sensors[7].on();
    if (false);
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
    else if( s0 &&  s1 &&  s2 &&  s3 &&  s4 && !s5 &&  s6 &&  s7) input =  10;
    else if( s0 &&  s1 &&  s2 &&  s3 &&  s4 && !s5 && !s6 &&  s7) input =  11;
    else if( s0 &&  s1 &&  s2 &&  s3 &&  s4 &&  s5 && !s6 &&  s7) input =  12;
    else if( s0 &&  s1 &&  s2 &&  s3 &&  s4 &&  s5 && !s6 && !s7) input =  13;
    else if( s0 &&  s1 &&  s2 &&  s3 &&  s4 &&  s5 &&  s6 && !s7) input =  14;
    else if(!s0 && !s1 && !s2 && !s3 && !s4 && !s5 && !s6 && !s7) input = 101;
    else if(!s0 && !s1 && !s2 && !s3 && !s4 && !s5 && !s6 &&  s7) input = 102;
    else if( s0 && !s1 && !s2 && !s3 && !s4 && !s5 && !s6 && !s7) input = 103;
    // Last known position = don't set anything
    return input;
}

void Track() {
    float scale = 1;
    int le = 0;
    long jctime = 0;
    short jc = 0;
    // 1, 2 = start
    // 3, 4 = lap 2
    // 5, 6 = lap 3
    // 7, 8 = lap 4
    // 9    = stop
    while (jc<9 && !ok())
    {
        long next = micros() + 10000;
        line();
        // Scale output based on change in error(Kd)
        float nextscale = min(1.0, 1.2 - abs(le - input) / 14.0) / max((double)voltage() / VOLT_BASELINE, 0.1); // 14 is the max error
        // Straight on junction
        if(input > 100) {
            input = input == 101
                    ? 7 : input == 103
                    ? 10 : 4;
            if(!jctime)
                jctime = millis();
            nextscale = 2; // robot go brrrr.
        }
        else if (jctime && millis() - jctime > 200)
        {
            jctime = 0;
            jc++;
        }
        else if (jctime)
            jctime = 0;
        pid.Compute();
        if(nextscale > scale) scale+=0.01;
        else if(nextscale < scale-0.05) scale -= 0.05;
        le = input;
        // Use the PID output to control the motors
        leftmotor = (SPEED + output) * scale;
        rightmotor = (SPEED - output) * scale * 0.93;
        motor(leftmotor, rightmotor);
        logger.log(input, output, scale, leftmotor, rightmotor);
        delayMicroseconds(next - micros());
    }
}

void tune() {
    PIDAutotuner tuner = PIDAutotuner();
    tuner.setTargetInputValue(7.0);
    tuner.setLoopInterval(1000);
    tuner.setOutputRange(-SPEED, SPEED);
    tuner.startTuningLoop(micros());
    while (!tuner.isFinished())
    {
        long us = micros();
        long next = us + 1000;
        float output = tuner.tunePID(line(), us);
        motor(SPEED + output, SPEED - output);
        delayMicroseconds(next - micros());
    }
    motor(0, 0);
    while(1) {
        while(!ok());
        beep();
        Serial.printf("Tuning complete: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", tuner.getKp(), tuner.getKi(), tuner.getKd());
        while(ok());
    }
}

void setup() {
    Serial.begin(115200);
    pid.SetMode(QuickPID::Control::automatic);
    pid.SetSampleTimeUs(20000); // Set sample time to 20ms
    pid.SetOutputLimits(-255, 255); // Set output limits to motor speed range
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
            Serial.printf("\u001bcPow: %.2fV\tLine: %d\nOut: %.2f\n\u001b[36mPin\tRaw\tStt\u001b[0m", voltage()/100.0, (int)input, output);
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
    // while(1) {
    //     motor((sin(millis()/1000.0)+1)*50);
    //     delay(16);
    // }
    Track();
    //  tune();
    motor(SPEED, SPEED);
    delay(50);
    motor(0, 0);
    while (ok())
        ;
    while (1)
    {
        while (!ok())
            ;
        beep();
        logger.dump();
        while (ok())
            ;
    }
}

void loop() {
}