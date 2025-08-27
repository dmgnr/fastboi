asm(".global _printf_float");
#include <MicroPOP32.h>
#include <./Logger.cpp>
#include <QuickPID.h>
#include <pidautotuner.h>
#include <./config.cpp>

PIDLogger logger;
int jccount;
const int sensorCount = sizeof(s) / sizeof(Sensor);
float input = sensorCount - 1, output, setpoint = sensorCount - 1, leftmotor, rightmotor;
QuickPID pid = QuickPID(&input, &output, &setpoint, KP, KI, KD, QuickPID::Action::reverse);

short voltage()
{
    return a(VOLTMETER_PIN) / 2.5;
}

SensorStripLogger sensorLogger(sensorCount);

float line() {
    input = line(s, input);
    return input;
}

void Track() {
    float scale = 1;
    float le = 0;
    long jctime = 0;
    short jc = 0;
    // 1, 2 = start
    // 3, 4 = lap 2
    // 5, 6 = lap 3
    // 7, 8 = lap 4
    // 9    = stop
    while (jc < LAP_COUNT && !ok())
    {
        long next = micros() + 10000;
        line();
        // Scale output based on change in error(Kd)
        float nextscale = min(1.0, 1.2 - abs(le - input) / (float)(sensorCount * 2))
                          / max((double)voltage() / VOLT_BASELINE, 0.1);
        // Straight on junction
        if(input > 100) {
            input = input == 101
                    ? (sensorCount - 1) : input == 103
                    ? (sensorCount * 2 - 4) : 4;
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
        // Acceleration
        if(nextscale > scale) scale+=0.01;
        else if(nextscale < scale-0.05) scale -= 0.05;
        le = (le * 9 + input) / 10.0;
        // Use the PID output to control the motors
        leftmotor = min((SPEED + output) * scale, 100.0f);
        rightmotor = min((SPEED - output) * scale, 100.0f) * RIGHT_OFFSET;
        // the reason I only slowdown motors for offset
        // is that if it was running on max speed and I speed it up
        // it would have no effect, nullifying the offset
        // and causing the robot to go off course

        motor(leftmotor, rightmotor);
        logger.log(input, output, scale, leftmotor, rightmotor);
        delayMicroseconds(next - micros());
    }
}

void tune() {
    PIDAutotuner tuner = PIDAutotuner();
    tuner.setTargetInputValue(sensorCount - 1);
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
                Serial.printf("\nA\u001b[33m%d\u001b[0m\t%d\t%s", i, s[i].get(), s[i].on()?"ON":"\u001b[2mOFF\u001b[22m");
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
    delay(100);
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