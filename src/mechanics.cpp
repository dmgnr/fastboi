#include <Arduino.h>
#include <Servo.h>

class Compartment {
public:
    Compartment() : isOpen(false), isInitialized(false), compartmentServo() {}

    void initialize(int pin = PB10)
    {
        if (!isInitialized)
        {
            compartmentServo.attach(pin);
            isInitialized = true;
        }
    }

    void open() {
        initialize();
        compartmentServo.write(90); // Open position
        isOpen = true;
    }

    void close() {
        initialize();
        compartmentServo.write(0); // Closed position
        isOpen = false;
    }

    bool getState() const {
        return isOpen;
    }

private:
    bool isOpen;
    bool isInitialized;
    Servo compartmentServo;
};