#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>

long count = 0;

long posDB = 146;
long negDB = -176;

BlueMotor::BlueMotor()
{
}

void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    // Question 1 - Quadrature Encoders
    attachInterrupt(digitalPinToInterrupt(ENCA), isrENCA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), isrENCB, CHANGE);
    reset();
}

long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}

// Question 1 - Quadrature Encoders
void BlueMotor::isrENCA()
{
    if (digitalRead(ENCA) == digitalRead(ENCB)) {
        count--;
    } else {
        count++;
    }
}

// Question 1 - Quadrature Encoders
void BlueMotor::isrENCB()
{
    if (digitalRead(ENCA) == digitalRead(ENCB)) {
        count++;
    } else {
        count--;
    }
}

void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400);
}

// Question 2 - moveTo() function
void BlueMotor::moveTo(long target)
{
    float Kp = 0.85;
    // proportional control
    long setEffortInt = Kp * (target - count);
    setEffortWithoutDB(setEffortInt);
    // if under tolerance, turn motor off
    if (setEffortInt < tolerance) {
        setEffort(0);
    }
}

// Question 3b - setEffortWithoutDB
void BlueMotor::setEffortWithoutDB(long effort) {
    if (effort >= 0) {
        //posDB is 146
        Serial.println((int)(posDB + (effort / 400.0) * (400 - abs(posDB))));
        setEffort((int)(posDB + (effort / 400.0) * (400 - abs(posDB))));
    }
    else if (effort < 0) {
        //negDB is -176
        Serial.println((int)(negDB + (effort / 400.0) * (400 - abs(negDB))));
        setEffort((int)(negDB + (effort / 400.0) * (400 - abs(negDB))));
    }
}