#include <Servo.h>

// global flag
bool error = false;

#define DEBUG true // print debug msg?

#define DELAY 500

// for servo
#define ROT_PIN 8
#define BASE_PIN 7

#define BASE_MIN_ANGLE 0
#define BASE_MAX_ANGLE 180

#define DELAY 500

#define ROT_INIT_ANGLE 88
#define ROT_STEP 15
#define BASE_INIT_ANGLE 0
#define BASE_STEP 30

Servo baseServo, rotServo;

int curBaseAngle = BASE_INIT_ANGLE;
int curRotAngle = ROT_INIT_ANGLE;

#define TRIG_PIN1 13
#define ECHO_PIN1 12
#define TRIG_PIN2 11
#define ECHO_PIN2 10

float getDist(int type)
{
    long duration;
    float distance;

    if (type == 1)
    {
        digitalWrite(TRIG_PIN1, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN1, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN1, LOW);
        duration = pulseIn(ECHO_PIN1, HIGH, 30000); // 타임아웃 30ms, 최대 약 5m
        distance = duration * 0.0343 / 2.0;         // cm 단위 거리 계산
    }
    else if (type == 2)
    {
        digitalWrite(TRIG_PIN2, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN2, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN2, LOW);
        duration = pulseIn(ECHO_PIN2, HIGH, 30000);
        distance = duration * 0.0343 / 2.0;
    }
    else
    {
        Serial.println("Error!");
        distance = -1;
    }

    return distance;
}

void setup()
{
    Serial.begin(9600);

    // Initialize Sonar Sensor 1
    pinMode(TRIG_PIN1, OUTPUT);
    pinMode(ECHO_PIN1, INPUT);

    // Initialize Sonar Sensor 2
    pinMode(TRIG_PIN2, OUTPUT);
    pinMode(ECHO_PIN2, INPUT);
}

void commonDelay()
{
    delay(DELAY);
}

void loop()
{
    // rotServo.write(ROT_INIT_ANGLE);
    // commonDelay();
    rotServo.write(ROT_INIT_ANGLE - ROT_STEP);
    commonDelay();
    float frontDist = getDist(1);

    rotServo.write(ROT_INIT_ANGLE);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE + ROT_STEP);
    commonDelay();
    float backDist = getDist(2);

    rotServo.write(ROT_INIT_ANGLE);

    Serial.print("Sensor 1 Front dist: ");
    Serial.print(frontDist);
    Serial.println(" cm");

    Serial.print("Sensor 2 Front dist: ");
    Serial.print(backDist);
    Serial.println(" cm");

    float distance1 = getDist(1);
    float distance2 = getDist(2);

    Serial.print("Sensor 1 dist: ");
    Serial.print(distance1);
    Serial.println(" cm");

    Serial.print("Sensor 2 dist: ");
    Serial.print(distance2);
    Serial.println(" cm");

    delay(1000);
}