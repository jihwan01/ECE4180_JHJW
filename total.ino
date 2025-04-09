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

#define ROT_INIT_ANGLE 88
#define ROT_STEP 15
#define BASE_INIT_ANGLE 0
#define BASE_STEP 30

Servo baseServo, rotServo;

int curBaseAngle = BASE_INIT_ANGLE;
int curRotAngle = ROT_INIT_ANGLE;

// for sonar
#define TRIG_PIN1 13
#define ECHO_PIN1 12
#define TRIG_PIN2 11
#define ECHO_PIN2 10
#define NUM_DIST 360 / BASE_STEP

#define CLIFF_THRESHOLD 100

float dist[NUM_DIST]; // 0~6 : left angle, 7~13 : right angle
float frontDist;
float backDist;

int movingStatus = 1; // 1 : can go forward 2 : can't go forward, can go backward, 3 : can't go forward and backward

// for motor
#define LEFT_SPEED_PIN 6
#define LEFT_DIR_PIN1 22
#define LEFT_DIR_PIN2 23

#define RIGHT_SPEED_PIN 9
#define RIGHT_DIR_PIN1 24
#define RIGHT_DIR_PIN2 25

#define STBY_PIN 27 // LOW : OFF Motor Driver, HIGH : ON Motor Driver

#define MOTOR_SPEED 100
#define ROTATION_TIME_PER_DEGREE 5

void setup()
{
    Serial.begin(9600); // Set rate

    // Initialize Status
    initAll();
}

// Initialize All
void initAll()
{
    initServo();
    initSonar();
    initLoadCell();
    initLCD();
    initMotor();
    initVC();
}

// Initialize servo status
void initServo()
{
    // Binding pin and object
    baseServo.attach(BASE_PIN);
    rotServo.attach(ROT_PIN);

    // Initialize global variable
    curBaseAngle = BASE_INIT_ANGLE;
    curRotAngle = ROT_INIT_ANGLE;

    // Initialize Servo Location
    baseServo.write(BASE_INIT_ANGLE);
    rotServo.write(ROT_INIT_ANGLE);
}

// Initialize sonar status
void initSonar()
{
    // Set PinMode Correctly
    pinMode(TRIG_PIN1, OUTPUT);
    pinMode(ECHO_PIN1, INPUT);
    pinMode(TRIG_PIN2, OUTPUT);
    pinMode(ECHO_PIN2, INPUT);
}

// Initialize sonar status
void initLoadCell()
{
    // TODO: Fill this block
}

// Initialize LCD status
void initLCD()
{
    // TODO: Fill this block
}

// Initialize DC Motor status
void initMotor()
{
    // Set PinMode Correctly
    pinMode(LEFT_SPEED_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN1, OUTPUT);
    pinMode(LEFT_DIR_PIN2, OUTPUT);

    pinMode(RIGHT_SPEED_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN1, OUTPUT);
    pinMode(RIGHT_DIR_PIN2, OUTPUT);

    pinMode(STBY_PIN, OUTPUT);

    // Set initial value of each pin
    digitalWrite(STBY_PIN, HIGH);

    analogWrite(LEFT_SPEED_PIN, 0);
    analogWrite(RIGHT_SPEED_PIN, 0);

    digitalWrite(LEFT_DIR_PIN1, LOW);
    digitalWrite(LEFT_DIR_PIN2, LOW);

    digitalWrite(RIGHT_DIR_PIN1, LOW);
    digitalWrite(RIGHT_DIR_PIN2, LOW);
}

// Initialize Vacuum
void initVC()
{
    // We don't need to use this.
    // We just control the vacuum with real switch
}

// For Easy Delay
void commonDelay()
{
    delay(DELAY);
}

// Get Distance from sonar
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
        duration = pulseIn(ECHO_PIN1, HIGH, 30000); // TimeOut 30ms, MAX dist 5m
        distance = duration * 0.0343 / 2.0;         // Get Distance (cm)
    }
    else if (type == 2)
    {
        digitalWrite(TRIG_PIN2, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN2, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN2, LOW);
        duration = pulseIn(ECHO_PIN2, HIGH, 30000); // TimeOut 30ms, MAX dist 5m
        distance = duration * 0.0343 / 2.0;         // Get Distance (cm)
    }
    else
    {
        Serial.println("Error!!\n");
        error = true;
    }
    return distance;
}

void nodding()
{
    // get dist idx
    rotServo.write(ROT_INIT_ANGLE);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE - ROT_STEP);
    commonDelay();
    frontDist = getDist(1);

    rotServo.write(ROT_INIT_ANGLE);
    commonDelay();
    rotServo.write(ROT_INIT_ANGLE + ROT_STEP);
    commonDelay();
    backDist = getDist(2);

    rotServo.write(ROT_INIT_ANGLE);

    // Is there cliff?
    if (frontDist < CLIFF_THRESHOLD && backDist < CLIFF_THRESHOLD)
    {
        movingStatus = 1; // can go forward
    }
    else if (frontDist >= CLIFF_THRESHOLD && backDist < CLIFF_THRESHOLD)
    {
        movingStatus = 2; // can't go forward, can go backward
    }
    else
    {
        movingStatus = 3; // can't go forward and backward
    }

    if (DEBUG)
    {
        Serial.print("Moving Status : ");
        Serial.print(movingStatus);
    }
}

int findMaxIdx()
{
    int maxIdx = 0;
    float max = -1;
    for (int i = 0; i < NUM_DIST; i++)
    {
        if (max < dist[i])
        {
            max = dist[i];
            maxIdx = i;
        }
    }

    if (DEBUG)
    {
        Serial.print("Max distance at index ");
        Serial.print(maxIdx);
        Serial.print(" (");
        Serial.print(maxIdx * BASE_STEP);
        Serial.println(" degrees)");
    }

    return maxIdx;
}

// Get Around Distance Information
void getAroundDist()
{
    for (curBaseAngle = BASE_MIN_ANGLE; curBaseAngle < BASE_MAX_ANGLE; curBaseAngle += BASE_STEP)
    {
        Serial.println(curBaseAngle);
        int idx = curBaseAngle / BASE_STEP;
        baseServo.write(curBaseAngle);
        dist[idx] = getDist(1);
        dist[idx + (NUM_DIST / 2)] = getDist(2);
        commonDelay();
    }
    baseServo.write(BASE_INIT_ANGLE);
    commonDelay();
}

void stopMotor()
{
    analogWrite(LEFT_SPEED_PIN, 0);
    analogWrite(RIGHT_SPEED_PIN, 0);
    digitalWrite(LEFT_DIR_PIN1, LOW);
    digitalWrite(LEFT_DIR_PIN2, LOW);
    digitalWrite(RIGHT_DIR_PIN1, LOW);
    digitalWrite(RIGHT_DIR_PIN2, LOW);
}
/*
    Rotate the robot according to the angleIdx
    Actual Rotation Angle = angleIdx * BASE_STEP
*/
void rotation(int angleIdx)
{
    int desiredAngle = angleIdx * BASE_STEP;
    if (desiredAngle > 180)
    {
        desiredAngle -= 360;
    }
    int rotationTime = abs(desiredAngle) * ROTATION_TIME_PER_DEGREE;
    if (DEBUG)
    {
        Serial.print("Rotating by ");
        Serial.print(desiredAngle);
        Serial.print(" degrees (time: ");
        Serial.print(rotationTime);
        Serial.println(" ms)");
    }

    if (desiredAngle > 0)
    {
        // clockwise, left forward / right backward

        digitalWrite(LEFT_DIR_PIN1, HIGH);
        digitalWrite(LEFT_DIR_PIN2, LOW);
        analogWrite(LEFT_SPEED_PIN, MOTOR_SPEED);

        digitalWrite(RIGHT_DIR_PIN1, LOW);
        digitalWrite(RIGHT_DIR_PIN2, HIGH);
        analogWrite(RIGHT_SPEED_PIN, MOTOR_SPEED);
    }
    else if (desiredAngle < 0)
    {
        // counter-clockwise, left backward / right forward

        digitalWrite(LEFT_DIR_PIN1, LOW);
        digitalWrite(LEFT_DIR_PIN2, HIGH);
        analogWrite(LEFT_SPEED_PIN, MOTOR_SPEED);

        digitalWrite(RIGHT_DIR_PIN1, HIGH);
        digitalWrite(RIGHT_DIR_PIN2, LOW);
        analogWrite(RIGHT_SPEED_PIN, MOTOR_SPEED);
    }

    // rotate!
    delay(rotationTime);

    stopMotor();
    commonDelay();
}

/*
    Move forward for time (ms)
*/
void moveForward(int time)
{
    // moving forward
    digitalWrite(LEFT_DIR_PIN1, HIGH);
    digitalWrite(LEFT_DIR_PIN2, LOW);
    analogWrite(LEFT_SPEED_PIN, MOTOR_SPEED);

    digitalWrite(RIGHT_DIR_PIN1, HIGH);
    digitalWrite(RIGHT_DIR_PIN2, LOW);
    analogWrite(RIGHT_SPEED_PIN, MOTOR_SPEED);

    delay(time);

    stopMotor();
    commonDelay();
}

void loop()
{
    getAroundDist();
    rotation(findMaxIdx());
    // baseServo.write(BASE_INIT_ANGLE);
    commonDelay();

    movingStatus = 1;
    nodding(); // check whether the robot can go forward or not
    if (movingStatus == 1)
    {
        moveForward(100);
    }
    else if (movingStatus == 2)
    {
        rotation(NUM_DIST / 2);
        moveForward(100);
    }
    else if (movingStatus == 3)
    {
        rotation(NUM_DIST / 4);
    }
}