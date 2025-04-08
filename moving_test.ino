#include <Servo.h>

// global flag
bool error = false;

#define DEBUG true // print debug msg?

#define DELAY 500

#define BASE_STEP 30

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
    initMotor();
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

    moveForward(300);
    commonDelay();

    rotation(2);
    commonDelay();
}