#include <Servo.h>

// global flag
bool error = false;

// for servo
#define ROT_PIN 8
#define BASE_PIN 7

#define Z_MIN_ANGLE 0
#define Z_MAX_ANGLE 180

#define DELAY 100

#define X_BASE_ANGLE 90
#define X_ROT_STEP 15
#define Z_BASE_ANGLE 0
#define Z_ROT_STEP 30

Servo baseServo, rotServo;

int curZAngle = Z_BASE_ANGLE;
int curXAngle = X_BASE_ANGLE;

// for sonar
#define TRIG_PIN1 13
#define ECHO_PIN1 12
#define TRIG_PIN2 11
#define ECHO_PIN2 10
#define NUM_DIST 360/Z_ROT_STEP


float dist[NUM_DIST]; // 0~6 : left angle, 7~13 : right angle

void setup()
{
    Serial.begin(9600); // Set rate

    // Initialize Status
    init();
}

// Initialize All
void init()
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
    curZAngle = Z_BASE_ANGLE;
    curXAngle = X_BASE_ANGLE;

    // Initialize Servo Location
    ZServo.write(Z_BASE_ANGLE);
    XServo.write(X_BASE_ANGLE);
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
    // TODO: Fill this block
}

// Initialize Vacuum
void initVC()
{
    // TODO: Fill this block
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
    if(type == 1){
        digitalWrite(TRIG_PIN1, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN1, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN1, LOW);
        duration = pulseIn(ECHO_PIN1, HIGH, 30000); // TimeOut 30ms, MAX dist 5m
        distance = duration * 0.0343 / 2.0;        // Get Distance (cm)
    }else if(type == 2){
        digitalWrite(TRIG_PIN2, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN2, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN2, LOW);
        duration = pulseIn(ECHO_PIN2, HIGH, 30000); // TimeOut 30ms, MAX dist 5m
        distance = duration * 0.0343 / 2.0;         // Get Distance (cm)
    }else{
        Serial.println("Error!!\n");
        error = true;
    }
    return distance;
}

void nodding()
{
    // get dist idx
    int idx = curZAngle / Z_ROT_STEP;

    XServo.write(X_BASE_ANGLE);
    commonDelay();
    XServo.write(X_BASE_ANGLE - X_ROT_STEP);
    commonDelay();
    dist[idx] = getDist(1);

    XServo.write(X_BASE_ANGLE);
    commonDelay();
    XServo.write(X_BASE_ANGLE + X_ROT_STEP);
    commonDelay();
    dist[idx+(NUM_DIST/2)] = getDist(2);

    XServo.write(X_BASE_ANGLE);
}

int findMaxIdx()
{
    int maxIdx = 0;
    float max = -1;
    for (int i = 0; i < NUM_DIST; i++)
    {
        if(max < dist[i]){
            max = dist[i];
            maxIdx = i;
        }
    }
    return maxIdx;
}

// Get Around Distance Information
void getAroundDist(){
    for (curZAngle = Z_MIN_ANGLE; curZAngle < Z_MAX_ANGLE; curZAngle += Z_ROT_STEP)
    {
        ZServo.write(angle);
        commonDelay();
        nodding();
    }
}

/*
    Rotate the robot according to the angleIdx
    Actual Rotation Angle = angleIdx * Z_ROT_STEP
*/
void Rotation(int angleIdx){

}

/*
    Move forward for time (ms)
*/
void moveForward(int time){

}


void loop()
{
    getAroundDist();
    Rotation(findMaxIdx());
    moveForward(100);
}