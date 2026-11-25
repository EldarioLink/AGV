#include <Arduino.h>

// =====================================================
// MOTOR RIGHT
// =====================================================

const int RIGHT_PWM = 2;
const int RIGHT_IN1 = 3;
const int RIGHT_IN2 = 4;

// =====================================================
// MOTOR LEFT
// =====================================================

const int LEFT_PWM = 7;
const int LEFT_IN1 = 8;
const int LEFT_IN2 = 9;

// =====================================================
// ENCODERS
// =====================================================

const int RIGHT_ENCODER_PIN = 6;
const int LEFT_ENCODER_PIN = 10;

volatile unsigned long rightEncoderPulses = 0;
volatile unsigned long leftEncoderPulses = 0;

// =====================================================
// WHEEL
// =====================================================

const float PULSES_PER_REVOLUTION = 135.0;
const float WHEEL_DIAMETER = 0.070;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// =====================================================
// PWM
// =====================================================

const int MIN_PWM = 80;
const int MAX_PWM = 255;

// =====================================================
// SPEED
// =====================================================

const float MAX_SPEED = 0.25;

// Шаг изменения целевой скорости
const float SPEED_STEP = 0.01;

// =====================================================
// TARGET SPEED
// =====================================================

// Команда от Orange Pi
float targetLeftSpeed = 0.0;
float targetRightSpeed = 0.0;

// Плавно изменяемая скорость
float currentLeftSpeed = 0.0;
float currentRightSpeed = 0.0;

// =====================================================
// PWM
// =====================================================

int leftPWM = 0;
int rightPWM = 0;

// =====================================================
// PID LEFT
// =====================================================

float KpLeft = 500.0;
float KiLeft = 80.0;
float KdLeft = 0.0;

float integralLeft = 0.0;
float previousErrorLeft = 0.0;

// =====================================================
// PID RIGHT
// =====================================================

float KpRight = 500.0;
float KiRight = 80.0;
float KdRight = 0.0;

float integralRight = 0.0;
float previousErrorRight = 0.0;

// =====================================================
// ENCODER INTERRUPTS
// =====================================================

void rightEncoderISR()
{
    rightEncoderPulses++;
}

void leftEncoderISR()
{
    leftEncoderPulses++;
}

// =====================================================
// MOTOR CONTROL
// =====================================================

void setRightMotor(int pwm)
{
    if (pwm > 0)
    {
        digitalWrite(RIGHT_IN1, LOW);
        digitalWrite(RIGHT_IN2, HIGH);

        analogWrite(RIGHT_PWM, pwm);
    }
    else if (pwm < 0)
    {
        digitalWrite(RIGHT_IN1, HIGH);
        digitalWrite(RIGHT_IN2, LOW);

        analogWrite(RIGHT_PWM, -pwm);
    }
    else
    {
        analogWrite(RIGHT_PWM, 0);

        digitalWrite(RIGHT_IN1, LOW);
        digitalWrite(RIGHT_IN2, LOW);
    }
}

// =====================================================

void setLeftMotor(int pwm)
{
    if (pwm > 0)
    {
        digitalWrite(LEFT_IN1, HIGH);
        digitalWrite(LEFT_IN2, LOW);

        analogWrite(LEFT_PWM, pwm);
    }
    else if (pwm < 0)
    {
        digitalWrite(LEFT_IN1, LOW);
        digitalWrite(LEFT_IN2, HIGH);

        analogWrite(LEFT_PWM, -pwm);
    }
    else
    {
        analogWrite(LEFT_PWM, 0);

        digitalWrite(LEFT_IN1, LOW);
        digitalWrite(LEFT_IN2, LOW);
    }
}

// =====================================================
// STOP
// =====================================================

void stopMotors()
{
    targetLeftSpeed = 0.0;
    targetRightSpeed = 0.0;

    currentLeftSpeed = 0.0;
    currentRightSpeed = 0.0;

    leftPWM = 0;
    rightPWM = 0;

    integralLeft = 0.0;
    integralRight = 0.0;

    previousErrorLeft = 0.0;
    previousErrorRight = 0.0;

    setLeftMotor(0);
    setRightMotor(0);
}

// =====================================================
// UART COMMAND
// =====================================================

void processCommand(String command)
{
    command.trim();

    if (command.startsWith("VEL"))
    {
        float leftSpeed;
        float rightSpeed;

        int result = sscanf(
            command.c_str(),
            "VEL %f %f",
            &leftSpeed,
            &rightSpeed);

        if (result == 2)
        {
            if (
                leftSpeed >= -MAX_SPEED &&
                leftSpeed <= MAX_SPEED &&
                rightSpeed >= -MAX_SPEED &&
                rightSpeed <= MAX_SPEED)
            {
                targetLeftSpeed = leftSpeed;
                targetRightSpeed = rightSpeed;

                integralLeft = 0.0;
                integralRight = 0.0;

                previousErrorLeft = 0.0;
                previousErrorRight = 0.0;

                Serial1.print("OK VEL ");
                Serial1.print(targetLeftSpeed, 3);
                Serial1.print(" ");
                Serial1.println(targetRightSpeed, 3);
            }
            else
            {
                Serial1.println("ERROR SPEED");
            }
        }
        else
        {
            Serial1.println("ERROR FORMAT");
        }
    }

    else if (command == "STOP")
    {
        stopMotors();

        Serial1.println("OK STOP");
    }
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    // ---------------------------------------------------
    // USB SERIAL
    // ---------------------------------------------------

    Serial.begin(115200);

    // ---------------------------------------------------
    // UART TO ORANGE PI
    // ---------------------------------------------------

    Serial1.setTX(0); // GP0
    Serial1.setRX(1); // GP1
    Serial1.begin(115200);

    // ---------------------------------------------------
    // RIGHT MOTOR
    // ---------------------------------------------------

    pinMode(RIGHT_PWM, OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);

    // ---------------------------------------------------
    // LEFT MOTOR
    // ---------------------------------------------------

    pinMode(LEFT_PWM, OUTPUT);
    pinMode(LEFT_IN1, OUTPUT);
    pinMode(LEFT_IN2, OUTPUT);

    // ---------------------------------------------------
    // ENCODERS
    // ---------------------------------------------------

    pinMode(RIGHT_ENCODER_PIN, INPUT);
    pinMode(LEFT_ENCODER_PIN, INPUT);

    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENCODER_PIN),
        rightEncoderISR,
        RISING);

    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENCODER_PIN),
        leftEncoderISR,
        RISING);

    // ---------------------------------------------------
    // STOP MOTORS
    // ---------------------------------------------------

    stopMotors();

    // ---------------------------------------------------
    // DEBUG
    // ---------------------------------------------------

    Serial.println();
    Serial.println("================================");
    Serial.println("TWO WHEEL SPEED CONTROLLER");
    Serial.println("================================");

    Serial.println("RIGHT:");
    Serial.println("PWM  = GP2");
    Serial.println("IN1  = GP3");
    Serial.println("IN2  = GP4");
    Serial.println("ENC  = GP6");

    Serial.println();

    Serial.println("LEFT:");
    Serial.println("PWM  = GP7");
    Serial.println("IN1  = GP8");
    Serial.println("IN2  = GP9");
    Serial.println("ENC  = GP10");

    Serial.println();

    Serial.println("UART:");
    Serial.println("TX = GP0");
    Serial.println("RX = GP1");

    Serial.println();
    Serial.println("Waiting for VEL command...");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    // ===================================================
    // RECEIVE UART COMMAND
    // ===================================================

    if (Serial1.available())
    {
        String command =
            Serial1.readStringUntil('\n');

        processCommand(command);
    }

    // ===================================================
    // SMOOTH SPEED RAMP
    // ===================================================

    static unsigned long lastSpeedRamp = 0;

    if (millis() - lastSpeedRamp >= 100)
    {
        lastSpeedRamp = millis();

        // LEFT

        if (currentLeftSpeed < targetLeftSpeed)
        {
            currentLeftSpeed += SPEED_STEP;

            if (currentLeftSpeed > targetLeftSpeed)
                currentLeftSpeed = targetLeftSpeed;
        }
        else if (currentLeftSpeed > targetLeftSpeed)
        {
            currentLeftSpeed -= SPEED_STEP;

            if (currentLeftSpeed < targetLeftSpeed)
                currentLeftSpeed = targetLeftSpeed;
        }

        // RIGHT

        if (currentRightSpeed < targetRightSpeed)
        {
            currentRightSpeed += SPEED_STEP;

            if (currentRightSpeed > targetRightSpeed)
                currentRightSpeed = targetRightSpeed;
        }
        else if (currentRightSpeed > targetRightSpeed)
        {
            currentRightSpeed -= SPEED_STEP;

            if (currentRightSpeed < targetRightSpeed)
                currentRightSpeed = targetRightSpeed;
        }
    }

    // ===================================================
    // SPEED MEASUREMENT + PID
    // ===================================================

    static unsigned long lastTime = 0;
    static unsigned long lastLeftPulses = 0;
    static unsigned long lastRightPulses = 0;

    unsigned long currentTime = millis();

    if (currentTime - lastTime >= 200)
    {
        float dt =
            (currentTime - lastTime) / 1000.0;

        // -------------------------------------------------
        // READ ENCODERS
        // -------------------------------------------------

        noInterrupts();

        unsigned long currentLeftPulses =
            leftEncoderPulses;

        unsigned long currentRightPulses =
            rightEncoderPulses;

        interrupts();

        unsigned long leftPulseDelta =
            currentLeftPulses - lastLeftPulses;

        unsigned long rightPulseDelta =
            currentRightPulses - lastRightPulses;

        // -------------------------------------------------
        // LEFT SPEED
        // -------------------------------------------------

        float leftPulsesPerSecond =
            leftPulseDelta / dt;

        float leftRPS =
            leftPulsesPerSecond /
            PULSES_PER_REVOLUTION;

        float actualLeftSpeed =
            leftRPS *
            WHEEL_CIRCUMFERENCE;

        // -------------------------------------------------
        // RIGHT SPEED
        // -------------------------------------------------

        float rightPulsesPerSecond =
            rightPulseDelta / dt;

        float rightRPS =
            rightPulsesPerSecond /
            PULSES_PER_REVOLUTION;

        float actualRightSpeed =
            rightRPS *
            WHEEL_CIRCUMFERENCE;

        // =================================================
        // LEFT PID
        // =================================================

        if (currentLeftSpeed > 0.0)
        {
            float error =
                currentLeftSpeed - actualLeftSpeed;

            integralLeft += error * dt;

            if (integralLeft > 1.0)
                integralLeft = 1.0;

            if (integralLeft < -1.0)
                integralLeft = -1.0;

            float derivative =
                (error - previousErrorLeft) / dt;

            float output =
                KpLeft * error +
                KiLeft * integralLeft +
                KdLeft * derivative;

            leftPWM += (int)output;

            if (leftPWM > MAX_PWM)
                leftPWM = MAX_PWM;

            if (leftPWM < MIN_PWM)
                leftPWM = MIN_PWM;

            setLeftMotor(leftPWM);

            previousErrorLeft = error;
        }
        else if (currentLeftSpeed < 0.0)
        {
            // Пока используем такую же PID-логику
            // для движения назад

            float desiredSpeed =
                -currentLeftSpeed;

            float error =
                desiredSpeed - actualLeftSpeed;

            integralLeft += error * dt;

            if (integralLeft > 1.0)
                integralLeft = 1.0;

            if (integralLeft < -1.0)
                integralLeft = -1.0;

            float output =
                KpLeft * error +
                KiLeft * integralLeft;

            leftPWM += (int)output;

            if (leftPWM > MAX_PWM)
                leftPWM = MAX_PWM;

            if (leftPWM < MIN_PWM)
                leftPWM = MIN_PWM;

            setLeftMotor(-leftPWM);
        }
        else
        {
            leftPWM = 0;
            setLeftMotor(0);

            integralLeft = 0.0;
            previousErrorLeft = 0.0;
        }

        // =================================================
        // RIGHT PID
        // =================================================

        if (currentRightSpeed > 0.0)
        {
            float error =
                currentRightSpeed - actualRightSpeed;

            integralRight += error * dt;

            if (integralRight > 1.0)
                integralRight = 1.0;

            if (integralRight < -1.0)
                integralRight = -1.0;

            float derivative =
                (error - previousErrorRight) / dt;

            float output =
                KpRight * error +
                KiRight * integralRight +
                KdRight * derivative;

            rightPWM += (int)output;

            if (rightPWM > MAX_PWM)
                rightPWM = MAX_PWM;

            if (rightPWM < MIN_PWM)
                rightPWM = MIN_PWM;

            setRightMotor(rightPWM);

            previousErrorRight = error;
        }
        else if (currentRightSpeed < 0.0)
        {
            float desiredSpeed =
                -currentRightSpeed;

            float error =
                desiredSpeed - actualRightSpeed;

            integralRight += error * dt;

            if (integralRight > 1.0)
                integralRight = 1.0;

            if (integralRight < -1.0)
                integralRight = -1.0;

            float output =
                KpRight * error +
                KiRight * integralRight;

            rightPWM += (int)output;

            if (rightPWM > MAX_PWM)
                rightPWM = MAX_PWM;

            if (rightPWM < MIN_PWM)
                rightPWM = MIN_PWM;

            setRightMotor(-rightPWM);
        }
        else
        {
            rightPWM = 0;
            setRightMotor(0);

            integralRight = 0.0;
            previousErrorRight = 0.0;
        }

        // =================================================
        // DEBUG USB
        // =================================================

        Serial.print("L pulses: ");
        Serial.print(leftPulseDelta);

        Serial.print(" | R pulses: ");
        Serial.print(rightPulseDelta);

        Serial.print(" || ");

        Serial.print("L Target: ");
        Serial.print(currentLeftSpeed, 3);

        Serial.print(" | L Speed: ");
        Serial.print(actualLeftSpeed, 3);

        Serial.print(" | L PWM: ");
        Serial.print(leftPWM);

        Serial.print(" || R Target: ");
        Serial.print(currentRightSpeed, 3);

        Serial.print(" | R Speed: ");
        Serial.print(actualRightSpeed, 3);

        Serial.print(" | R PWM: ");
        Serial.println(rightPWM);

        // -------------------------------------------------
        // UPDATE
        // -------------------------------------------------

        lastLeftPulses = currentLeftPulses;
        lastRightPulses = currentRightPulses;

        lastTime = currentTime;
    }
}
