#include <Arduino.h>

// =====================================================
// MOTOR
// =====================================================

const int ENA = 2;
const int IN1 = 3;
const int IN2 = 4;

// =====================================================
// ENCODER
// =====================================================

const int ENCODER_PIN = 6;

volatile unsigned long encoderPulses = 0;

// =====================================================
// WHEEL
// =====================================================

const float PULSES_PER_REVOLUTION = 672.0;
const float WHEEL_DIAMETER = 0.070;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// =====================================================
// PWM
// =====================================================

const int MIN_PWM = 80;
const int MAX_PWM = 255;

int pwm = 80;

// =====================================================
// SPEED
// =====================================================

// Скорость, которую запросил пользователь
float targetSpeed = 0.0;

// Скорость, которую реально сейчас требует PID
float currentTargetSpeed = 0.0;

// Максимальная скорость
const float MAX_SPEED = 0.25;

// Шаг плавного изменения скорости
const float SPEED_STEP = 0.01;

// =====================================================
// PID
// =====================================================

float Kp = 500.0;
float Ki = 80.0;
float Kd = 0.0;

float integral = 0.0;
float previousError = 0.0;

// =====================================================
// ENCODER INTERRUPT
// =====================================================

void encoderISR()
{
  encoderPulses++;
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENCODER_PIN, INPUT);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_PIN),
      encoderISR,
      RISING);

  analogWrite(ENA, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  Serial.println();
  Serial.println("================================");
  Serial.println("ONE WHEEL SPEED CONTROLLER");
  Serial.println("================================");
  Serial.println("Enter speed in m/s:");
  Serial.println("0.05");
  Serial.println("0.10");
  Serial.println("0.15");
  Serial.println("Stop: 0");
  Serial.println();
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  // =================================================
  // RECEIVE COMMAND
  // =================================================

  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0)
    {
      float newSpeed = command.toFloat();

      if (newSpeed >= 0.0 && newSpeed <= MAX_SPEED)
      {
        targetSpeed = newSpeed;

        Serial.print("New target speed: ");
        Serial.print(targetSpeed, 3);
        Serial.println(" m/s");

        // Сбрасываем интегральную часть
        integral = 0.0;
        previousError = 0.0;

        // Если остановка
        if (targetSpeed == 0.0)
        {
          currentTargetSpeed = 0.0;

          pwm = 0;

          analogWrite(ENA, 0);

          digitalWrite(IN1, LOW);
          digitalWrite(IN2, LOW);
        }
        else
        {
          // Направление вперёд
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);

          // Минимальный рабочий PWM
          if (pwm < MIN_PWM)
            pwm = MIN_PWM;

          analogWrite(ENA, pwm);
        }
      }
    }
  }

  // =================================================
  // SMOOTH TARGET SPEED
  // =================================================

  static unsigned long lastSpeedRamp = 0;

  if (millis() - lastSpeedRamp >= 100)
  {
    lastSpeedRamp = millis();

    // Увеличиваем целевую скорость
    if (currentTargetSpeed < targetSpeed)
    {
      currentTargetSpeed += SPEED_STEP;

      if (currentTargetSpeed > targetSpeed)
        currentTargetSpeed = targetSpeed;
    }

    // Уменьшаем целевую скорость
    else if (currentTargetSpeed > targetSpeed)
    {
      currentTargetSpeed -= SPEED_STEP;

      if (currentTargetSpeed < targetSpeed)
        currentTargetSpeed = targetSpeed;
    }
  }

  // =================================================
  // SPEED MEASUREMENT + PID
  // =================================================

  static unsigned long lastTime = 0;
  static unsigned long lastPulses = 0;

  unsigned long currentTime = millis();

  if (currentTime - lastTime >= 200)
  {
    float dt =
        (currentTime - lastTime) / 1000.0;

    // Считываем энкодер
    noInterrupts();

    unsigned long currentPulses =
        encoderPulses;

    interrupts();

    unsigned long pulseDelta =
        currentPulses - lastPulses;

    // =================================================
    // CALCULATE SPEED
    // =================================================

    float pulsesPerSecond =
        pulseDelta / dt;

    float rps =
        pulsesPerSecond /
        PULSES_PER_REVOLUTION;

    float actualSpeed =
        rps * WHEEL_CIRCUMFERENCE;

    // =================================================
    // PID
    // =================================================

    if (currentTargetSpeed > 0.0)
    {
      float error =
          currentTargetSpeed - actualSpeed;

      // Integral
      integral += error * dt;

      // Ограничение integral
      if (integral > 1.0)
        integral = 1.0;

      if (integral < -1.0)
        integral = -1.0;

      // Derivative
      float derivative =
          (error - previousError) / dt;

      // PID
      float output =
          Kp * error +
          Ki * integral +
          Kd * derivative;

      // Изменение PWM
      pwm += (int)output;

      // Ограничение PWM
      if (pwm > MAX_PWM)
        pwm = MAX_PWM;

      if (pwm < MIN_PWM)
        pwm = MIN_PWM;

      analogWrite(ENA, pwm);

      previousError = error;
    }

    // =================================================
    // DEBUG
    // =================================================

    Serial.print("Target: ");
    Serial.print(targetSpeed, 3);

    Serial.print(" | Ramp: ");
    Serial.print(currentTargetSpeed, 3);

    Serial.print(" | Speed: ");
    Serial.print(actualSpeed, 3);

    Serial.print(" | Error: ");
    Serial.print(currentTargetSpeed - actualSpeed, 3);

    Serial.print(" | PWM: ");
    Serial.print(pwm);

    Serial.print(" | Pulses/s: ");
    Serial.println(pulsesPerSecond);

    lastPulses = currentPulses;
    lastTime = currentTime;
  }
}
