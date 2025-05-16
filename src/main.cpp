#include <Arduino.h>
#include <WiFi.h>
#include <AccelStepper.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// Конфигурация пинов
#define MOTOR_STEP_PIN 4   // Пин управления шагами
#define MOTOR_DIR_PIN 16   // Пин управления направлением
#define ENDSTOP_PIN 17     // Пин концевика
#define ENABLE_PIN 5       // Пин управления питанием драйвера.
#define ENABLE_ACTIVE LOW  // Уровень активного состояния ENABLE (LOW - включен, HIGH - выключен)
#define ENDSTOP_ACTIVE LOW // Уровень активного состояния концевика

// Пины управления фотоаппаратом через транзисторы/реле
#define FOCUS_CONTROL_PIN 18   // IN1 на модуле
#define SHUTTER_CONTROL_PIN 19 // IN2 на модуле

// Механические параметры
#define MICROSTEPS 16
#define STEPS_PER_REVOLUTION 100
#define SCREW_LEAD 2.0
#define GEAR_RATIO (109.0 / 12.0)
#define MAX_TRAVEL 97.0     // Максимальное расстояние в миллиметрах от ноля до конца
#define HOMING_SPEED 10.0   // Скорость хоуминга
#define DEFAULT_ACCEL 100.0 // Ускорение по умолчанию в шаг/с^2
#define DEBOUNCE_DELAY 50   // Задержка в миллисекундах, регулируйте по необходимости

// Список сетей Wi-Fi для подключения (SSID и пароль)
struct WifiCredentials
{
  const char *ssid;
  const char *password;
};

WifiCredentials wifiNetworks[] = {
    {"SSID1", "PASSWORD1"},
    // {"SSID2", "PASSWORD2"},
    // {"SSID3", "PASSWORD3"},
    {nullptr, nullptr} // Маркер конца списка
};

WebServer server(80);

unsigned long last_server_handle_time = 0;
const unsigned long server_handle_interval = 50;
float startPosition = 0.0;
bool returnToStartEnabled = false;

class MacroRail
{
public:
  enum State
  {
    IDLE,
    HOMING,
    HOMING_COMPLETE,
    HOMING_RETRACT,
    MOVING,
    SHOOTING,
    ERROR
  };

  struct Settings
  {
    float step_size = 0.3; // Шаг в мм
    int total_photos = 3; // Количество фотографий
    float max_speed = 0.7;  // Максимальная корость в мм/с
    int focus_time = 500;   // Время удержания автофокуса в мс
    int release_time = 200; // Время удержания спуска затвора в мс
    int before_shoot_delay = 100; // Задержка перед спуском затвора в мс
    int after_shoot_delay = 100; // Задержка после спуска затвора в мс
  };

  unsigned long homing_retract_start;
  static const char *get_state_string(State s);

  float get_target_position() const
  {
    return stepper.targetPosition() / steps_per_mm();
  }

  float get_current_position() const
  {
    return stepper.currentPosition() / steps_per_mm();
  }

  long get_current_steps() const
  {
    return stepper.currentPosition();
  }

  float get_steps_per_mm() const
  {
    return steps_per_mm();
  }

  void motor_test()
  {
    enable_motor();
    stepper.move(100);
    while (stepper.distanceToGo() != 0)
    {
      stepper.run();
    }
    disable_motor();
  }

  void enable() { enable_motor(); }
  void disable() { disable_motor(); }

  void force_enable() { enable_motor(); }

  void test_direction()
  {
    enable_motor();
    Serial.println("Testing direction...");

    Serial.println("Moving forward 1mm...");
    stepper.moveTo(1 * steps_per_mm());
    while (stepper.distanceToGo() != 0)
    {
      stepper.run();
    }
    delay(1000);

    Serial.println("Moving back to 0mm...");
    stepper.moveTo(0);
    while (stepper.distanceToGo() != 0)
    {
      stepper.run();
    }

    disable_motor();
    Serial.println("Direction test completed");
  }

  MacroRail() : stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN),
                state(IDLE),
                current_pos(0)
  {
    pinMode(ENDSTOP_PIN, INPUT_PULLUP);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, !ENABLE_ACTIVE);

    pinMode(FOCUS_CONTROL_PIN, OUTPUT);
    digitalWrite(FOCUS_CONTROL_PIN, LOW);

    pinMode(SHUTTER_CONTROL_PIN, OUTPUT);
    digitalWrite(SHUTTER_CONTROL_PIN, LOW);

    stepper.setPinsInverted(true, false, false); // DIR, STEP, ENABLE
    stepper.setEnablePin(-1);                    // Управление ENABLE вручную
    stepper.setAcceleration(10000);
    stepper.setMaxSpeed(settings.max_speed * steps_per_mm());

    Serial.printf("Motor settings: %.2f steps/mm\n", steps_per_mm());
  }

  bool check_endstop()
  {
    static unsigned long last_endstop_change = 0;
    static bool last_endstop_state = (digitalRead(ENDSTOP_PIN) == ENDSTOP_ACTIVE);
    bool current_endstop_state = (digitalRead(ENDSTOP_PIN) == ENDSTOP_ACTIVE);
    unsigned long current_time = millis();

    if (current_endstop_state != last_endstop_state)
    {
      if (current_time - last_endstop_change > DEBOUNCE_DELAY)
      {
        last_endstop_state = current_endstop_state;
        last_endstop_change = current_time;
        return current_endstop_state;
      }
    }
    return last_endstop_state;
  }

  void update()
  {
    static int previous_state = -1;
    static bool previous_endstop_state = false;
    bool current_endstop_state = (digitalRead(ENDSTOP_PIN) == ENDSTOP_ACTIVE);

    if (state != previous_state || current_endstop_state != previous_endstop_state)
    {
      Serial.printf("State: %d (%s), Endstop: %d (%s)\n",
                    state, get_state_string(state),
                    current_endstop_state, current_endstop_state ? "PRESSED" : "released");
      previous_state = state;
      previous_endstop_state = current_endstop_state;
    }

    if (state == HOMING)
    {
      if (check_endstop() && !homing_endstop_triggered)
      {
        homing_endstop_triggered = true;
        long steps_moved = stepper.currentPosition() - homing_start_position;
        float mm_moved = steps_moved / steps_per_mm();
        unsigned long time_elapsed = millis() - homing_start_time;
        float actual_speed = abs(mm_moved) / (time_elapsed / 1000.0);

        Serial.println("\n=== ENDSTOP HIT ===");
        Serial.printf("Moved: %ld steps (%.2fmm) in %lums\n",
                      steps_moved, mm_moved, time_elapsed);
        Serial.printf("Avg speed: %.1fmm/s (target %.1fmm/s)\n",
                      actual_speed, HOMING_SPEED);
        Serial.printf("Final speed: %.1f steps/s\n", stepper.speed());

        disable_motor(); // Немедленно отключаем двигатель
        Serial.printf("Motor disabled\n");
        delay(1000); // Даем время остановиться

        state = HOMING_RETRACT;
        homing_retract_start = millis();
        enable_motor(); // Включаем двигатель обратно перед ретрактом
        Serial.printf("Motor enabled\n");
        complete_homing();
      }
      stepper.run();
    }
    else if (state == HOMING_RETRACT)
    {
      if (stepper.distanceToGo() == 0)
      {
        stepper.setCurrentPosition(0);
        current_pos = 0;
        state = IDLE;
        disable_motor();
        Serial.println("=== RETRACT COMPLETE - ZERO SET ===");
        is_busy = false;
        homing_endstop_triggered = false; // Сбрасываем флаг после успешного хоуминга
      }
      else if (millis() - homing_retract_start > 60000)
      {
        Serial.println("Retract timeout!");
        stepper.stop();
        state = ERROR;
        disable_motor();
      }
      stepper.run();
    }
    else if (state == MOVING)
    {
      if (stepper.distanceToGo() == 0)
      {
        state = IDLE;
        disable_motor();
        is_busy = false;
      }
      else
      {
        stepper.run();
        current_pos = stepper.currentPosition() / steps_per_mm();
      }
    }
    else if (state == SHOOTING)
    {
      handle_shooting();
    }
    else if (state == ERROR)
    {
      handle_error();
    }
    else if (state == IDLE)
    {
      handle_idle();
      is_busy = false;
    }
    else // Для всех остальных состояний (на всякий случай)
    {
      if (check_endstop())
      {
        emergency_stop("Endstop triggered");
      }
      stepper.run(); // Чтобы AccelStepper мог обрабатывать команды
    }
  }

  void start_homing()
  {
    Serial.println("start_homing() CALLED");
    if (state == ERROR)
      return;
    is_busy = true;
    enable_motor();
    state = HOMING;
    homing_endstop_triggered = false;
    homing_start_time = millis();
    homing_start_position = stepper.currentPosition();

    stepper.setMaxSpeed(settings.max_speed * steps_per_mm());
    stepper.setAcceleration(10000);
    stepper.move(-MAX_TRAVEL * steps_per_mm());

    Serial.println("=== HOMING STARTED ===");
    Serial.printf("Start position: %ld steps (%.2f mm)\n",
                  homing_start_position,
                  homing_start_position / steps_per_mm());
  }

  void move_to(float position)
  {
    if (state != IDLE && state != SHOOTING)
      return;

    is_busy = true;
    position = constrain(position, 0, MAX_TRAVEL);
    long target_steps = position * steps_per_mm();

    // логирование текущего и целевого положения
    Serial.printf("Move command: %.2fmm -> %ld steps (current: %ld, pos: %.2fmm)\n",
                  position, target_steps, stepper.currentPosition(), current_pos);

    enable_motor();
    stepper.moveTo(target_steps);
    state = MOVING;
  }

  void start_shooting(const Settings &new_settings)
  {
    if (state != IDLE)
      return;
    is_busy = true;
    settings = new_settings;
    photo_count = 0;
    shooting_stage = 0;
    state = SHOOTING;

    enable_motor();
    update_motor_settings();

    Serial.printf("Starting shooting: %d photos, step %.2fmm, speed %.1f mm/s, before: %dms, after: %dms\n",
                  settings.total_photos, settings.step_size, settings.max_speed,
                  settings.before_shoot_delay, settings.after_shoot_delay);
  }

  void stop()
  {
    stepper.stop();
    state = IDLE;
    disable_motor();
    is_busy = false;
    Serial.println("Movement stopped");
  }

  void reset_emergency()
  {
    if (state == ERROR && digitalRead(ENDSTOP_PIN) != LOW)
    {
      state = IDLE;
      is_busy = false;
    }
  }

  // Геттеры
  float get_position() const { return current_pos; }
  State get_state() const { return state; }
  Settings get_settings() const { return settings; }
  int get_photo_count() const { return photo_count; }

private:
  mutable AccelStepper stepper;
  State state;
  Settings settings;
  float current_pos;
  int photo_count = 0;

  unsigned long homing_start_time;
  long homing_start_position;
  bool homing_endstop_triggered = false;
  bool is_busy = false;

  void update_motor_settings()
  {
    stepper.setMaxSpeed(settings.max_speed * steps_per_mm());
    stepper.setAcceleration(DEFAULT_ACCEL * steps_per_mm());
  }

  float steps_per_mm() const
  {
    return (STEPS_PER_REVOLUTION * MICROSTEPS * GEAR_RATIO) / SCREW_LEAD;
  }

  void enable_motor()
  {
    digitalWrite(ENABLE_PIN, ENABLE_ACTIVE);
    delayMicroseconds(100); // Короткая задержка для стабилизации
  }

  void disable_motor()
  {
    digitalWrite(ENABLE_PIN, !ENABLE_ACTIVE); // Инвертируем состояние
  }

  void shooting_finished_callback()
  {
    Serial.println("Shooting finished!");
    if (returnToStartEnabled)
    {
      Serial.print("Returning to start position: ");
      Serial.println(startPosition, 2);
      move_to(startPosition);
      returnToStartEnabled = false;
    }
  }

  void complete_homing()
  {
    stepper.setMaxSpeed(settings.max_speed * steps_per_mm());

    stepper.setAcceleration(DEFAULT_ACCEL * steps_per_mm());

    long retract_distance = (long)(1 * steps_per_mm());
    Serial.printf("=== HOMING COMPLETE - STARTING RETRACT ===\n");
    Serial.printf("Current position before retract command: %ld steps\n", stepper.currentPosition());
    Serial.printf("Target retract distance: %ld steps\n", retract_distance);
    Serial.printf("Current state - %ld \n", state);

    stepper.setCurrentPosition(0);
    stepper.moveTo(retract_distance);
    Serial.printf("Target position set for retract: %ld steps\n", stepper.targetPosition());
  }

  void handle_shooting()
  {
    static bool motor_enabled = false;

    if (stepper.distanceToGo() != 0)
    {
      if (!motor_enabled)
      {
        enable_motor();
        motor_enabled = true;
        movement_start_time = millis(); // Записываем время начала движения
      }
      stepper.run();
      current_pos = stepper.currentPosition() / steps_per_mm();
      return;
    }
    else if (motor_enabled)
    {
      disable_motor();
      motor_enabled = false;
      stage_start_time = millis(); // Записываем время остановки для задержки перед съемкой
      shooting_stage = 0;          // Переходим к задержке перед съемкой
      Serial.println("Movement complete - waiting before shoot");
      return;
    }

    switch (shooting_stage)
    {
    case 0: // Ожидание перед съемкой
      if (millis() - stage_start_time > settings.before_shoot_delay)
      {
        digitalWrite(FOCUS_CONTROL_PIN, HIGH); // Включаем автофокус (замыкаем 1 и 2)
        stage_start_time = millis();
        shooting_stage = 1;
        Serial.println("Focusing started");
      }
      break;

    case 1: // Ожидание фокусировки
      if (millis() - stage_start_time > settings.focus_time)
      {
        digitalWrite(SHUTTER_CONTROL_PIN, HIGH); // Включаем спуск затвора (добавляем контакт 3)
        stage_start_time = millis();
        shooting_stage = 2;
        Serial.println("Shutter released");
      }
      break;

    case 2: // Ожидание спуска затвора
      if (millis() - stage_start_time > settings.release_time)
      {
        digitalWrite(FOCUS_CONTROL_PIN, LOW);   // Выключаем автофокус
        digitalWrite(SHUTTER_CONTROL_PIN, LOW); // Выключаем спуск затвора
        photo_count++;
        Serial.printf("Photo %d taken at %.2fmm\n", photo_count, current_pos);
        stage_start_time = millis(); // Записываем время спуска затвора для задержки после съемки
        shooting_stage = 3;          // Переходим к задержке после съемки
        Serial.println("Waiting after shoot");
      }
      break;

    case 3: // Ожидание после съемки
      if (millis() - stage_start_time > settings.after_shoot_delay)
      {
        if (photo_count < settings.total_photos)
        {
          float new_pos = current_pos + settings.step_size;
          new_pos = constrain(new_pos, 0, MAX_TRAVEL);
          enable_motor();
          stepper.moveTo(new_pos * steps_per_mm());
          update_motor_settings();
          shooting_stage = 0; // Снова ждем остановки
        }
        else
        {
          state = IDLE;
          disable_motor();
          is_busy = false;
          Serial.println("Shooting completed");
          shooting_finished_callback();
        }
      }
      break;
    }
  }

  uint8_t shooting_stage = 0; // 0-начало, 1-фокусировка, 2-спуск, 3-ожидание после
  unsigned long stage_start_time = 0;
  unsigned long movement_start_time = 0;

  void handle_error()
  {
    // digitalWrite(STATUS_LED, millis() % 200 < 100); // Больше не используется
  }

  void handle_idle()
  {
    // digitalWrite(STATUS_LED, millis() % 1000 < 500); // Больше не используется
    disable_motor();
  }

  void emergency_stop(const char *reason)
  {
    stepper.stop();
    digitalWrite(ENABLE_PIN, !ENABLE_ACTIVE); // Принудительное отключение
    state = ERROR;
    disable_motor();
    Serial.print("EMERGENCY STOP: ");
    Serial.println(reason);
  }
};

const char *MacroRail::get_state_string(MacroRail::State s)
{
  switch (s)
  {
  case MacroRail::IDLE:
    return "IDLE";
  case MacroRail::HOMING:
    return "HOMING";
  case MacroRail::HOMING_COMPLETE:
    return "HOMING_COMPLETE";
  case MacroRail::HOMING_RETRACT:
    return "HOMING_RETRACT";
  case MacroRail::MOVING:
    return "MOVING";
  case MacroRail::SHOOTING:
    return "SHOOTING";
  case MacroRail::ERROR:
    return "ERROR";
  default:
    return "UNKNOWN";
  }
}

MacroRail rail;

const char *favicon = R"(
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">
  <circle cx="12" cy="12" r="10" fill="red"/>
  <circle cx="12" cy="12" r="6" fill="white"/>
  <circle cx="12" cy="12" r="3" fill="black"/>
</svg>
)";

void handleRoot()
{
  String html = R"rawliteral(
<!DOCTYPE html>
<html>

<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="/favicon.svg" type="image/svg+xml">
  <style>
    html {
      height: 97%;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 12px;
    }

    body {
      height: 100%;
      font-family: Consolas;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      background-color: #1b1a1a;
      color: #f0f0f0;
      border-radius: 10px;
    }

    h1 {
      display: flex;
      justify-content: center;
      align-items: center;
      font-size: 26px;
      color: rgb(151, 151, 151);
      text-shadow: 3px 3px 5px rgb(22, 22, 22);
      margin: 0px;
    }

    h3 {
      text-align: center;
      font-size: 22px;
      margin: 8px 0px 0px 0px;
    }

    .header {
      display: flex;
      align-items: center;
      gap: 16px;
      border-radius: 8px;
      background: linear-gradient(#011800, #2e2e2e, #3f3f3f, #2e2e2e, #2e2e2e, #011800);
    }

    .header-img {
      display: flex;
      justify-content: center;
      align-items: center;
      font-size: 40px;
      margin-left: 8px;
    }

    .container-wrapper {
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100%;
      width: 100%;
      position: relative;
    }

    .container {
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      height: 98%;
      width: 100%;
      max-width: 500px;
      position: relative;
      background: #011800;
      border-radius: 10px;
      padding: 10px;
    }

    @property --angle {
      syntax: '<angle>';
      inherits: false;
      initial-value: 0deg;
    }

    .container-wrapper::after,
    .container-wrapper::before {
      content: '';
      position: absolute;
      height: 100%;
      width: 100%;

      background-image: conic-gradient(from var(--angle),
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green,
          #011d01,
          green);

      top: 50%;
      left: 50%;
      translate: -50% -50%;
      z-index: -1;
      padding: 6px;
      border-radius: 14px;
      animation: 100s spin linear infinite;
    }

    .container-wrapper::before {
      filter: blur(1.5rem);
    }

    @keyframes spin {
      0% {
        --angle: 0deg;
      }

      100% {
        --angle: 360deg;
      }
    }

    .btn {
      padding: 10px 15px;
      margin: 3px 0px 3px 0px !important;
      font-size: 16px;
      background: #075709;
      color: white;
      border: none;
      border-radius: 4px;
    }

    button {
      cursor: pointer;
      transition: all 50ms ease-in-out;

      &:active {
        transform: scale(0.9);
      }

      &:hover {
        cursor: pointer;
      }
    }

    .btn-stop {
      padding: 15px;
      margin: 5px;
      font-size: 22px;
      color: white;
      border: 3px solid #ffffff;
      border-radius: 50%;
      background: #f44336;
      width: 60px;
      height: 60px;
      display: flex;
      justify-content: center;
      align-items: center;
      box-shadow: 0 0 10px 0 #f44336 inset, 0 0 10px 4px #f44336;
      text-shadow: 3px 3px 5px rgb(3, 39, 0);
    }

    .btn-start {
      font-size: 24px;
      font-weight: bold;
      border-radius: 50%;
      width: 80px;
      height: 80px;
      border-color: #2ecc71;
      color: #fff;
      box-shadow: 0 0 10px 0 #2ecc71 inset, 0 0 10px 4px #2ecc71;
      text-shadow: 3px 3px 5px rgb(3, 39, 0);

      &:active {
        transform: scale(0.9);
      }
    }

    .status {
      padding: 10px;
      text-align: center;
      margin-top: 2px;
    }

    .form-group {
      flex-grow: 1;
      margin: 4px 0;
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 8px;
      width: fit-content;
    }

    .form-group label {
      margin-right: 16px;
    }

    .stack-settings-form {
      gap: 4px;
      display: flex;
      flex-direction: column;
      align-items: start;
      min-width: 62%;
    }

    label {
      white-space: nowrap;
      font-size: 18px;
      display: inline-block;
      width: 150px;
      text-align: left;
    }

    input[type="number"],
    select {
      border: 2px solid green;
      border-radius: 4px;
      background: #222222;
      color: rgb(173, 255, 173);
      font-size: 16px;
    }

    input[type="number"] {
      width: 76px;
      height: 24px;
    }

    select {
      width: 84px;
      height: 30px;
    }

    .controls button {
      font-size: 14px;
      width: 98%;
    }

    .controls {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 4px;
      margin-top: 6px;
    }

    .controls>*:nth-child(odd) {
      /* Вибираємо непарні елементи (перший стовпчик) */
      justify-self: start;
      /* Вирівнюємо по лівому краю (за замовчуванням) */
    }

    .controls>*:nth-child(even) {
      /* Вибираємо парні елементи (другий стовпчик) */
      justify-self: end;
      /* Вирівнюємо по правому краю */
    }

    .position-form {
      max-height: 40px;
      display: flex;
      flex-grow: 1;
      justify-content: space-between;
      align-items: center;
      gap: 4px;
      margin: 0px;
      padding: 0px;
    }

    .main-controls {
      display: grid;
      justify-content: space-between;
      width: 100%;
      grid-template-columns: auto auto auto;
      margin-bottom: 6px;
    }

    .main-controls .btn {
      width: 110px;
    }

    .return_to_start {
      width: 80px;
      height: 26px;
      background: #222222;
      position: relative;
      border: 2px solid green;
      border-radius: 50px;
      box-shadow: inset 0px 1px 1px rgba(0, 0, 0, 0.5), 0px 1px 0px rgba(255, 255, 255, 0.2);

      &:after {
        content: 'OFF';
        color: white;
        position: absolute;
        right: 10px;
        z-index: 0;
        font: 12px/26px Arial, sans-serif;
        font-weight: bold;
      }

      &:before {
        content: 'ON';
        color: rgb(173, 255, 173);
        text-shadow: 0px 0px 6px rgba(180, 255, 184, 0.8);
        position: absolute;
        left: 10px;
        z-index: 0;
        font: 12px/26px Arial, sans-serif;
        font-weight: bold;
      }

      label {
        display: block;
        width: 34px;
        height: 20px;
        cursor: pointer;
        position: absolute;
        top: 3px;
        left: 3px;
        z-index: 1;
        background: #fcfff4;
        background: linear-gradient(top, #fcfff4 0%, #dfe5d7 40%, #b3bead 100%);
        border-radius: 50px;
        transition: all 0.4s ease;
        box-shadow: 0px 2px 5px 0px rgba(0, 0, 0, 0.3);
      }

      input[type=checkbox] {
        visibility: hidden;

        &:checked+label {
          left: 43px;
        }
      }
    }
  </style>
  <script>
    function updateStatus() {
      fetch('/status').then(r => r.json()).then(data => {
        let statusText = 'Position: ' + data.position.toFixed(2) + ' mm | State: ' + data.state;
        if (data.shooting) {
          statusText += ' | Progress: ' + data.photo_count + '/' + data.total_photos;
        }
        document.getElementById('status').innerHTML = statusText;
      }); setTimeout(updateStatus, 2000);
    }
    window.onload = updateStatus;
    function startShooting() {
      const photos = document.getElementById('photos').value;
      const step = document.getElementById('step').value;
      const speed = document.getElementById('speed').value;
      const beforeShoot = document.getElementById('before_shoot').value;
      const shutterSpeed = document.getElementById('shutter_speed').value;
      const focusTime = document.getElementById('focus_time').value;
      const releaseTime = document.getElementById('release_time').value;
      const returnToStartCheckbox = document.getElementById('return_to_start');
      const returnToStart = returnToStartCheckbox.checked ? "1" : "0"; // 1 если включен, 0 если выключен
      fetch('/start?photos=' + photos + '&step=' + step + '&speed=' + speed + '&before=' + beforeShoot + '&after=' + shutterSpeed + '&focus_time=' + focusTime + '&release_time=' + releaseTime + '&return_to_start=' + returnToStart);
      return false;
    }
    function moveRelative(offset) {
      fetch('/move?offset=' + offset);
    }
    function updateEndstop() {
      fetch('/endstop').then(r => r.text()).then(t => {
        document.getElementById('endstop-status').innerHTML =
          'Endstop: ' + (t === '1' ? 'PRESSED' : 'released');
      });
      setTimeout(updateEndstop, 2000);
    }
    updateEndstop();
  </script>
</head>

<body>
  <div class="container-wrapper">
    <div class="container">
      <div class="header">
        <div class="header-img">
  <svg fill="rgb(151, 151, 151)" height="40px" width="40px" version="1.1" id="Layer_1" xmlns="http://www.w3.org/2000/svg"
    xmlns:xlink="http://www.w3.org/1999/xlink" viewBox="0 0 399.9 399.9" xml:space="preserve">
    <g id="SVGRepo_bgCarrier" stroke-width="0"></g>
    <g id="SVGRepo_tracerCarrier" stroke-linecap="round" stroke-linejoin="round"></g>
    <g id="SVGRepo_iconCarrier">
      <g>
        <g>
          <path
            d="M366.5,89.1h-24.1l-23.2-50.3c-1.8-3.9-5.8-6.5-10.1-6.5H201.7c-4.3,0-8.3,2.5-10.1,6.5l-23.2,50.3h-49.9V62.4 c0-6.1-5-11.1-11.1-11.1H50.2c-6.1,0-11.1,5-11.1,11.1v26.7h-5.8c-18.4,0-33.3,15-33.3,33.3v211.9c0,18.4,15,33.3,33.3,33.3h333.3 c18.4,0,33.3-15,33.3-33.3V122.4C399.8,104.1,384.8,89.1,366.5,89.1z M208.8,54.6H302l15.9,34.5H192.8L208.8,54.6z M61.2,73.5h35 v15.6h-35V73.5z M366.5,345.4H33.1c-6.1,0-11.1-5-11.1-11.1V227h17.3c6.1,0,11.1-5,11.1-11.1c0-6.1-5-11.1-11.1-11.1H22v-22.2 h39.5c6.1,0,11.1-5,11.1-11.1c0-6.1-5-11.1-11.1-11.1H22v-37.9c0-6.1,5-11.1,11.1-11.1h333.3c6.1,0,11.1,5,11.1,11.1v211.8h0.1 C377.6,340.4,372.6,345.4,366.5,345.4z">
          </path>
        </g>
      </g>
      <g>
        <g>
          <path
            d="M255.4,130.8c-53.8,0-97.6,43.8-97.6,97.6s43.8,97.6,97.6,97.6c53.8,0,97.6-43.8,97.6-97.6 C352.9,174.6,309.1,130.8,255.4,130.8z M255.4,303.7c-41.5,0-75.3-33.8-75.3-75.3s33.8-75.3,75.3-75.3s75.3,33.8,75.3,75.3 C330.7,269.9,296.9,303.7,255.4,303.7z">
          </path>
        </g>
      </g>
      <g>
        <g>
          <path
            d="M255.4,175.3c-29.3,0-53.1,23.8-53.1,53.1s23.8,53.1,53.1,53.1c29.3,0,53.1-23.8,53.1-53.1 C308.5,199.1,284.6,175.3,255.4,175.3z M255.4,259.3c-17,0-30.9-13.9-30.9-30.9s13.9-30.9,30.9-30.9s30.9,13.9,30.9,30.9 S272.4,259.3,255.4,259.3z">
          </path>
        </g>
      </g>
      <g>
        <g>
          <path
            d="M353.8,127.8h-9.9c-6.1,0-11.1,5-11.1,11.1c0,6.1,5,11.1,11.1,11.1h9.9c6.1,0,11.1-5,11.1-11.1 C364.9,132.8,360,127.8,353.8,127.8z">
          </path>
        </g>
      </g>
      <g>
        <g>
          <path
            d="M117.2,138.8c-6.1,0-11.1,5-11.1,11.1v156.9c0,6.1,5,11.1,11.1,11.1c6.1,0,11.1-5,11.1-11.1V149.9 C128.3,143.8,123.3,138.8,117.2,138.8z">
          </path>
        </g>
      </g>
    </g>
  </svg>
</div>
        <h1>Macro Rail Controller</h1>
      </div>
      <div class="main-controls">
        <button class="btn" onclick="fetch('/home')">Home</button>
        <button class="btn-stop" onclick="fetch('/stop')">Stop</button>
        <button class="btn" onclick="fetch('/reset')">Reset Error</button>
      </div>
      <form class="position-form" onsubmit="fetch('/move?pos='+document.getElementById('pos').value);return false;">
        <label for="pos">Position (mm):</label>
        <input type="number" step="0.01" id="pos" placeholder="mm" required style="height: 32px;" min="0" max="97.0">
        <button type="submit" class="btn" style="width: 110px;">Move to</button>
      </form>
      <div class="controls">
        <button onclick="moveRelative(-0.01)" class="btn">-0.01</button>
        <button onclick="moveRelative(0.01)" class="btn">+0.01</button>
        <button onclick="moveRelative(-0.1)" class="btn">-0.1</button>
        <button onclick="moveRelative(0.1)" class="btn">+0.1</button>
        <button onclick="moveRelative(-1)" class="btn">-1</button>
        <button onclick="moveRelative(1)" class="btn">+1</button>
      </div>
      <h3>Stack Settings</h3>
      <form onsubmit="return startShooting()"
        style="display: flex; align-items: center; justify-content: space-between; gap: 12px;">
        <div class="stack-settings-form">
          <div class="form-group"><label for="photos">Photo count:</label>
            <input type="number" id="photos" value="3" min="1">
          </div>
          <div class="form-group"><label for="step">Step size mm:</label>
            <input type="number" step="0.01" id="step" value="0.30" min="0.00">
          </div>
          <div class="form-group"><label for="speed">Speed mm/s:</label>
            <input type="number" step="0.01" id="speed" value="%f" min="0.01">
          </div>
          <div class="form-group"><label for="before_shoot">Before shoot ms:</label>
            <input type="number" id="before_shoot" value="%d" min="0">
          </div>
          <div class="form-group"><label for="shutter_speed">Shutter speed</label>
            <select id="shutter_speed">
              <option value="1">>=1000</option>
              <option value="2">>500</option>
              <option value="3">400</option>
              <option value="4">>250</option>
              <option value="5">200</option>
              <option value="7">160</option>
              <option value="8">125</option>
              <option value="10">100</option>
              <option value="13">80</option>
              <option value="17">60</option>
              <option value="20">50</option>
              <option value="25">40</option>
              <option value="34">30</option>
              <option value="40">25</option>
              <option value="50">20</option>
              <option value="67">15</option>
              <option value="77">13</option>
              <option value="100">10</option>
              <option value="125">8</option>
              <option value="167">6</option>
              <option value="200">5</option>
              <option value="250">4</option>
              <option value="334">3</option>
              <option value="400">2.5</option>
              <option value="500">2</option>
              <option value="625">1.6</option>
              <option value="770">1.3</option>
              <option value="1000">1''</option>
              <option value="1300">1.3''</option>
              <option value="1600">1.6''</option>
              <option value="2000">2''</option>
              <option value="2500">2.5''</option>
              <option value="3000">3''</option>
              <option value="4000">4''</option>
              <option value="5000">5''</option>
              <option value="6000">6''</option>
              <option value="8000">8''</option>
              <option value="10000">10''</option>
              <option value="13000">13''</option>
              <option value="15000">15''</option>
              <option value="20000">20''</option>
              <option value="25000">25''</option>
              <option value="30000">30''</option>
            </select>
          </div>
          <div class="form-group"><label for="focus_time">Focus time ms:</label>
            <input type="number" id="focus_time" value="%d" min="0">
          </div>
          <div class="form-group"><label for="release_time">Release time ms:</label>
            <input type="number" id="release_time" value="%d" min="0">
          </div>
          <div class="form-group"><label for="return_to_start">Return to start</label>
            <section title=".return_to_start">
              <div class="return_to_start">
                <input type="checkbox" value="" id="return_to_start" name="check" unchecked />
                <label for="return_to_start"></label>
              </div>
            </section>
          </div>
        </div>
        <div style="display: flex; align-items: center; justify-content: center;">
          <button type="submit" class="btn btn-start">Start</button>
        </div>
      </form>
      <div class="status">
        <div id="status">Loading...</div>
        <div id="endstop-status">Endstop: </div>
      </div>
      <div id="progress"></div>
    </div>
  </div>
</body>

</html>
    )rawliteral";

  MacroRail::Settings currentSettings = rail.get_settings();
  String formattedHtml = String(html.c_str());
  formattedHtml.replace("%f", String(currentSettings.max_speed, 2));
  formattedHtml.replace("%d", String(currentSettings.before_shoot_delay));
  formattedHtml.replace("%d", String(currentSettings.focus_time));
  formattedHtml.replace("%d", String(currentSettings.release_time));

  // Добавим выбор текущей выдержки в выпадающем списке
  formattedHtml.replace("<option value=\"" + String(currentSettings.after_shoot_delay) + "\">", "<option value=\"" + String(currentSettings.after_shoot_delay) + "\" selected>");

  server.send(200, "text/html", formattedHtml);
}

void handleFavicon()
{
  server.send(200, "image/svg+xml", favicon);
}

void handleStatus()
{
  JsonDocument doc;
  doc["position"] = rail.get_position();
  doc["target"] = rail.get_target_position(); // Используем публичный метод
  doc["steps"] = rail.get_current_steps();    // Используем публичный метод
  switch (rail.get_state())
  {
  case MacroRail::IDLE:
    doc["state"] = "Ready";
    break;
  case MacroRail::HOMING:
    doc["state"] = "Homing";
    break;
  case MacroRail::MOVING:
    doc["state"] = "Moving";
    break;
  case MacroRail::SHOOTING:
    doc["state"] = "Shooting";
    break;
  case MacroRail::ERROR:
    doc["state"] = "ERROR";
    break;
  default:
    doc["state"] = "Unknown";
  }
  doc["photo_count"] = rail.get_photo_count();
  doc["total_photos"] = rail.get_settings().total_photos;
  doc["shooting"] = rail.get_state() == MacroRail::SHOOTING;

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void setup()
{
  Serial.begin(115200);

  Serial.println("\nConnecting to Wi-Fi...");

  int networkIndex = 0;
  while (wifiNetworks[networkIndex].ssid != nullptr)
  {
    Serial.print("Trying to connect to: ");
    Serial.println(wifiNetworks[networkIndex].ssid);

    WiFi.begin(wifiNetworks[networkIndex].ssid, wifiNetworks[networkIndex].password);

    // Попытка подключения в течение определенного времени
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    { // Попытка в течение 10 секунд (20 * 500ms)
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\nConnected to: " + String(wifiNetworks[networkIndex].ssid));
      Serial.println("IP address: " + WiFi.localIP().toString());
      break; // Выходим из цикла, если подключение успешно
    }
    else
    {
      Serial.println("\nConnection failed.");
      networkIndex++; // Переходим к следующей сети
    }
  }

  // Настройка сервера
  server.on("/", handleRoot);
  server.on("/favicon.svg", handleFavicon);
  server.on("/status", handleStatus);
  server.on("/home", []()
            {
        rail.start_homing();
        server.send(200, "text/plain", "Homing started"); });
  server.on("/stop", []()
            {
        rail.stop();
        server.send(200, "text/plain", "Stopped"); });
  server.on("/move", []()
            {
        if (server.hasArg("pos")) {
            rail.move_to(server.arg("pos").toFloat());
            server.send(200, "text/plain", "Moving to absolute position");
        } else if (server.hasArg("offset")) {
            float offset = server.arg("offset").toFloat();
            float currentPosition = rail.get_position();
            float newPosition = currentPosition + offset;
            rail.move_to(newPosition); // Используем существующую функцию, она сама проверит границы
            server.send(200, "text/plain", "Moving by offset");
        } else {
            server.send(400, "text/plain", "Invalid move request");
        } });
  server.on("/start", []()
            {
    MacroRail::Settings settings = rail.get_settings();
    if (server.hasArg("photos")) settings.total_photos = server.arg("photos").toInt();
    if (server.hasArg("step")) settings.step_size = server.arg("step").toFloat();
    if (server.hasArg("speed")) settings.max_speed = server.arg("speed").toFloat();
    if (server.hasArg("before")) settings.before_shoot_delay = server.arg("before").toInt();
    if (server.hasArg("after")) settings.after_shoot_delay = server.arg("after").toInt();
    if (server.hasArg("focus_time")) settings.focus_time = server.arg("focus_time").toInt();
    if (server.hasArg("release_time")) settings.release_time = server.arg("release_time").toInt();
    if (server.hasArg("return_to_start")) {
        returnToStartEnabled = (server.arg("return_to_start") == "1");
    } else {
        returnToStartEnabled = false; // По умолчанию выключено
    }

    startPosition = rail.get_current_position(); // Запоминаем стартовую позицию
    rail.start_shooting(settings);
    server.send(200, "text/plain", "Shooting started"); });

  server.on("/reset", []()
            {
        rail.reset_emergency();
        server.send(200, "text/plain", "System reset"); });
  server.on("/endstop", []()
            { server.send(200, "text/plain",
                          digitalRead(ENDSTOP_PIN) == ENDSTOP_ACTIVE ? "1" : "0"); });
  server.begin();

  rail.start_homing();
}

void loop()
{
  rail.update();

  unsigned long current_time = millis();
  if (current_time - last_server_handle_time >= server_handle_interval)
  {
    server.handleClient();
    last_server_handle_time = current_time;
  }
}