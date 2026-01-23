#include "Encoders.h"
#include "Kinematics.h"
#include "Motors.h"
#include "PID.h"
#include "LineSensors.h"
#include "Magnetometer.h"
#include "lcd.h"        //LCD 支持
// ------------------------------

// === 全局对象 ===
Kinematics_c pose;
Motors_c motors;
PID_c heading_pid;
PID_c distance_pid;
LineSensors_c linesensors;
Magnetometer_c mag;
LCD_c display(0,1,14,17,13,30);   // LCD 引脚定义

// === 时间参数 ===
const unsigned long ODOM_PERIOD_MS = 20;
const unsigned long CTRL_PERIOD_MS = 20;
const unsigned long MAG_PERIOD_MS  = 100;
unsigned long last_odom_ms = 0;
unsigned long last_ctrl_ms = 0;
unsigned long last_mag_ms  = 0;
unsigned long start_time_ms = 0;   // 计时4分钟

const float PI_F = 3.14159265359f;

// === 控制参数 ===
const float ANGLE_TOL = 0.05;
const float DIST_TOL  = 10.0;
const float MAX_DRIVE_PWM = 12.0;
const float MIN_DRIVE_PWM = 8.0;

// === 线传感器参数（仅在 PUSH 阶段启用） ===
#define BLACK_THR 0.6
bool line_sensor_enabled = false;

// === 磁铁检测参数 ===
bool magnet_detected = false;
unsigned long push_start_ms = 0;
bool pushing = false;

// === 推球黑线延时参数 ===
const unsigned long PUSH_EXTRA_TIME_MS = 200;  // 黑线检测后继续前推时间
bool push_after_line = false;                  // 是否正在执行额外前推
unsigned long push_extra_start = 0;            // 额外前推计时起点

// === 状态定义 ===
enum TravelStage { STAGE_IDLE, STAGE_TURNING, STAGE_DRIVING, STAGE_DONE };
TravelStage travel_stage = STAGE_IDLE;

// === 目标点 ===
float target_x = 0, target_y = 0, target_theta = 0;

// === Foraging 状态 ===
enum ForageStage { SEARCH, PUSH, RETURN, WAIT };
ForageStage forage_stage = SEARCH;

// === 工具函数 ===
float wrapPi(float a) {
  while (a <= -PI_F) a += 2.0f * PI_F;
  while (a >  PI_F) a -= 2.0f * PI_F;
  return a;
}
float angleDiff(float target, float current) {
  return wrapPi(target - current);
}
void updateMinMax(LineSensors_c &ls) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (ls.readings[i] < ls.minimum[i]) ls.minimum[i] = ls.readings[i];
    if (ls.readings[i] > ls.maximum[i]) ls.maximum[i] = ls.readings[i];
  }
}

// === 转向 ===
bool rotateToAngle(float target_theta) {
  float ang_err = wrapPi(target_theta - pose.theta);
  float w = heading_pid.update(0, ang_err);
  if (fabs(w) < MIN_DRIVE_PWM) w = copysign(MIN_DRIVE_PWM, w);
  w = constrain(w, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
  motors.setPWM(-w, w);
  if (fabs(ang_err) < ANGLE_TOL) {
    motors.setPWM(0, 0);
    return true;
  }
  return false;
}

// === 前进 ===
bool moveToXY(float x, float y) {
  float dx = x - pose.x;
  float dy = y - pose.y;
  float dist_err = sqrt(dx * dx + dy * dy);
  float target_th = atan2(dy, dx);
  float ang_err = angleDiff(target_th, pose.theta);

  if (dist_err < DIST_TOL) {
    motors.setPWM(0, 0);
    return true;
  }

  float v = distance_pid.update(0, -dist_err);
  float w = heading_pid.update(0, ang_err);
  float w_gain = constrain(dist_err / 200.0, 0.2, 1.0);
  w *= w_gain;

  if (fabs(v) < MIN_DRIVE_PWM) v = copysign(MIN_DRIVE_PWM, v);
  if (fabs(w) < MIN_DRIVE_PWM) w = copysign(MIN_DRIVE_PWM, w);
  v = constrain(v, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
  w = constrain(w, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);

  float l_pwm = v - w;
  float r_pwm = v + w;
  motors.setPWM(l_pwm, r_pwm);
  return false;
}

// === 高层导航 ===
bool travelToXY(float x, float y) {
  switch (travel_stage) {
    case STAGE_IDLE:
      target_x = x;
      target_y = y;
      target_theta = atan2(y - pose.y, x - pose.x);
      heading_pid.initialise(60.0, 0.5, 0.0, -1);
      distance_pid.initialise(0.5, 0.0, 0.0, 1);
      travel_stage = STAGE_TURNING;
      Serial.println("➡️ 开始旋转对准目标...");
      break;

    case STAGE_TURNING:
      if (rotateToAngle(target_theta)) {
        heading_pid.reset();
        distance_pid.reset();
        travel_stage = STAGE_DRIVING;
        Serial.println("✅ 角度对准，开始前进...");
      }
      break;

    case STAGE_DRIVING:
      if (moveToXY(target_x, target_y)) {
        travel_stage = STAGE_DONE;
        Serial.println("🎯 到达目标点!");
      }
      break;

    case STAGE_DONE:
      motors.setPWM(0, 0);
      return true;
  }
  return false;
}

// === setup ===
void setup() {
  Serial.begin(9600);
  delay(1000);

  setupEncoder0();
  setupEncoder1();
  motors.initialise();
  pose.initialise(0.0, 0.0, 0.0);
  linesensors.initialiseForADC();

  if (mag.initialise())
    Serial.println("🧭 磁力计初始化成功");
  else
    Serial.println("⚠️ 磁力计初始化失败");

  // === 黑线校准 ===
  Serial.println("📏 开始线传感器校准 (原地旋转360°)...");
  for (int i = 0; i < NUM_SENSORS; i++) {
    linesensors.minimum[i] = 1023;
    linesensors.maximum[i] = 0;
  }
  pose.initialise(0, 0, 0);
  motors.setPWM(15, -15);
  while (fabs(pose.theta) < 2 * PI_F) {
    pose.update();
    linesensors.readSensorsADC();
    updateMinMax(linesensors);
    delay(20);
  }
  motors.setPWM(0, 0);
  for (int i = 0; i < NUM_SENSORS; i++) {
    float diff = linesensors.maximum[i] - linesensors.minimum[i];
    if (diff < 1) diff = 1;
    linesensors.scaling[i] = 1.0f / diff;
  }
  Serial.println("✅ 线传感器校准完毕！");

  heading_pid.reset();
  distance_pid.reset();
  pushing = false;
  magnet_detected = false;
  line_sensor_enabled = false;

  start_time_ms = millis();
  Serial.println("✅ 初始化完成，准备搜索磁铁");

  // LCD 启动倒计时
  display.setMaxMinutes(4);
  display.startStopwatch();
  display.gotoXY(0, 0);
  display.print("⏱ Timer Start");
}

// === loop ===
void loop() {
  unsigned long now = millis();

  // 每秒刷新 LCD 倒计时
  display.timeRemaining();

  // 超时结束
  if (now - start_time_ms > 240000) {
    motors.setPWM(0, 0);
    Serial.println("⏰ 4分钟结束，停止任务");
    while (1);
  }

  // 更新里程计
  if (now - last_odom_ms >= ODOM_PERIOD_MS) {
    last_odom_ms += ODOM_PERIOD_MS;
    pose.update();
  }

  // 磁铁检测
  if (now - last_mag_ms >= MAG_PERIOD_MS) {
    last_mag_ms = now;
    mag.getReadings();
    float z_val = mag.readings[2];
    float abs_z = fabs(z_val);
    Serial.print("Mag Z = ");
    Serial.println(z_val);

    if (!magnet_detected && abs_z > 10500) {
      magnet_detected = true;
      Serial.println("🧲 磁铁检测到!");
      push_start_ms = millis();
      pushing = true;
      forage_stage = PUSH;
      travel_stage = STAGE_IDLE;

      // 启用线检测（推球边界检测）
      line_sensor_enabled = true;
      Serial.println("✅ 启用线检测（推球阶段）");
    } else if (magnet_detected && abs_z < 10200) {
      magnet_detected = false;
    }
  }

  // 控制逻辑
  if (now - last_ctrl_ms >= CTRL_PERIOD_MS) {
    last_ctrl_ms += CTRL_PERIOD_MS;

    static const float waypoints[][2] = {
      {270, -50}, {230, -255}, {310, -420},
      {167, -420}, {47, -400}, {80, -160}
    };
    static const int NUM_WP = sizeof(waypoints) / sizeof(waypoints[0]);
    static int current_wp = 0;
    static bool reached = false;

    switch (forage_stage) {

      case SEARCH:
        if (!reached) reached = travelToXY(waypoints[current_wp][0], waypoints[current_wp][1]);
        else {
          reached = false;
          motors.setPWM(0, 0);
          Serial.print("✅ Waypoint ");
          Serial.println(current_wp);
          delay(200);

          if (magnet_detected) break;  // 已进入 PUSH
          current_wp++;
          if (current_wp < NUM_WP) {
            Serial.print("➡️ 前往下一个点 ");
            Serial.println(current_wp);
            travel_stage = STAGE_IDLE;
          } else {
            Serial.println("🚫 所有点巡完 → RETURN");
            forage_stage = RETURN;
            travel_stage = STAGE_IDLE;
            current_wp = 0;
          }
        }
        break;

      case PUSH:
        if (pushing && line_sensor_enabled) {
          motors.setPWM(15, 15);
          linesensors.calcCalibratedADC();
          float left  = linesensors.calibrated[0];
          float mid   = linesensors.calibrated[2];
          float right = linesensors.calibrated[4];

          // 第一次检测到黑线时开始额外前推计时
          if (!push_after_line && (mid > BLACK_THR || left > BLACK_THR || right > BLACK_THR)) {
            push_after_line = true;
            push_extra_start = millis();
            Serial.println("🏁 检测到黑线，继续前推...");
          }

          // 到达额外前推时间后执行返回
          if (push_after_line && millis() - push_extra_start >= PUSH_EXTRA_TIME_MS) {
            motors.setPWM(0, 0);
            Serial.println("✅ 前推完成，进入 RETURN 阶段");
            pushing = false;
            push_after_line = false;
            line_sensor_enabled = false;
            forage_stage = RETURN;
            travel_stage = STAGE_IDLE;
          }
        }
        break;

      case RETURN:
        if (travelToXY(0, 0)) {
          motors.setPWM(0, 0);
          Serial.println("🏠 已返回起点，等待4秒");
          unsigned long wait_t0 = millis();
          while (millis() - wait_t0 < 4000) motors.setPWM(0, 0);

          // 返回起点后保持线检测禁用
          line_sensor_enabled = false;
          Serial.println("🚫 返回起点后线检测保持关闭");

          current_wp = 0;
          travel_stage = STAGE_IDLE;
          forage_stage = SEARCH;
        }
        break;

      case WAIT:
        break;
    }
  }
}
