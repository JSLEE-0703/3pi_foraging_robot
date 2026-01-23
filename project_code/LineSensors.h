// 引脚映射（3pi+ 32U4）
#define NUM_SENSORS 5
const int sensor_pins[NUM_SENSORS] = { A11, A0, A2, A3, A4 };
#define EMIT_PIN 11

class LineSensors_c {
public:
  float readings[NUM_SENSORS];
  float minimum[NUM_SENSORS];
  float maximum[NUM_SENSORS];
  float scaling[NUM_SENSORS];
  float calibrated[NUM_SENSORS];

  void initialiseForADC() {
    // 打开红外发射灯
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, HIGH);

    // 关键：启用内部上拉，才能形成可测的分压
    for (int i = 0; i < NUM_SENSORS; i++) {
      pinMode(sensor_pins[i], INPUT_PULLUP);
    }
  }

  void readSensorsADC() {
    // 确保配置就绪（重复调用也安全）
    initialiseForADC();

    for (int i = 0; i < NUM_SENSORS; i++) {
      readings[i] = analogRead(sensor_pins[i]);  // 0..1023
    }
  }

  void calcCalibratedADC() {
    readSensorsADC();
    for (int i = 0; i < NUM_SENSORS; i++) {
      float raw = readings[i];
      float norm = (raw - minimum[i]) * scaling[i];
      if (norm < 0) norm = 0;
      if (norm > 1) norm = 1;
      calibrated[i] = norm;
    }
  }

  void calibrateRotate(unsigned long ms=4000) {
    // 初始化 min/max
    for (int i = 0; i < NUM_SENSORS; i++) {
      minimum[i] = 1023;
      maximum[i] = 0;
    }
    unsigned long t0 = millis();
    while (millis() - t0 < ms) {
      readSensorsADC();
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (readings[i] < minimum[i]) minimum[i] = readings[i];
        if (readings[i] > maximum[i]) maximum[i] = readings[i];
      }
      delay(20);
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
      float diff = maximum[i] - minimum[i];
      if (diff < 1) diff = 1;
      scaling[i] = 1.0f / diff;
    }
  }
};
