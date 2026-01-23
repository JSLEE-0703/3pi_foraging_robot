#ifndef _PID_H
#define _PID_H

// --------------------------------------------
//  PID 控制器类（标准化、逻辑一致版本）
// --------------------------------------------
class PID_c {
  public:

    // ---- PID 内部变量 ----
    float last_error;
    float p_term;
    float i_term;
    float d_term;
    float i_sum;
    float feedback;

    // ---- PID 增益 ----
    float p_gain;
    float i_gain;
    float d_gain;

    // ---- 输出极性 (1 = 正常系统, -1 = 反向系统) ----
    int polarity;

    // ---- 时间记录 ----
    unsigned long ms_last_t;

    // ---- 构造函数 ----
    PID_c() {
      polarity = 1;  // 默认正向系统
    }

    // ---- 初始化 ----
    void initialise(float p, float i, float d, int pol = 1) {
      feedback = 0;
      last_error = 0;
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;

      p_gain = p;
      i_gain = i;
      d_gain = d;

      polarity = pol;  // 保存输出极性

      ms_last_t = millis();
    }

    // ---- 重置 PID 状态 ----
    void reset() {
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;
      last_error = 0;
      feedback = 0;
      ms_last_t = millis();
    }

    // ---- 主更新函数 ----
    float update(float demand, float measurement) {
      unsigned long ms_now_t = millis();
      unsigned long ms_dt = ms_now_t - ms_last_t;
      ms_last_t = ms_now_t;

      float float_dt = (float)ms_dt;
      if (float_dt == 0.0f) return feedback;

      // 1️⃣ 计算误差
      float error = demand - measurement;

      // 2️⃣ 比例项
      p_term = p_gain * error;

      // 3️⃣ 积分项
      i_sum += error * float_dt;
      i_term = i_gain * i_sum;

      // 4️⃣ 微分项
      float diff_error = (error - last_error) / float_dt;
      d_term = d_gain * diff_error;

      last_error = error;

      // 5️⃣ 输出合成 + 极性控制
      feedback = polarity * (p_term + i_term + d_term);

      return feedback;
    }

}; // end of class

#endif
