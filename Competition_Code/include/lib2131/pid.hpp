
#pragma once

namespace lib2131
{
class PID
{
 private:
  float kP, kI, kD;

  float P, I, D;
  float lastP, lastI, lastD;

 public:
  PID();
  PID(float KP, float KI, float KD);

  float calc(float error, int dTime);
};
}  // namespace lib2131
