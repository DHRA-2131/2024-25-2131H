
#pragma once

namespace lib2131
{
class PID
{
 private:
  float m_kP, m_kI, m_kD;

  float P, I, D;
  float lastP, lastI, lastD;

 public:
  PID(float kP, float kI, float kD) : m_kP(kP), m_kI(kI), m_kD(kD) {}

  float calc(float error, int deltaTime)
  {
    P = error;
    I += (lastP + P) * deltaTime * 0.5;
    D = (lastP - P) / deltaTime;

    return P * m_kP + I * m_kI + D * m_kD;
  }
};
}  // namespace lib2131
