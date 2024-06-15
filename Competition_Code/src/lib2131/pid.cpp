#include "lib2131/pid.hpp"

namespace lib2131
{
PID::PID() : kP(0), kI(0), kD(0), P(0), I(0), D(0), lastP(0), lastI(0), lastD(0) {}
PID::PID(float KP, float KI, float KD)
    : kP(KP), kI(KI), kD(KD), P(0), I(0), D(0), lastP(0), lastI(0), lastD(0)
{
}

float PID::calc(float error, int dTime)
{
  P = error;
  I += P * dTime;
  D = (P - lastP) / dTime;

  lastP = P;
  lastI = I;
  lastD = D;

  return P * kP + I * kI - D * kD;
}
}  // namespace lib2131
