
#pragma once

namespace lib2131
{
class PID
{
 private:
  double kP, kI, kD;

  double P, I, D;
  double lastP, lastI, lastD;

 public:
  PID();
  PID(double KP, double KI, double KD);

  double calc(double error, int dTime);
};
}  // namespace lib2131
