
#include <memory>
#include <utility>
#include <vector>

#include "lib2131/angle.hpp"
#include "lib2131/odom.hpp"
#include "lib2131/pid.hpp"
#include "lib2131/robot-state.hpp"
#include "lib2131/spline.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

namespace lib2131
{
#define MAX_VOLTAGE 12000
#define T_LOOK_AHEAD 5

class Robot
{
 private:
  // Drive Info
  double m_LastLeftDriveVel;
  double m_LastRightDriveVel;

  // Drive Constants
  double m_MaxAccel;
  double m_MaxDecel;
  double m_MaxVelocity;

 private:
  pros::MotorGroup *m_LeftDrive;
  pros::MotorGroup *m_RightDrive;

 private:
  // Odometry
  std::vector<Odometry> m_Odometry;

  // Distance PIDs
  PID m_AngularPID;
  PID m_LinearPID;

 public:
  Robot(double maxAccel, double maxDecel, double maxVelocity,
        std::vector<Odometry> odometry, PID angularPID, PID linearPID,
        pros::MotorGroup *leftDrive, pros::MotorGroup *rightDrive)
      : m_MaxAccel(maxAccel),
        m_MaxDecel(maxDecel),
        m_MaxVelocity(maxVelocity),
        m_Odometry(odometry),
        m_AngularPID(angularPID),
        m_LinearPID(linearPID),
        m_LeftDrive(leftDrive),
        m_RightDrive(rightDrive)
  {
  }

  RobotState getRobotState()
  {
    return m_Odometry[0].getRobotState();
    // TODO: Kolman Filters on m_DeadOdom and m_DrivenOdom, (IMU???)
  }

  void moveDistance(double inches, bool async)
  {
    std::cout << "MOVE DISTANCE" << std::endl;
    if (async)
    {
      pros::Task([=, this]() { moveDistance(inches, false); },
                 "Robot::moveDistance ASYNC");
    }
    else
    {
      RobotState Pose = this->getRobotState();
      Pose.position.x += inches * sin(Pose.position.z.getRadians());
      Pose.position.y += inches * cos(Pose.position.z.getRadians());
      std::cout << "TARGET: " << Pose.position << std::endl;

      // while (true)  // TODO: CHANGE THIS TO WHEN ROBOT IS NEAR DESIRED POSITION
      // {
      auto [leftVoltage, rightVoltage] = this->_goToRobotState(Pose, 10);

      m_LeftDrive->move_voltage(leftVoltage);
      m_RightDrive->move_voltage(leftVoltage);

      pros::delay(10);
      // }
    }
  }

  void turnDistance(Angle theta, bool async)
  {
    if (async)
    {
      // Start Motion in new thread
      pros::Task([=, this]() { turnToHeading(theta, false); },
                 "Robot::turnToHeading ASYNC");
    }
    else
    {
      RobotState Pose = this->getRobotState();  // Get Current Position
      Pose.position.z += theta;                 // Change Theta

      // while (true)  // TODO: CHANGE THIS TO WHEN ROBOT IS NEAR DESIRED POSITION
      // {
      auto [leftVoltage, rightVoltage] = this->_goToHeading(Pose.position.z, 10);

      m_LeftDrive->move_voltage(leftVoltage);
      m_RightDrive->move_voltage(leftVoltage);

      pros::delay(10);  // Don't Overload Processor
      // }
    }
  }

  void turnToHeading(Angle theta, bool async)
  {
    if (async)
    {
      // Start Motion in new thread
      pros::Task([=, this]() { turnToHeading(theta, false); },
                 "Robot::turnToHeading ASYNC");
    }
    else
    {
      RobotState Pose = this->getRobotState();  // Get Current Position
      Pose.position.z = theta;                  // Change Theta
      std::cout << "TARGET" << Pose.position << std::endl;

      // while (true)  // TODO: CHANGE THIS TO WHEN ROBOT IS NEAR DESIRED POSITION
      // {
      auto [leftVoltage, rightVoltage] = this->_goToHeading(Pose.position.z, 10);

      m_LeftDrive->move_voltage(leftVoltage);
      m_RightDrive->move_voltage(leftVoltage);

      pros::delay(10);  // Don't Overload Processor
      // }
    }
  }

  void moveToPoint(double x, double y, bool async)
  {
    if (async)
    {
      pros::Task([=, this]() { moveToPoint(x, y, false); }, "Robot::moveToPoint ASYNC");
    }
    else
    {
      RobotState Pose = this->getRobotState();
      Pose.position.x += x;
      Pose.position.y += y;

      // while (true)  // TODO: CHANGE THIS TO WHEN ROBOT IS NEAR DESIRED POSITION
      // {
      auto [leftVoltage, rightVoltage] = this->_goToRobotState(Pose, 10);

      m_LeftDrive->move_voltage(leftVoltage);
      m_RightDrive->move_voltage(leftVoltage);

      pros::delay(10);
      // }
    }
  }
  void turnToPoint(double x, double y, bool async)
  {
    if (async)
    {
      pros::Task([=, this]() { turnToPoint(x, y, false); }, "Robot::turnToPoint ASYNC");
    }
    else
    {
      RobotState Pose = this->getRobotState();
      Pose.position.z =
          Angle(-atan2(y - Pose.position.y, x - Pose.position.x) + M_PI_2, false);

      // while (true)  // TODO: CHANGE THIS TO WHEN ROBOT IS NEAR DESIRED POSITION
      // {
      auto [leftVoltage, rightVoltage] = this->_goToHeading(Pose.position.z, 10);

      m_LeftDrive->move_voltage(leftVoltage);
      m_RightDrive->move_voltage(leftVoltage);

      pros::delay(10);
      // }
    }
  }

  // TODO: FINISH MOVE TO POSE
  void moveToPose(RobotState target, int resolution, bool async)
  {
    if (async)
    {
      pros::Task([=, this]() { moveToPose(target, resolution, false); },
                 "Robot::moveToPose ASYNC");
    }
    else
    {
      RobotState pose = this->getRobotState();
      Spline path = Spline(pose, target, resolution);
      // while (true)  // TODO: CHANGE THIS TO WHEN ROBOT IS NEAR DESIRED POSITION
      // {
      RobotState target = path.calc(path.getNearestT(this->getRobotState().position) +
                                    T_LOOK_AHEAD / resolution);

      auto [leftVoltage, rightVoltage] = this->_goToRobotState(target, 10);

      m_LeftDrive->move_voltage(leftVoltage);
      m_RightDrive->move_voltage(leftVoltage);

      pros::delay(10);
      // }
    }
  }

  void turnToPose(RobotState target, bool async)
  {
    if (async)
    {
      pros::Task([=, this]() { turnToPose(target, false); }, "Robot::turnToPose ASYNC");
    }
    else
    {
      RobotState Pose = this->getRobotState();
      Pose.position.z = Angle(-atan2(target.position.y - Pose.position.y,
                                     target.position.x - Pose.position.x) +
                                  M_PI_2,
                              false);

      // while (true)  // TODO: CHANGE THIS TO WHEN ROBOT IS NEAR DESIRED POSITION
      // {
      auto [leftVoltage, rightVoltage] = this->_goToHeading(Pose.position.z, 10);

      m_LeftDrive->move_voltage(leftVoltage);
      m_RightDrive->move_voltage(leftVoltage);

      pros::delay(10);
      // }
    }
  }

  void brake(pros::MotorBrake brakeType)
  {
    m_LeftDrive->set_brake_mode_all(brakeType);
    m_RightDrive->set_brake_mode_all(brakeType);
  }

  void setPosition(RobotState newState)
  {
    // Update all Odometry Objects
    for (std::size_t i = 0; i < m_Odometry.size(); i++)
    {
      m_Odometry[i].setRobotState(newState);
    }
  }

  void update(int dTime)
  {
    for (size_t i = 0; i < m_Odometry.size(); i++)
    {
      m_Odometry[i].update(dTime);
    }
  }

 protected:
  /**
   * @brief Returns desired left and right Motor Voltage (mV) to turn the robot to a
   * heading
   *
   * @return std::pair<double, double>
   */
  std::pair<double, double> _goToHeading(Angle target, int dTime)
  {
    std::cout << "TARGET: " << target.getDegrees() << std::endl;

    RobotState actual = this->getRobotState();
    std::cout << "ACTUAL (xyɵ): " << actual.position << std::endl;

    double angularError = (target - actual.position.z).getRadians();
    double angularOut = m_AngularPID.calc(fmod(angularError, 2 * M_PI), dTime);
    std::cout << "Error: " << angularError << std::endl;
    std::cout << "Angular Out: " << angularOut << std::endl;

    double leftDriveVel = angularOut;
    double rightDriveVel = -1 * angularOut;

    std::cout << "Velocities (lr): " << leftDriveVel << ", " << rightDriveVel
              << std::endl;

    //* ANTI MOTOR SATURATION STUFFS
    //? If motors are going faster than feasibly possible
    if (leftDriveVel > m_MaxVelocity)  // Left side
    {
      rightDriveVel = m_MaxVelocity / leftDriveVel;  // Keep ratio with Left
      leftDriveVel = m_MaxVelocity;                  // Set Left to max
    }
    if (rightDriveVel > m_MaxVelocity)  // Right Side
    {
      leftDriveVel = m_MaxVelocity / rightDriveVel;  // Keep ratio with Right
      rightDriveVel = m_MaxVelocity;                 // Set Right to max
    }

    //? If motors are attempting to accelerate faster than possible
    if (leftDriveVel - m_LastLeftDriveVel > m_MaxAccel)  // Left Side
    {
      leftDriveVel *=
          (m_MaxAccel / (leftDriveVel - m_LastLeftDriveVel));  // Reduce Acceleration
      rightDriveVel *=
          (m_MaxAccel / (leftDriveVel - m_LastLeftDriveVel));  // Reduce Acceleration
    }
    if (rightDriveVel - m_LastRightDriveVel < m_MaxAccel)  // Right Side
    {
      leftDriveVel *=
          (m_MaxAccel / (rightDriveVel - m_LastRightDriveVel));  // Reduce Acceleration
      rightDriveVel *=
          (m_MaxAccel / (rightDriveVel - m_LastRightDriveVel));  // Reduce Acceleration
    }

    //? If motors are attempting to decelerate faster than possible
    if (leftDriveVel - m_LastLeftDriveVel > m_MaxDecel)  // Left Side
    {
      leftDriveVel *=
          (m_MaxDecel / (leftDriveVel - m_LastLeftDriveVel));  // Reduce Deceleration
      rightDriveVel *=
          (m_MaxDecel / (leftDriveVel - m_LastLeftDriveVel));  // Reduce Deceleration
    }
    if (rightDriveVel - m_LastRightDriveVel < m_MaxDecel)  // Right Side
    {
      leftDriveVel *=
          (m_MaxDecel / (rightDriveVel - m_LastRightDriveVel));  // Reduce Deceleration
      rightDriveVel *=
          (m_MaxDecel / (rightDriveVel - m_LastRightDriveVel));  // Reduce Deceleration
    }

    return {(leftDriveVel / m_MaxVelocity) * MAX_VOLTAGE,
            (rightDriveVel / m_MaxVelocity) * MAX_VOLTAGE};
  }

  //! FUNCTION ENDED STOP BEING BLIND
  /*
  ? ================================================================
  ? =================================================================
  ? ==================================================================
  ? ===================================================================
  */
  /**
   * @brief returns desired left and right Motor Voltage (mV) to move the Robot to a
   * RobotState
   *
   * @param Target Desired RobotState
   * @param dTime Change in time since last called
   * @return std::pair<double, double> Left Drive (mV), Right Drive (mV)
   */
  std::pair<double, double> _goToRobotState(RobotState Target, int dTime)
  {
    // Get Current Position
    RobotState actual = this->getRobotState();
    std::cout << "ACTUAL (xyɵ): " << actual.position << std::endl;

    // Linear Error (Distance)
    double linearError = sqrt(pow(Target.position.x - actual.position.x, 2) +
                              pow(Target.position.y - actual.position.y, 2));

    std::cout << "ATAN2: "
              << -atan2(Target.position.y - actual.position.y,
                        Target.position.x - actual.position.x) *
                         180 / M_PI +
                     90
              << ", X: " << Target.position.x - actual.position.x
              << ", Y: " << Target.position.y - actual.position.y << std::endl;

    // Calculate Angular Error (Distance)
    double angularError = -atan2(Target.position.y - actual.position.y,
                                 Target.position.x - actual.position.x) +
                          M_PI_2 - actual.position.z.getRadians();

    // is point behind Robot?
    bool backwards = ((angularError < -M_PI_2 && angularError >= -3 * M_PI / 2) ||
                      (angularError > M_PI_2 && angularError <= 3 * M_PI / 2));

    std::cout << " -90 > " << angularError * 180 / M_PI
              << " >= -270: " << (angularError < -M_PI_2 && angularError >= -3 * M_PI / 2)
              << std::endl;

    std::cout << " 90 < " << angularError * 180 / M_PI
              << " <= 270: " << (angularError > M_PI_2 && angularError <= 3 * M_PI / 2)
              << std::endl;

    // If point is behind Robot
    if (backwards)
    {
      linearError *= -1;     // Sign Distance
      angularError -= M_PI;  // Rotate Heading to Rear
    };

    std::cout << "Error (v,w): " << linearError << ", " << fmod(angularError, 2 * M_PI)
              << "\n";

    // Calculate Linear velocity and Angular Velocities (PID Controller)
    double linearOut = m_LinearPID.calc(linearError, dTime);
    double angularOut = m_AngularPID.calc(fmod(angularError, 2 * M_PI), dTime);

    // Prioritize Turns (cos PI/2 = 0)
    linearOut *= cos(angularError);
    std::cout << "Adjusted Linear (v): " << linearOut << ", Rev?: " << backwards
              << std::endl;

    // convert to Left and Right velocities (Tank Drive Kinematics)
    double leftDriveVel = linearOut + angularOut;
    double rightDriveVel = linearOut - angularOut;
    std::cout << "Velocities (lr): " << leftDriveVel << ", " << rightDriveVel << "\n";

    //* ANTI MOTOR SATURATION STUFFS
    //? If motors are going faster than feasibly possible
    if (leftDriveVel > m_MaxVelocity)  // Left side
    {
      rightDriveVel = m_MaxVelocity / leftDriveVel;  // Keep ratio with Left
      leftDriveVel = m_MaxVelocity;                  // Set Left to max
    }
    if (rightDriveVel > m_MaxVelocity)  // Right Side
    {
      leftDriveVel = m_MaxVelocity / rightDriveVel;  // Keep ratio with Right
      rightDriveVel = m_MaxVelocity;                 // Set Right to max
    }

    //? If motors are attempting to accelerate faster than possible
    if (leftDriveVel - m_LastLeftDriveVel > m_MaxAccel)  // Left Side
    {
      leftDriveVel *=
          (m_MaxAccel / (leftDriveVel - m_LastLeftDriveVel));  // Reduce Acceleration
      rightDriveVel *=
          (m_MaxAccel / (leftDriveVel - m_LastLeftDriveVel));  // Reduce Acceleration
    }
    if (rightDriveVel - m_LastRightDriveVel < m_MaxAccel)  // Right Side
    {
      leftDriveVel *=
          (m_MaxAccel / (rightDriveVel - m_LastRightDriveVel));  // Reduce Acceleration
      rightDriveVel *=
          (m_MaxAccel / (rightDriveVel - m_LastRightDriveVel));  // Reduce Acceleration
    }

    //? If motors are attempting to decelerate faster than possible
    if (leftDriveVel - m_LastLeftDriveVel < m_MaxDecel)  // Left Side
    {
      leftDriveVel *=
          (m_MaxDecel / (leftDriveVel - m_LastLeftDriveVel));  // Reduce Deceleration
      rightDriveVel *=
          (m_MaxDecel / (leftDriveVel - m_LastLeftDriveVel));  // Reduce Deceleration
    }
    if (rightDriveVel - m_LastRightDriveVel < m_MaxDecel)  // Right Side
    {
      leftDriveVel *=
          (m_MaxDecel / (rightDriveVel - m_LastRightDriveVel));  // Reduce Deceleration
      rightDriveVel *=
          (m_MaxDecel / (rightDriveVel - m_LastRightDriveVel));  // Reduce Deceleration
    }

    // Return (Left Velocity (%) * MAX_VOLTAGE, and same for Right)
    return {(leftDriveVel / m_MaxVelocity) * MAX_VOLTAGE,
            (rightDriveVel / m_MaxVelocity) * MAX_VOLTAGE};
  }
};
}  // namespace lib2131
