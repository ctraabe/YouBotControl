#ifndef CSL_YOUBOT_H_
#define CSL_YOUBOT_H_

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

class CSLYouBot
{
  public:
    CSLYouBot();

    void SetJointAngle(const int joint, const quantity<plane_angle>& value);
    quantity<plane_angle> GetJointAngle(const int joint);

    void WaitForArm(volatile int &received_sigterm);

  private:
    youbot::YouBotBase youbot_base_;
    youbot::YouBotManipulator youbot_arm_;
    youbot::JointAngleSetpoint joint_angle_setpoint_[5];
};

#endif // CSL_YOUBOT_H_