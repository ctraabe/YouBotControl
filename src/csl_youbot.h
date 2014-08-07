#ifndef CSL_YOUBOT_H_
#define CSL_YOUBOT_H_

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

class CSLYouBot
{
  public:
    CSLYouBot(bool use_arm = true);

    void GetBasePosition(quantity<si::length> &x, quantity<si::length> &y,
      quantity<plane_angle> &psi)
    {
      youbot_base_.getBasePosition(x, y, psi);
    }
    void SetBaseVelocity(const quantity<si::velocity> &u,
      const quantity<si::velocity> &v,
      const quantity<si::angular_velocity> &r)
    {
      youbot_base_.setBaseVelocity(u, v, r);
    }

    quantity<plane_angle> GetJointAngle(const int &joint);
    void SetJointAngle(const int &joint, const quantity<plane_angle> &angle);
    void WaitForArm(volatile int &received_sigterm);

    quantity<si::length> GetGripperSpacing();
    void SetGripperSpacing(const quantity<si::length> &spacing);
    void WaitForGripper(volatile int &received_sigterm);

  private:
    bool use_arm_;
    youbot::YouBotBase youbot_base_;
    youbot::YouBotManipulator youbot_arm_;
    youbot::JointAngleSetpoint joint_angle_setpoint_[5];
    youbot::GripperBarSpacingSetPoint gripper_spacing_setpoint_;
};

#endif // CSL_YOUBOT_H_