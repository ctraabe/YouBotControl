#include "csl_youbot.h"

CSLYouBot::CSLYouBot()
  : youbot_base_("youbot-base", YOUBOT_CONFIGURATIONS_DIR),
    youbot_arm_("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR)
{
  youbot_base_.doJointCommutation();

  quantity<angular_acceleration> angAcc;
  youbot::MotorAcceleration acceleration;
  acceleration.setParameter( angAcc.from_value(100000.0) );

  youbot_arm_.doJointCommutation();
  youbot_arm_.calibrateManipulator();
}

void CSLYouBot::SetJointAngle(const int joint,
  const quantity<plane_angle>& value)
{
  joint_angle_setpoint_[joint-1].angle = value;
  youbot_arm_.getArmJoint(joint).setData(joint_angle_setpoint_[joint-1]);
}

quantity<plane_angle> CSLYouBot::GetJointAngle(const int joint)
{
  youbot::JointSensedAngle joint_angle_sensed;
  youbot_arm_.getArmJoint(joint).getData(joint_angle_sensed);
  return joint_angle_sensed.angle;
}

void CSLYouBot::WaitForArm(volatile int &received_sigterm)
{
// TODO: put this somewhere else
#define kJointErrorTolerance (0.01 * radian)
  quantity<plane_angle> joint_error_max;
  do {
    SLEEP_MILLISEC(50);

    joint_error_max = 0.0 * radian;
    for (int joint = 1; joint <= ARMJOINTS; ++joint)
    {
      joint_error_max = max(abs(GetJointAngle(joint)
        - joint_angle_setpoint_[joint-1].angle), joint_error_max);
    }
  } while (joint_error_max > kJointErrorTolerance && !received_sigterm);
}
