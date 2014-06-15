#include <iomanip>
#include <iostream>
#include <string>

#include <signal.h>
#include <unistd.h>

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

#include "csl_youbot.h"
#include "odometry_comms.h"
#include "read_csl_config.h"
#include "serial.h"
#include "start_menu.h"
#include "vehicle_wrapper_comms.h"

static volatile int received_sigterm = 0;
static volatile int received_nb_signals = 0;

static void sigterm_handler(int sig)
{
  received_sigterm = sig;
  received_nb_signals++;
  if (received_nb_signals > 4) exit(123);
}

static int CloseSeiralAndReturn(Serial &serial_csl, Serial &serial_odometry,
  const int &return_val)
{
  serial_csl.Close();
  serial_odometry.Close();
  return return_val;
}

void CameraClamp(CSLYouBot &csl_youbot)
{
  youbot::GripperBarSpacingSetPoint barSetPoint;
  youbot::GripperSensedBarSpacing barSensed;

  // Fully open gripper and wait for completion.
  // (This step removes any backlash that may cause overestimate of width)
  csl_youbot.SetGripperSpacing(0.023 * meter);
  csl_youbot.WaitForGripper(received_sigterm);

  // Close the bar to just bigger than camera width.
  csl_youbot.SetGripperSpacing(0.0165 * meter);
  csl_youbot.WaitForGripper(received_sigterm);

  // Wait for user input, then clamp.
  std::cout << "Press any key to clamp..." << std::endl;
  getchar();
  csl_youbot.SetGripperSpacing(0.012 * meter);
  csl_youbot.WaitForGripper(received_sigterm);
  std::cout << std::endl << "Clamped!" << std::endl;
}

int main()
{
  signal(SIGQUIT, sigterm_handler); /* Quit (POSIX).  */
  signal(SIGINT , sigterm_handler); /* Interrupt (ANSI).  */
  signal(SIGTERM, sigterm_handler); /* Termination (ANSI).  */

  float arm_up[5] = {1.0, 1.0, -1.0, 1.0, 1.0},
    arm_down[5] = {0.11, 0.11, -0.11, 0.11, 0.12};
  string serial_csl_port("/dev/ttyUSB0");
  string serial_odometry_port("/dev/ttyUSB1");

  ReadCSLConfig(arm_up, arm_down, serial_csl_port, serial_odometry_port);

  Serial serial_csl(serial_csl_port, 38400);  // vehicle wrapper serial comms
  Serial serial_odometry(serial_odometry_port, 38400);  // Odometry data output

  // Quit if failed to open either serial port.
  if (!serial_csl.IsOpen() || !serial_odometry.IsOpen())
    return CloseSeiralAndReturn(serial_csl, serial_odometry, 1);

  try
  {
    CSLYouBot csl_youbot;

    // Make sure the arm is in the stowed position (can be canceled).
    for (int joint = 1; joint <= ARMJOINTS; ++joint)
      csl_youbot.SetJointAngle(joint, arm_down[joint-1] * radian);
    csl_youbot.WaitForArm(received_sigterm);

    // Clear the interrupt (user must push ctrl-C again to cancel program).
    received_sigterm = 0;

    // Start menu (including camera clamping).
    StartMenuInitTerminal();
    enum StartMenuResult start_menu_result;
    do
    {
      start_menu_result = StartMenu(received_sigterm);
      if (start_menu_result == START_MENU_RESULT_CLAMP)
      {
        CameraClamp(csl_youbot);
      }
    } while (start_menu_result == START_MENU_RESULT_CLAMP);
    StartMenuRevertTerminal();

    //Quit here if abort was called during start menu.
    if (received_sigterm)
      return CloseSeiralAndReturn(serial_csl, serial_odometry, 0);

    // Put the arm up.
    csl_youbot.SetJointAngle(5, arm_up[4] * radian);
    csl_youbot.WaitForArm(received_sigterm);

    csl_youbot.SetJointAngle(1, arm_up[0] * radian);
    csl_youbot.WaitForArm(received_sigterm);

    csl_youbot.SetJointAngle(2, arm_up[1] * radian);
    csl_youbot.SetJointAngle(3, arm_up[2] * radian);
    csl_youbot.SetJointAngle(4, arm_up[3] * radian);
    csl_youbot.WaitForArm(received_sigterm);

    // Starting the main loop.
    while (!received_sigterm)
    {
      // Poll vehicle wrapper for commands and execute if received.
      VehiclePacket_t vehicle_packet;
      if (ParseVehiclePacket(serial_csl, vehicle_packet))
      {
        try
        {
          csl_youbot.SetBaseVelocity(
            -0.75 * ((float)vehicle_packet.pitch / 1250. - 1.) * meter_per_second,
            0.75 * ((float)vehicle_packet.roll / 1250. - 1.) * meter_per_second,
            0.5 * ((float)vehicle_packet.yaw / 1250. - 1.) * radian_per_second);
        }
        catch(exception& ex)
        {
          cout << "BaseWHAT: " << ex.what() << endl;
        }

        cout << "pitch: " << -((float)vehicle_packet.pitch / 1250. - 1.)
        << ", roll: " << ((float)vehicle_packet.roll / 1250. - 1.)
        << ", yaw: " << ((float)vehicle_packet.yaw / 1250. - 1.) << endl;
      }

      // Send health packet to vehicle wrapper.
      SendHealthPacket(serial_csl);

      quantity<si::length> longitudinalPosition, transversalPosition;
      quantity<plane_angle> orientation;
      csl_youbot.GetBasePosition(longitudinalPosition, transversalPosition,
        orientation);

      static float x = 0.0, y = 0.0, psi = 0.0;
      x = longitudinalPosition / meter;
      y = transversalPosition / meter;
      psi = orientation / radian;

      SendOdometryPacket(x, y, psi, serial_odometry);

      SLEEP_MILLISEC(10);
    }

    csl_youbot.SetBaseVelocity(0.0 * meter_per_second, 0.0 * meter_per_second,
      0.0 * radian_per_second);

    // Clear the interrupt (user must push ctrl-C again to cancel stow).
    received_sigterm = 0;

    csl_youbot.SetJointAngle(2, arm_down[1] * radian);
    csl_youbot.SetJointAngle(3, arm_down[2] * radian);
    csl_youbot.SetJointAngle(4, arm_down[3] * radian);
    csl_youbot.WaitForArm(received_sigterm);

    csl_youbot.SetJointAngle(1, arm_down[0] * radian);
    csl_youbot.WaitForArm(received_sigterm);

    csl_youbot.SetJointAngle(5, arm_down[4] * radian);
    csl_youbot.WaitForArm(received_sigterm);
  }
  catch (exception& ex)
  {
    cout << "Exception: " << ex.what() << endl;
  }

  return CloseSeiralAndReturn(serial_csl, serial_odometry, 0);
}
