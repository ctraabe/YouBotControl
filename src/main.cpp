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

#define MAIN_LOOP_FREQUENCY 100
#define COMMAND_TIMEOUT 0.5

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

  // User supplied variables and default values.
  bool use_arm = false, do_csl_comms = false, do_odemtry_comms = false;
  float arm_up[5] = {1.0, 1.0, -1.0, 1.0, 1.0},
    arm_down[5] = {0.11, 0.11, -0.11, 0.11, 0.12};
  string serial_csl_port("/dev/ttyUSB0");
  string serial_odometry_port("/dev/ttyUSB1");
  int serial_csl_baudrate = 38400, serial_odometry_baudrate = 38400;

  ReadConfigArm(use_arm, arm_up, arm_down);
  ReadConfigCSLSerial(do_csl_comms, serial_csl_port, serial_csl_baudrate);
  ReadConfigOdometrySerial(do_odemtry_comms, serial_odometry_port,
    serial_odometry_baudrate);

  // Vehicle wrapper serial comms.
  Serial serial_csl;
  if (do_csl_comms)
    serial_csl.Open(serial_csl_port, serial_csl_baudrate);

  // Odometry data output.
  Serial serial_odometry;
  if (do_odemtry_comms)
    serial_odometry.Open(serial_odometry_port, serial_odometry_baudrate);

  // Quit if failed to open either serial port.
  if (do_csl_comms && !serial_csl.IsOpen()
    || do_odemtry_comms && !serial_odometry.IsOpen())
  {
    return CloseSeiralAndReturn(serial_csl, serial_odometry, 1);
  }

  try
  {
    CSLYouBot csl_youbot(use_arm);

    if (use_arm)
    {
      // Make sure the arm is in the stowed position (can be canceled).
      for (int joint = 1; joint <= ARMJOINTS; ++joint)
        csl_youbot.SetJointAngle(joint, arm_down[joint-1] * radian);
      csl_youbot.WaitForArm(received_sigterm);

      // Clear the interrupt (user must push ctrl-C again to cancel program).
      received_sigterm = 0;
    }

    // Start menu (including camera clamping).
    StartMenuInitTerminal();
    enum StartMenuResult start_menu_result;
    do
    {
      start_menu_result = StartMenu(use_arm, received_sigterm);
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
    if (use_arm) {
      csl_youbot.SetJointAngle(5, arm_up[4] * radian);
      csl_youbot.WaitForArm(received_sigterm);

      csl_youbot.SetJointAngle(1, arm_up[0] * radian);
      csl_youbot.WaitForArm(received_sigterm);

      csl_youbot.SetJointAngle(2, arm_up[1] * radian);
      csl_youbot.SetJointAngle(3, arm_up[2] * radian);
      csl_youbot.SetJointAngle(4, arm_up[3] * radian);
      csl_youbot.WaitForArm(received_sigterm);
    }

    // Starting the main loop.
    while (!received_sigterm)
    {
      // Poll vehicle wrapper for commands and execute if received.
      VehiclePacket_t vehicle_packet;
      static double timeout = 999.9;
      if (do_csl_comms && ParseVehiclePacket(serial_csl, vehicle_packet))
      {
        try
        {
          csl_youbot.SetBaseVelocity(
            vehicle_packet.channel1 * meter_per_second,
            -vehicle_packet.channel2 * meter_per_second,
            vehicle_packet.channel3 * radian_per_second);
          timeout = 0.0;
        }
        catch(exception& ex)
        {
          cout << "BaseWHAT: " << ex.what() << endl;
        }
      }
      else
      {
        timeout += 1. / (double)MAIN_LOOP_FREQUENCY;
        if (timeout > COMMAND_TIMEOUT)
        {
          csl_youbot.SetBaseVelocity(0.0 * meter_per_second,
            0.0 * meter_per_second, 0.0 * radian_per_second);
        }
      }

      // Send health packet to vehicle wrapper.
      if (do_csl_comms) SendHealthPacket(serial_csl);

      quantity<si::length> longitudinalPosition, transversalPosition;
      quantity<plane_angle> orientation;
      csl_youbot.GetBasePosition(longitudinalPosition, transversalPosition,
        orientation);

      static float x = 0.0, y = 0.0, psi = 0.0;
      x = longitudinalPosition / meter;
      y = transversalPosition / meter;
      psi = orientation / radian;

      if (do_odemtry_comms) SendOdometryPacket(x, y, psi, serial_odometry);

      // Output command and odometry to screen @ 1 sec intervals
      static int counter = 1;
      if (!--counter)
      {
        // Output every second
        counter = MAIN_LOOP_FREQUENCY;
        cout << "pitch: " << vehicle_packet.channel1
          << ", roll: " << vehicle_packet.channel2
          << ", yaw: " << vehicle_packet.channel3 << endl;
        // cout << "x: " << x << " y: " << y << " psi: " << psi * 180.0 / 3.14159
        //   << endl;
      }

      SLEEP_MILLISEC(1000 / MAIN_LOOP_FREQUENCY);
    }

    csl_youbot.SetBaseVelocity(0.0 * meter_per_second, 0.0 * meter_per_second,
      0.0 * radian_per_second);

    // Clear the interrupt (user must push ctrl-C again to cancel stow).
    received_sigterm = 0;

    // Put the arm back in the stowed position.
    if (use_arm)
    {
      csl_youbot.SetJointAngle(2, arm_down[1] * radian);
      csl_youbot.SetJointAngle(3, arm_down[2] * radian);
      csl_youbot.SetJointAngle(4, arm_down[3] * radian);
      csl_youbot.WaitForArm(received_sigterm);

      csl_youbot.SetJointAngle(1, arm_down[0] * radian);
      csl_youbot.WaitForArm(received_sigterm);

      csl_youbot.SetJointAngle(5, arm_down[4] * radian);
      csl_youbot.WaitForArm(received_sigterm);
    }
  }
  catch (exception& ex)
  {
    cout << "Exception: " << ex.what() << endl;
  }

  return CloseSeiralAndReturn(serial_csl, serial_odometry, 0);
}
