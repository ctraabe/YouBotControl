#include "start_menu.h"

#include <iostream>

#include <boost/thread/thread.hpp>

#include <signal.h>
#include <termios.h>

void StartMenu(volatile int &received_sigterm)
{
  struct termios oldt, newt;

  // Get current terminal attributes and save for later reversion.
  tcgetattr(STDIN_FILENO, &oldt);
  // Modify the terminal removing the following attributes:
  // ICANON: disable buffering so that each keystroke is pushed to stdin
  // ECHO: disable echoing of stdin to terminal
  // VINTR: disable generation of SIGINT on ctrl-C press (gets blocked)
  // VQUIT: disable generation of SIGQUIT on ctrl-\ press (gets blocked)
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO | VINTR | VQUIT);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  char keypress = 0;
  while (keypress != 's' && !received_sigterm)
  {
    std::cout << std::endl << "Commands:" << std::endl << std::endl;
    std::cout << "  c - Set the camera" << std::endl;
    std::cout << "  s - start the program" << std::endl;
    std::cout << "  a - abort" << std::endl << std::endl;

    // Read a character from stdin then flush the buffer.
    std::cin >> keypress;

    switch (keypress) {
      case 3:  // ctrl-C
      case 'a':
      case 'A':
      {
        raise(SIGINT);
        break;
      }
      case 28:
      {
        raise(SIGQUIT);
        break;
      }
      case 'c':
      case 'C':
      {
        // youbot::GripperBarSpacingSetPoint barSetPoint;
        // youbot::GripperSensedBarSpacing barSensed;

        // // Fully open gripper and wait for completion.
        // // (This step removes any backlash that may cause overestimate of width)
        // barSetPoint.barSpacing = 0.023 * meter;
        // youbot_arm->getArmGripper().setData(barSetPoint);
        // do {
        //   boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        //   youbot_arm->getArmGripper().getData(barSensed);
        // } while (barSensed.barSpacing < 0.023 * meter);

        // // Close the bar to just bigger than camera width.
        // barSetPoint.barSpacing = 0.0165 * meter;
        // youbot_arm->getArmGripper().setData(barSetPoint);
        // do {
        //   boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        //   youbot_arm->getArmGripper().getData(barSensed);
        // } while (barSensed.barSpacing > 0.0165 * meter);

        // // Wait for user input, then clamp.
        // std::cout << "Press any key to clamp..." << std::endl;
        // getchar();
        // barSetPoint.barSpacing = 0.014 * meter;
        // youbot_arm->getArmGripper().setData(barSetPoint);
        // do {
        //   boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        //   youbot_arm->getArmGripper().getData(barSensed);
        // } while (barSensed.barSpacing > 0.014 * meter);
        // std::cout << std::endl << "Clamped!" << std::endl;
        break;
      }
      default:
      {
        break;
      }
    }

    // Wait a moment to make sure signals get raised.
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }

  // Reset the terminal settings.
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}