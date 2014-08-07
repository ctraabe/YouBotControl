#include "start_menu.h"

#include <iostream>

#include <signal.h>
#include <termios.h>

// TODO: make this a class with private data
static struct termios oldt, newt;

void StartMenuInitTerminal()
{
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
}

enum StartMenuResult StartMenu(bool use_arm, volatile int &received_sigterm)
{
  std::cout << std::endl << "Commands:" << std::endl << std::endl;
  if (use_arm) std::cout << "  c - Set the camera" << std::endl;
  std::cout << "  s - start the program" << std::endl;
  std::cout << "  a - abort" << std::endl << std::endl;

  char keypress = 0;
  while (!received_sigterm)
  {
    // Read a character from stdin then flush the buffer.
    std::cin >> keypress;

    switch (keypress) {
      case 3:  // ctrl-C
      case 'a':
      case 'A':
        raise(SIGINT);
        return START_MENU_RESULT_ABORT;
        break;
      case 28:  // ctrl-\
        raise(SIGQUIT);
        return START_MENU_RESULT_ABORT;
        break;
      case 'c':
      case 'C':
        if (use_arm) return START_MENU_RESULT_CLAMP;
        break;
      case 's':
      case 'S':
        return START_MENU_RESULT_RUN;
        break;
      default:
        break;
    }
  }
}

void StartMenuRevertTerminal()
{
  // Reset the terminal settings.
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}