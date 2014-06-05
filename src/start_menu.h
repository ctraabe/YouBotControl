#ifndef START_MENU_H_
#define START_MENU_H_

enum StartMenuResult
{
  START_MENU_RESULT_ABORT,
  START_MENU_RESULT_CLAMP,
  START_MENU_RESULT_RUN,
};

void StartMenuInitTerminal();
enum StartMenuResult StartMenu(volatile int &received_sigterm);
void StartMenuRevertTerminal();

#endif // START_MENU_H_
