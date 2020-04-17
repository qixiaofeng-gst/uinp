#include <stdbool.h>

#include <termios.h>

void
touchTerminal(bool isRestore)
{
    static struct termios backup;
    if (isRestore) {
        tcsetattr(0, TCSAFLUSH, &backup);
    } else {
        tcgetattr(0, &backup);
    }
}

void
turnOffEcho()
{
    struct termios termInformation;
    tcgetattr(0, &termInformation);
    termInformation.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSAFLUSH, &termInformation);
}
