#include <stdbool.h>

#include <termios.h>

void
touch_terminal(bool isRestore)
{
    static struct termios backup;
    if (isRestore) {
        tcsetattr(0, TCSAFLUSH, &backup);
    } else {
        tcgetattr(0, &backup);
    }
}

void
turn_off_echo()
{
    struct termios termInformation;
    tcgetattr(0, &termInformation);
    termInformation.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSAFLUSH, &termInformation);
}
