#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "keyboard_ctrl.hpp"

#define key_ESC 27

using namespace std;

static struct termios initial_settings;
static struct termios new_settings;

static int peek_character = -1;

int kbhit() {
  char ch;

  int nread;

  if (peek_character != -1) {
    return 1;
  }

  new_settings.c_cc[VMIN] = 0;
  tcsetattr(0, TCSANOW, &new_settings);

  nread = read(0, &ch, 1);

  new_settings.c_cc[VMIN] = 1;

  tcsetattr(0, TCSANOW, &new_settings);

  if (nread == 1) {
    peek_character = ch;

    return 1;
  }

  return 0;
}

int readch() {
  char ch;

  if (peek_character != -1) {
    ch = peek_character;
    peek_character = -1;

    return ch;
  }

  read(0, &ch, 1);

  return ch;
}

void init_keyboard() {
  tcgetattr(0, &initial_settings);

  new_settings = initial_settings;
  new_settings.c_lflag &= ~ICANON;
  new_settings.c_lflag &= ~ECHO;
  new_settings.c_lflag &= ~ISIG;
  new_settings.c_cc[VMIN] = 1;
  new_settings.c_cc[VTIME] = 0;

  tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard() { tcsetattr(0, TCSANOW, &initial_settings); }
