
#ifndef _KEYBOARD_CTRL_H
#define _KEYBOARD_CTRL_H

#define key_ESC 27

void init_keyboard();
void close_keyboard();
int kbhit();
int readch();

#endif
