#ifndef __KEY_H
#define __KEY_H
#include "sys.h"

#define KEY PAin(0)
#define KEY_ON 1
#define KEY_OFF 0
#define No_Action 0
#define Click 1
#define KEY2_STATE PCin(13)

void KEY_Init(void);
uint8_t User_Key_Scan(void);

#endif
