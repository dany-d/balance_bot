#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#define CFG_PATH "/home/debian/balancebot-f19/pid.cfg"

int mb_controller_init(double *D1_KP, double *D1_KI, double *D1_KD, double *D2_KP, double *D2_KI, double *D2_KD);
int mb_controller_update(mb_state_t* mb_state);
int mb_controller_cleanup();

#endif

