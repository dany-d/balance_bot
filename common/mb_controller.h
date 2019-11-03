#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H

#include <rc/math/filter.h>
#include "mb_structs.h"
#define CFG_PATH "/home/debian/balancebot-f19/pid.cfg"

int mb_controller_init(rc_filter_t *g_D1_filter, rc_filter_t *g_D2_filter, rc_filter_t *g_D3_filter);
int mb_controller_cleanup(rc_filter_t *g_D1_filter, rc_filter_t *g_D2_filter, rc_filter_t *g_D3_filter);
int mb_controller_update(mb_state_t *mb_state, mb_setpoints_t *mb_setpoints,
        rc_filter_t *g_D1_filter, rc_filter_t *g_D2_filter, rc_filter_t *g_D3_filter);

#endif

