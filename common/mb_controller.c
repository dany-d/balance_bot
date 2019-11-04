#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"

static float saturate(float input) {
    if (input < -1.0) {
        return -1.0;
    }
    else if (input > 1.0) {
        return 1.0;
    }else{
        return input;
    }
}

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
int mb_controller_init(rc_filter_t *g_D1_filter, rc_filter_t *g_D2_filter, rc_filter_t *g_D3_filter, double *sse) {
    char param_name[100] = {0};
    FILE *fp;

    double D1_KP = 0;
    double D1_KI = 0;
    double D1_KD = 0;
    double D2_KP = 0;
    double D2_KI = 0;
    double D2_KD = 0;
    double D3_KP = 0;
    double D3_KI = 0;
    double D3_KD = 0;

    fp = fopen(CFG_PATH, "r");
    if (fp == NULL) {
        fprintf(stderr, "controller config file [%s] doesn't exist.\n", CFG_PATH);
        return -1;
    }
    fscanf(fp, "%s %lf\n", param_name, &D1_KP);
    //fprintf(stdout, "%s %lf\n", param_name, D1_KP);
    fscanf(fp, "%s %lf\n", param_name, &D1_KI);
    //fprintf(stdout, "%s %lf\n", param_name, D1_KI);
    fscanf(fp, "%s %lf\n", param_name, &D1_KD);
    //fprintf(stdout, "%s %lf\n", param_name, D1_KD);

    fscanf(fp, "%s %lf\n", param_name, &D2_KP);
    // fprintf(stdout, "%s %lf\n", param_name, D2_KP);
    fscanf(fp, "%s %lf\n", param_name, &D2_KI);
    // fprintf(stdout, "%s %lf\n", param_name, D2_KI);
    fscanf(fp, "%s %lf\n", param_name, &D2_KD);
    // fprintf(stdout, "%s %lf\n", param_name, D2_KD);

    fscanf(fp, "%s %lf\n", param_name, &D3_KP);
    // fprintf(stdout, "%s %lf\n", param_name, D3_KP);
    fscanf(fp, "%s %lf\n", param_name, &D3_KI);
    // fprintf(stdout, "%s %lf\n", param_name, D3_KI);
    fscanf(fp, "%s %lf\n", param_name, &D3_KD);
    // fprintf(stdout, "%s %lf\n", param_name, D3_KD);

    fscanf(fp, "%s %lf\n", param_name, sse);
    // fprintf(stdout, "%s %lf\n", param_name, *sse);

    fclose(fp);

    if(rc_filter_pid(g_D1_filter, D1_KP, D1_KI, D1_KD, 2.7*DT, DT)){
            fprintf(stderr,"ERROR in rc_filter_pid.\n");
            return -1;
    }
    rc_filter_enable_saturation(g_D1_filter, -1.0, 1.0);
    rc_filter_enable_soft_start(g_D1_filter, 0.5);

    if(rc_filter_pid(g_D2_filter, D2_KP, D2_KI, D2_KD, 19.4*DT, DT)){
            fprintf(stderr,"ERROR in rc_filter_pid.\n");
            return -1;
    }
    rc_filter_enable_saturation(g_D2_filter, -0.1, 0.1);
    rc_filter_enable_soft_start(g_D2_filter, 0.5);

    if(rc_filter_pid(g_D3_filter, D3_KP, D3_KI, D3_KD, 4*DT, DT)){
        fprintf(stderr,"ERROR in rc_filter_pid.\n");
        return -1;
    }
    rc_filter_enable_saturation(g_D3_filter, -0.5, 0.5);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
*
*
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/
int mb_controller_update(mb_state_t *mb_state, mb_setpoints_t *mb_setpoints,
        rc_filter_t *g_D1_filter, rc_filter_t *g_D2_filter, rc_filter_t *g_D3_filter, double sse) {

    double theta_ref = 0;
    double pwm_duty = 0;
    double turning_pwm_duty = 0;

    if (mb_setpoints->manual_ctl == 2){
        theta_ref = mb_setpoints->theta_ref;
        // theta_ref = rc_filter_march(g_D2_filter, (mb_setpoints->wheel_angle-mb_state->phi));
        pwm_duty = rc_filter_march(g_D1_filter, (theta_ref+sse-mb_state->theta));
        turning_pwm_duty = mb_setpoints->heading_angle;
        //turning_pwm_duty = rc_filter_march(g_D3_filter, mb_setpoints->heading_angle-mb_state->yaw);
    }
    if (mb_setpoints->manual_ctl == 0){
        theta_ref = rc_filter_march(g_D2_filter, (mb_setpoints->wheel_angle-mb_state->phi));
        pwm_duty = rc_filter_march(g_D1_filter, (theta_ref+sse-mb_state->theta));
        turning_pwm_duty = rc_filter_march(g_D3_filter, mb_setpoints->heading_angle-mb_state->yaw);
    }
    mb_state->left_pwm = saturate(-turning_pwm_duty+pwm_duty);
    mb_state->right_pwm = saturate(turning_pwm_duty+pwm_duty);

    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
*
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/
int mb_controller_cleanup(rc_filter_t *g_D1_filter, rc_filter_t *g_D2_filter, rc_filter_t *g_D3_filter) {
    rc_filter_free(g_D1_filter);
    rc_filter_free(g_D2_filter);
    rc_filter_free(g_D3_filter);
    return 0;
}
