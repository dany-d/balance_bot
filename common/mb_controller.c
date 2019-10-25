#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_init(double *D1_KP, double *D1_KI, double *D1_KD, double *D2_KP, double *D2_KI, double *D2_KD){
    char param_name[100] = {0};
    FILE *fp;
    fp = fopen(CFG_PATH, "r");
    if (fp == NULL) {
        fprintf(stderr, "controller config file [%s] doesn't exist.\n", CFG_PATH);
        return -1;
    }
    fscanf(fp, "%s %lf\n", param_name, D1_KP);
    fprintf(stdout, "%s %lf\n", param_name, *D1_KP);
    fscanf(fp, "%s %lf\n", param_name, D1_KI);
    fprintf(stdout, "%s %lf\n", param_name, *D1_KI);
    fscanf(fp, "%s %lf\n", param_name, D1_KD);
    fprintf(stdout, "%s %lf\n", param_name, *D1_KD);

    fscanf(fp, "%s %lf\n", param_name, D2_KP);
    fprintf(stdout, "%s %lf\n", param_name, *D2_KP);
    fscanf(fp, "%s %lf\n", param_name, D2_KI);
    fprintf(stdout, "%s %lf\n", param_name, *D2_KI);
    fscanf(fp, "%s %lf\n", param_name, D2_KD);
    fprintf(stdout, "%s %lf\n", param_name, *D2_KD);
    fclose(fp);
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

int mb_controller_update(mb_state_t* mb_state){
    /*TODO: Write your controller here*/
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

int mb_controller_cleanup(){
    return 0;
}