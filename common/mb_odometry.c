/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

void mb_odometry_init(){
    mb_odometry.x = 0;
    mb_odometry.y = 0;
    mb_odometry.psi= 0;
}

void mb_odometry_update(){
    /* TODO */
    // change  the sign of left encoder
    int diff_left_encoder = -(mb_state.left_encoder-mb_state.last_left_encoder);
    int diff_right_encoder = (mb_state.right_encoder-mb_state.last_right_encoder);
    double diff_left_wheel_dist = WHEEL_DIAMETER * M_PI * diff_left_encoder/ENCODER_RES/GEAR_RATIO;
    double diff_right_wheel_dist = WHEEL_DIAMETER * M_PI * diff_right_encoder/ENCODER_RES/GEAR_RATIO;
    double diff_psi = (diff_right_wheel_dist - diff_left_wheel_dist)/WHEEL_BASE;
    double diff_d = (diff_right_wheel_dist + diff_left_wheel_dist)/2.0;
    double diff_x = diff_d*cos(mb_odometry.psi+diff_psi/2.0);
    double diff_y = diff_d*sin(mb_odometry.psi+diff_psi/2.0);
    mb_odometry.x = mb_odometry.x + diff_x;
    mb_odometry.y = mb_odometry.y + diff_y;
    mb_odometry.psi = mb_clamp_radians(mb_odometry.psi + diff_psi);

    mb_state.last_left_encoder = mb_state.left_encoder;
    mb_state.last_right_encoder = mb_state.right_encoder;
}

float mb_clamp_radians(float angle){
    if(angle >= M_PI){
        angle = angle - 2*M_PI;
    }
    if(angle <= -M_PI){
        angle = angle + 2*M_PI;
    }
    return angle;
}
