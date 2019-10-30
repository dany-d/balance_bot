/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
/* TODO */
    mb_odometry.x = x;
    mb_odometry.y = y;
    mb_odometry.theta = theta;
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
/* TODO */
    int diff_left_encoder = -(mb_state.left_encoder-mb_state.last_left_encoder);
    int diff_right_encoder = (mb_state.right_encoder-mb_state.last_right_encoder);
    double diff_left_wheel_dist = WHEEL_DIAMETER * M_PI * diff_left_encoder/ENCODER_RES/GEAR_RATIO;
    double diff_right_wheel_dist = WHEEL_DIAMETER * M_PI * diff_right_encoder/ENCODER_RES/GEAR_RATIO;
    double diff_theta = (diff_right_wheel_dist - diff_left_encoder)/WHEEL_BASE;
    double diff_d = (diff_right_wheel_dist + diff_left_encoder)/2.0;
    double diff_x = diff_d*cos(mb_odometry.theta+diff_theta/2.0);
    double diff_y = diff_d*sin(mb_odometry.theta+diff_theta/2.0);
    mb_odometry.x = mb_odometry.x + diff_x;
    mb_odometry.y = mb_odometry.y + diff_y;
    mb_odometry.theta = mb_odometry.theta + diff_theta;
}


float mb_clamp_radians(float angle){
    if(angle >= M_PI * 2){
        angle = angle - M_PI * 2;
    }
    if(angle <= -M_PI * 2){
        angle = angle + M_PI * 2;
    }
    return angle;
}