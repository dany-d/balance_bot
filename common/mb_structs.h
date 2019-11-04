#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   phi;               // average wheel angle (rad)
    float   yaw;               // heading angle (rad)
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading

    //outputs
    float   left_pwm;  //left wheel command [-1..1]
    float   right_pwm; //right wheel command [-1..1]

    float opti_x;
    float opti_y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;

    //TODO: Add more variables to this state as needed
    int  last_left_encoder;
    int  last_right_encoder;
    float last_yaw; // only gryo value
    float dist_travelled;
    float last_heading_angle; // last fused odm angle
    float angle_travelled; // accumulate angle
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    float wheel_angle; // mapping to phi
    float heading_angle; // mapping to yaw
    int manual_ctl;
    float theta_ref;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{
    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float psi;      //orientation from initialization in rad
    float psi_no_clamp;
};

#endif
