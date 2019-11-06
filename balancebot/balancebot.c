/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
*
*******************************************************************************/

#include <math.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/math/filter.h>
#include <rc/time.h>

#include "../common/mb_defs.h"
#include "../common/mb_motor.h"

#include "balancebot.h"

// Global Variables
rc_filter_t g_D1_filter = RC_FILTER_INITIALIZER;
rc_filter_t g_D2_filter = RC_FILTER_INITIALIZER;
rc_filter_t g_D3_filter = RC_FILTER_INITIALIZER;
double sse = 0.0;
int race = 1;
// utility function
static int sleep_dump(int seconds) {
    for (int s=seconds; s>0; s--) {
        printf("sleep for [%d] seconds.\n", s);
        rc_nanosleep(1E9);
    }
    return 0;
}
/*******************************************************************************
* int main()
*
*******************************************************************************/
int main(){
    // rc_dsm_bind_routine(); //bind DSM
    // return 0;

    // rc_dsm_calibrate_routine();
    // return 0;

	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

	printf("initializing xbee... \n");
	//initalize XBee Radio
	int baudrate = BAUDRATE;
	if(XBEE_init(baudrate)==-1){
		fprintf(stderr,"Error initializing XBee\n");
		return -1;
	};

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);


	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);


	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;
    mpu_config.dmp_fetch_accel_gyro = 1;
	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	printf("initializing controller...\n");
	if (mb_controller_init(&g_D1_filter, &g_D2_filter, &g_D3_filter, &sse) < 0) {
        fprintf(stderr,"controller initialization failed.\n");
        return -1;
    }

    printf("wait for imu stable for 5 sec\n");
    sleep_dump(5);

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	printf("initializing motors...\n");
	mb_motor_init();

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);
	mb_state.last_left_encoder = 0;
	mb_state.last_right_encoder = 0;

	printf("initializing odometry...\n");
	mb_odometry_init();

	//attach controller function to IMU interrupt
	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}

	// exit cleanly
    printf("Exit Gracefully\n");
    mb_controller_cleanup(&g_D1_filter, &g_D2_filter, &g_D3_filter);
	rc_mpu_power_off();
	mb_motor_cleanup();
	rc_led_cleanup();
    rc_dsm_cleanup();
	rc_encoder_eqep_cleanup();
    rc_nanosleep(1E9);
	rc_remove_pid_file(); // remove pid file LAST
	return 0;
}

/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
*
*
*******************************************************************************/
void balancebot_controller(){

	//lock state mutex
	pthread_mutex_lock(&state_mutex);

	// Read IMU
	mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X];
    mb_state.yaw = mpu_data.dmp_TaitBryan[TB_YAW_Z];

	// Read encoders
	mb_state.left_encoder = rc_encoder_eqep_read(1);
	mb_state.right_encoder = rc_encoder_eqep_read(2);

    // get distance travelled output of block G2
    double left_phi = -2 *3.14 * mb_state.left_encoder/ENCODER_RES/GEAR_RATIO;
    double right_phi = 2 *3.14 * mb_state.right_encoder/ENCODER_RES/GEAR_RATIO;
    mb_state.phi = (left_phi+right_phi)/2;
    double avg_encoder = (-mb_state.left_encoder+mb_state.right_encoder)/2.0;

	mb_state.dist_travelled = WHEEL_DIAMETER * M_PI * avg_encoder/ENCODER_RES/GEAR_RATIO;

    // Update odometry
    mb_odometry_update();

    // for heading angle
    double odm_angle_diff = mb_odometry.psi - mb_state.last_heading_angle;
    mb_state.last_heading_angle = mb_odometry.psi;
    mb_state.angle_travelled = mb_state.angle_travelled + odm_angle_diff;

    // Calculate controller outputs
    mb_controller_update(&mb_state, &mb_setpoints,
            &g_D1_filter, &g_D2_filter, &g_D3_filter, sse);

    if(rc_get_state()!=EXITING){
        mb_motor_set(LEFT_MOTOR, mb_state.left_pwm);
        mb_motor_set(RIGHT_MOTOR, mb_state.right_pwm);
    }

	XBEE_getData();
	double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	double tb_array[3] = {0, 0, 0};
	rc_quaternion_to_tb_array(q_array, tb_array);
	mb_state.opti_x = xbeeMsg.x;
	mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_roll = tb_array[0];
	mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up

   	//unlock state mutex
    pthread_mutex_unlock(&state_mutex);
    if(rc_get_state()==EXITING){
        mb_motor_set_all(0);
        return;
    }

}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
static float fwd_speed2dist(float speed) {
    // input is m/s
    float rad_s = speed/WHEEL_DIAMETER/M_PI; // m/s to rad/s
    return rad_s/RC_CTL_HZ;
}
static float trun_speed2angle(float speed) {
    // input is rad/s
    return speed/RC_CTL_HZ;
}
void* setpoint_control_loop(void* ptr){
    float forward_speed = 0;
    float turning_speed = 0;
    float manual_ctl = 0;
    float margin = 0.1; // if dsm value is smaller than the margin, setting to zero

    // autonomous mode variable
    int race_task = 2; // 1 drag race 2 square
    mb_setpoints.turn = 1;
    // int wp_i = 1;
    int finish = 0;
    //double wp_arry[] = {0.0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5, 10, 10.5, 11.0};
    // double wp_arry2[] = {0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 1};
    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return NULL;
	}

	while(1){
        if (rc_get_state() == EXITING) {
            break;
        }
		if(rc_dsm_is_new_data()){
			// TODO: Handle the DSM data from the Spektrum radio reciever
			// You may should implement switching between manual and autonomous mode
			// using channel 5 of the DSM data.
            forward_speed =  rc_dsm_ch_normalized(FOWARD_CHANNEL); // percentage -1.0 ~ 1.0
            turning_speed =  rc_dsm_ch_normalized(TURNING_CHANNEL);
            manual_ctl = rc_dsm_ch_normalized(MANUAL_CHANNEL);

            // margin
            if (fabs(forward_speed)<margin) forward_speed = 0;
            if (fabs(turning_speed)<margin) turning_speed = 0;

            // from normalized value to speed
            forward_speed = MAX_FWD_SPEED*forward_speed;      // m/s
            turning_speed = MAX_TURN_SPEED*turning_speed;   // rad/s

            // set manual flag
            if(manual_ctl > 0.5) {
                mb_setpoints.manual_ctl = 0;
            } else if (manual_ctl < -0.5){
                mb_setpoints.manual_ctl = 2;
            } else {
                mb_setpoints.manual_ctl = 1;
            }

            if (mb_setpoints.manual_ctl == 2) {
                // manual control mode
                // set value to global data structure
                mb_setpoints.fwd_velocity = forward_speed;
                mb_setpoints.turn_velocity = turning_speed;
                mb_setpoints.theta_ref = forward_speed * 5 / 180 * M_PI;
                mb_setpoints.wheel_angle += fwd_speed2dist(3*forward_speed);
                mb_setpoints.heading_angle = trun_speed2angle(turning_speed)*5;
            } else if (mb_setpoints.manual_ctl==1){
                // reset encoder
                mb_setpoints.wheel_angle = 0.0;
                mb_setpoints.heading_angle = mb_state.yaw;
                // wp_i = 1;
                finish = 0;
                if (mb_controller_init(&g_D1_filter, &g_D2_filter, &g_D3_filter, &sse) < 0) {
                    fprintf(stderr,"controller initialization failed.\n");
                    return NULL;
                }
                rc_encoder_eqep_write(1, 0);
                rc_encoder_eqep_write(2, 0);
                mb_state.last_left_encoder = 0;
                mb_state.last_right_encoder = 0;
                if (race_task==2){
                    mb_setpoints.heading_angle = mb_state.angle_travelled;
                }
                // fprintf(stderr,"=======================================\n");
            } else {
                // autonomuos mode
                //mb_setpoints.wheel_angle = 0.5/WHEEL_DIAMETER/M_PI;
                //mb_setpoints.heading_angle = 1.57;
                //mb_setpoints.wheel_angle = 0.5/WHEEL_DIAMETER/M_PI;

                ////// 11 meter racing
                if (race_task == 1 && finish == 0){
                    if (mb_state.phi > 11.5 * 2/ WHEEL_DIAMETER){ // final
                        mb_setpoints.wheel_angle += fwd_speed2dist(0.1);
                        if (mb_state.phi > 11.89 * 2/ WHEEL_DIAMETER){
                            mb_setpoints.wheel_angle =11.9 * 2/ WHEEL_DIAMETER;
                            finish = 1;
                        }
                    } else if (mb_state.phi > 1 * 2/ WHEEL_DIAMETER
                        && mb_state.phi <= 2 * 2/ WHEEL_DIAMETER){ // mid start
                        mb_setpoints.wheel_angle += fwd_speed2dist(6);
                    } else if  (mb_state.phi > 2 * 2/ WHEEL_DIAMETER
                        && mb_state.phi <= 5 * 2/ WHEEL_DIAMETER){
                        mb_setpoints.wheel_angle += fwd_speed2dist(7);
                    } else if  (mb_state.phi > 5 * 2/ WHEEL_DIAMETER
                        && mb_state.phi <= 10 * 2/ WHEEL_DIAMETER){
                        mb_setpoints.wheel_angle += fwd_speed2dist(7.3);
                    } else if  (mb_state.phi > 10 * 2/ WHEEL_DIAMETER
                        && mb_state.phi <= 11.2 * 2/ WHEEL_DIAMETER){ // mid end
                        mb_setpoints.wheel_angle += fwd_speed2dist(6);
                    } else if  (mb_state.phi > 11.2 * 2/ WHEEL_DIAMETER
                        && mb_state.phi <= 11.5 * 2/ WHEEL_DIAMETER){ // mid end
                        mb_setpoints.wheel_angle += fwd_speed2dist(0.2);

                    } else { // start
                        mb_setpoints.wheel_angle += fwd_speed2dist(4.5);
                    }
                }
                /*
                if (race_task == 1 && finish == 0){
                    if (mb_state.phi > 11.1 * 2/ WHEEL_DIAMETER){ // final
                        mb_setpoints.wheel_angle += fwd_speed2dist(0.1);
                        if (mb_state.phi > 11.59 * 2/ WHEEL_DIAMETER){
                            mb_setpoints.wheel_angle =11.6 * 2/ WHEEL_DIAMETER;
                            finish = 1;
                        }
                    } else if (mb_state.phi > 1 * 2/ WHEEL_DIAMETER
                        && mb_state.phi <= 2 * 2/ WHEEL_DIAMETER){ // mid start
                        mb_setpoints.wheel_angle += fwd_speed2dist(5);
                    } else if (mb_state.phi > 2 * 2/ WHEEL_DIAMETER
                        && mb_state.phi <= 4 * 2/ WHEEL_DIAMETER){ // mid start
                        mb_setpoints.wheel_angle += fwd_speed2dist(6);
                    } else if  (mb_state.phi > 4 * 2/ WHEEL_DIAMETER
                        && mb_state.phi <= 10 * 2/ WHEEL_DIAMETER){
                        mb_setpoints.wheel_angle += fwd_speed2dist(7);
                    } else if  (mb_state.phi > 10 * 2/ WHEEL_DIAMETER
                        && mb_state.phi <= 11 * 2/ WHEEL_DIAMETER){ // mid end
                        mb_setpoints.wheel_angle += fwd_speed2dist(6);
                    } else if  (mb_state.phi > 11 * 2/ WHEEL_DIAMETER
                        && mb_state.phi <= 11.2 * 2/ WHEEL_DIAMETER){ // mid end
                        mb_setpoints.wheel_angle += fwd_speed2dist(0.2);
                    } else { // start
                        mb_setpoints.wheel_angle += fwd_speed2dist(4);
                    }
                }*/



                if (race_task == 2){
                    if (mb_state.phi > 0.7 * 2/ WHEEL_DIAMETER){ // final
                        mb_setpoints.wheel_angle += fwd_speed2dist(0.5);
                        if (mb_state.phi > (1.09-WHEEL_BASE) * 2/ WHEEL_DIAMETER){
                            mb_setpoints.wheel_angle = (1.1-WHEEL_BASE) * 2/ WHEEL_DIAMETER;
                            mb_setpoints.turn = 1;
                            mb_setpoints.heading_angle += M_PI/2;
                            // rc_nanosleep(1E9);
                            mb_setpoints.turn = 0;
                            rc_encoder_eqep_write(1, 0);
                            rc_encoder_eqep_write(2, 0);
                            mb_state.last_right_encoder = 0;
                            mb_state.last_left_encoder = 0; 
                            mb_setpoints.wheel_angle = 0;
                        }
                    } else if (mb_state.phi > 0.2 * 2/ WHEEL_DIAMETER
                             && mb_state.phi <= 0.6 * 2/ WHEEL_DIAMETER){ // mid start
                            mb_setpoints.wheel_angle += fwd_speed2dist(0.5);
                    } else {
                        mb_setpoints.wheel_angle += fwd_speed2dist(0.3);
                    }
                }
            }
	 	    rc_nanosleep(1E9/RC_CTL_HZ);
	    }
    }
	return NULL;
}

/*******************************************************************************
* printf_loop()
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state

    // dump odometry data into file
    int dump_data = 0;
    char odm_data_path[] = "odometry.csv";
    FILE *fp = NULL;

    if (dump_data) {
        fp = fopen(odm_data_path, "w");
        if (fp == NULL) {
            fprintf(stderr, "cannot open file [%s].\n", odm_data_path);
            return NULL;
        }
    }

	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |            MOCAP            |");
			printf("\n");
			printf(" odo yaw |");
            printf("  theta  |");
			printf("    φ    |");
			printf(" heading |");
			// printf("  L Enc  |");
			// printf("  R Enc  |");
			// printf("    X    |");
			// printf("    Y    |");
			printf("    ψ    |");
            printf("   dist  |");
            printf("  manual |");
			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;

		if(new_state == RUNNING) {
			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_state.angle_travelled);
            printf("%7.3f  |", mb_state.theta);
			printf("%7.3f  |", mb_state.dist_travelled);
			printf("%7.3f  |", mb_setpoints.heading_angle);
			// printf("%7d  |", mb_state.left_encoder);
			// printf("%7d  |", mb_state.right_encoder);
			// printf("%7.3f  |", mb_state.opti_x);
			// printf("%7.3f  |", mb_state.opti_y);
			//printf("%7.3f  |", mb_state.opti_yaw);
            //printf("%7.3f  |", mb_state.dist_travelled);
            printf("%7.3f  |", mb_setpoints.heading_angle);
			printf("%7.3f  |", mb_setpoints.wheel_angle*WHEEL_DIAMETER/2);
            printf("  %d  |", mb_setpoints.manual_ctl);
			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
        if (fp){fprintf(fp, "%.4f, %.4f, %.4f, %.4f\n", mb_odometry.x, mb_odometry.y, mb_odometry.psi, mb_state.yaw);}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
    if(fp){fclose(fp);}
	return NULL;
}
