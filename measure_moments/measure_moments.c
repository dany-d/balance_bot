/*******************************************************************************
* measure_moments.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the moments of inertia of your Balancebot
*
* TODO: capture the gyro data and timestamps to a file to determine the period.
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/mpu.h>


FILE* f1;

/*******************************************************************************
* int main()
*
*******************************************************************************/
int main(){

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
    // if(rc_dsm_init()==-1){
    //     fprintf(stderr,"failed to start initialize DSM\n");
    //     return -1;
    // }


    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();

    rc_set_state(RUNNING);

    // set up IMU configuration
    printf("initializing imu... \n");
    // set up mpu configuration
    rc_mpu_config_t conf = rc_mpu_default_config();
    rc_mpu_data_t data;
    //mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
    //mpu_config.orient = ORIENTATION_Z_UP;

    // now set up the imu for dmp interrupt operation
    if(rc_mpu_initialize_dmp(&data, conf)){
        printf("rc_mpu_initialize_failed\n");
        return -1;
    }

    //rc_nanosleep(5E9); // wait for imu to stabilize

    uint64_t tstart=rc_nanos_since_boot();
    uint64_t tcurr = 0;
    //std::vector<std::vector<float>> angles_data;
    //float oneline_data[4];
    while(tcurr < 10E9 /**/){
        rc_nanosleep(1E6);
        float pitch= data.dmp_TaitBryan[TB_PITCH_X];
        float roll = data.dmp_TaitBryan[TB_ROLL_Y];
        float yaw = data.dmp_TaitBryan[TB_YAW_Z];
        float time_elapsed = (float)(rc_nanos_since_boot()-tstart)/1E9;
        /*
        oneline_data.push_back(pitch);
        oneline_data.push_back(roll);
        oneline_data.push_back(yaw);
        oneline_data.push_back(timeelapsed);t
        angles_data.push_back(oneline_data);
        */

        tcurr = rc_nanos_since_boot()-tstart;
        fprintf(stdout, "%7.3f, %7.3f, %7.3f, %7.3f\n", time_elapsed, roll, pitch, yaw);
        fprintf(stderr, "%7.3f, %7.3f, %7.3f, %7.3f\n", time_elapsed, roll, pitch, yaw);

     //    if(!rc_mpu_read_accel(&data)) {
     //        fprintf(stderr, "ERROR: failed to read accel\n");
     //    }
     //    fprintf(stdout,"accl X:%lf\n", data.accel[0]);
     //    fprintf(stdout,"accl Y:%lf\n", data.accel[1]);
     //    fprintf(stdout,"accl Z:%lf\n", data.accel[2]);

     //    if(!rc_mpu_read_gyro(&data)) {
     //        fprintf(stderr, "ERROR: failed to read gyro\n");
     //    }
     //    fprintf(stdout,"gyro X:%lf\n", data.gyro[0]);
     //    fprintf(stdout,"gyro Y:%lf\n", data.gyro[1]);
     //    fprintf(stdout,"gyro Z:%lf\n", data.gyro[2]);
    	// rc_nanosleep(1E9);
    }

	// exit cleanly
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}
