#ifndef MBOT_H
#define MBOT_H

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <hardware/gpio.h>
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>
#include <mbot/motor/motor.h>
#include <mbot/defs/mbot_pins.h>
#include <mbot/defs/mbot_params.h>
#include <mbot/fram/fram.h>
#include <mbot/imu/imu.h>
#include <rc/math/filter.h>
#include <rc/mpu/mpu.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/mbot_channels.h>
#include <mbot_lcm_msgs_serial.h>
#include <mbot/servo/servo.h>

#include <math.h>
#include <inttypes.h>

#define MESSAGE_CONFIRMATION_CHANNEL "MSG_CONFIRM"

// TODO: Decide which controller is used, open loop = 1, PID = 0
#define OPEN_LOOP 1

//Define drive type of this robot. See mbot_params.h.
// #define MBOT_DRIVE_TYPE OMNI_120_DRIVE
#define MBOT_DRIVE_TYPE DIFFERENTIAL_DRIVE

extern mbot_bhy_data_t mbot_imu_data;

// Global pointer to the i2c bus
static i2c_inst_t *i2c;

// data to hold calibration coefficients
float coeffs[12];  // 4 calibration parameters per motor 

enum drive_modes{
    MODE_MOTOR_PWM = 0,
    MODE_MOTOR_VEL_OL = 1,
    MODE_MOTOR_VEL_PID = 2,
    MODE_MBOT_VEL = 3
};

/*
* Messages used by the MBot code, 
* we also use these to store state
*/
// origin: mbot
serial_mbot_imu_t mbot_imu = {0};
serial_pose2D_t mbot_odometry = {0};
serial_mbot_encoders_t mbot_encoders = {0};
serial_twist2D_t mbot_vel = {0};
serial_mbot_motor_pwm_t mbot_motor_pwm = {0};
serial_mbot_motor_vel_t mbot_motor_vel = {0};

// origin: comms
serial_twist2D_t mbot_vel_cmd = {0};
serial_mbot_motor_pwm_t mbot_motor_pwm_cmd = {0};
serial_mbot_motor_vel_t mbot_motor_vel_cmd = {0};
serial_timestamp_t mbot_received_time = {0};
serial_forklift_mode_t forklift_mode_cmd = {0};


//callback functions
void timestamp_cb(serial_timestamp_t *msg);
void reset_encoders_cb(serial_mbot_encoders_t *msg);
void reset_odometry_cb(serial_pose2D_t *msg);
void mbot_vel_cmd_cb(serial_twist2D_t *msg);
void mbot_motor_vel_cmd_cb(serial_mbot_motor_vel_t *msg);
void mbot_motor_pwm_cmd_cb(serial_mbot_motor_pwm_t *msg);
void mbot_forklift_mode_cmd_cb(serial_forklift_mode_t *msg);
bool mbot_loop(repeating_timer_t *rt);
void mbot_read_encoders(serial_mbot_encoders_t* encoders);
void mbot_calculate_motor_vel(serial_mbot_encoders_t encoders, serial_mbot_motor_vel_t *motor_vel);

//helper functions
float _calibrated_pwm_from_vel_cmd(float vel_cmd, int motor_idx);

rc_filter_t pid_left;
rc_filter_t pid_right;
rc_filter_t pid_angular; 
rc_filter_t lowpass_linear;
rc_filter_t lowpass_angular;

// forklift
float sv2_channel_normalized_pulse_;
float sv3_channel_normalized_pulse_;

const uint8_t SV2_CHANNEL = 2;
const uint8_t SV3_CHANNEL = 3;
const float forklift_angle_up_ = 0.65;
const float forklift_angle_stack_ = 0.50;
const float forklift_angle_down_ = -0.35;
float forklift_angle_live_ = forklift_angle_up_; // set to up since we are initilziing as up
bool test_flag_ = true;


#endif
