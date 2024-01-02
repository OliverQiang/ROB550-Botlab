#include "odometry.h"
#include <math.h>
#include <stdio.h>
#define M_PI 3.14159265358979323846

static float theta_prev = 0.0f; 
float degrees_to_radians(float degrees) {
    return degrees * (M_PI / 180.0f);
}
float DELTA_THETA_THRESHOLD = 0.1 * (M_PI / 180.0f);

int mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, serial_twist2D_t *mbot_vel){
    mbot_vel->vx =  DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f;
    mbot_vel->vy = 0;
    mbot_vel->wz =  DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS);
    return 0; // Return 0 to indicate success
}
int mbot_calculate_diff_body_vel_imu(float wheel_left_vel, float wheel_right_vel, serial_mbot_imu_t imu, serial_twist2D_t *mbot_vel){
        
    mbot_vel->vx = DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f;
    mbot_vel->vy = 0;
    mbot_vel->wz = DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS);

    // gyrodometry
    float delta_theta_gyro = imu.gyro[2]; 
    float delta_theta_odo = mbot_vel->wz; 
    float delta_g_o = delta_theta_gyro - delta_theta_odo;

    if(fabs(delta_g_o) > DELTA_THETA_THRESHOLD){
        theta_prev += delta_theta_gyro; 
    }
    else{
        theta_prev += delta_theta_odo; 
    }

    // mbot_vel->wz = theta_prev; 

    return 0; 
}
int mbot_calculate_omni_body_vel(float wheel_left_vel, float wheel_right_vel, float wheel_back_vel, serial_twist2D_t *mbot_vel){
    mbot_vel->vx =  OMNI_WHEEL_RADIUS * (wheel_left_vel * INV_SQRT3 - wheel_right_vel * INV_SQRT3);
    mbot_vel->vy =  OMNI_WHEEL_RADIUS * (-wheel_left_vel / 3.0 - wheel_right_vel / 3.0 + wheel_back_vel * (2.0 / 3.0));
    mbot_vel->wz =  OMNI_WHEEL_RADIUS * -(wheel_left_vel + wheel_right_vel + wheel_back_vel) / (3.0f * OMNI_BASE_RADIUS);
    return 0; // Return 0 to indicate success
}
int mbot_calculate_omni_body_vel_imu(float wheel_left_vel, float wheel_right_vel, float wheel_back_vel, serial_mbot_imu_t imu, serial_twist2D_t *mbot_vel){
    return 0; // Return 0 to indicate success
}

int mbot_calculate_odometry(serial_twist2D_t mbot_vel, float dt, serial_pose2D_t *odometry){
    float vx_space = mbot_vel.vx * cos(odometry->theta) - mbot_vel.vy * sin(odometry->theta);
    float vy_space = mbot_vel.vx * sin(odometry->theta) + mbot_vel.vy * cos(odometry->theta);

    odometry->x += vx_space * dt;
    odometry->y += vy_space * dt;
    odometry->theta += mbot_vel.wz * dt;

    // Normalize theta to be between -pi and pi
    while (odometry->theta > M_PI) odometry->theta -= 2 * M_PI;
    while (odometry->theta <= -M_PI) odometry->theta += 2 * M_PI;
    // printf("%f %f \n", odometry->x, odometry->y);


    return 0; // Return 0 to indicate success
}