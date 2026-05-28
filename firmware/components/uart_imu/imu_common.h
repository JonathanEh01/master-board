#ifndef IMU_COMMON_H
#define IMU_COMMON_H

void imu_common_reset(void);

void imu_common_set_acc_g(float x, float y, float z);
void imu_common_set_gyro_rads(float x, float y, float z);
void imu_common_set_attitude_rad(float roll, float pitch, float yaw);
void imu_common_set_linear_acc_g(float x, float y, float z);
void imu_common_set_quaternion(float w, float x, float y, float z);

#endif
