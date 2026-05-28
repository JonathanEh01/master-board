#include "uart_imu.h"
#include "imu_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include <math.h>
#include <stdio.h>

#define FLOAT_TO_D16QN(a, n) ((int16_t)((a) * (1 << (n))))

#define IMU_QN_ACC 11
#define IMU_QN_GYR 11
#define IMU_QN_EF 13

struct imu_data_float
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
    float roll;
    float pitch;
    float yaw;
    float linacc_x;
    float linacc_y;
    float linacc_z;
};

static struct imu_data_float imu;
static portMUX_TYPE imu_lock = portMUX_INITIALIZER_UNLOCKED;

static inline uint16_t float_to_d16qn(float value, int q)
{
    return (uint16_t)FLOAT_TO_D16QN(value, q);
}

static inline float clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

void imu_common_reset(void)
{
    portENTER_CRITICAL(&imu_lock);
    imu = (struct imu_data_float){0};
    portEXIT_CRITICAL(&imu_lock);
}

void imu_common_set_acc_g(float x, float y, float z)
{
    portENTER_CRITICAL(&imu_lock);
    imu.acc_x = x;
    imu.acc_y = y;
    imu.acc_z = z;
    portEXIT_CRITICAL(&imu_lock);
}

void imu_common_set_gyro_rads(float x, float y, float z)
{
    portENTER_CRITICAL(&imu_lock);
    imu.gyr_x = x;
    imu.gyr_y = y;
    imu.gyr_z = z;
    portEXIT_CRITICAL(&imu_lock);
}

void imu_common_set_attitude_rad(float roll, float pitch, float yaw)
{
    portENTER_CRITICAL(&imu_lock);
    imu.roll = roll;
    imu.pitch = pitch;
    imu.yaw = yaw;
    portEXIT_CRITICAL(&imu_lock);
}

void imu_common_set_linear_acc_g(float x, float y, float z)
{
    portENTER_CRITICAL(&imu_lock);
    imu.linacc_x = x;
    imu.linacc_y = y;
    imu.linacc_z = z;
    portEXIT_CRITICAL(&imu_lock);
}

void imu_common_set_quaternion(float w, float x, float y, float z)
{
    const float sinr_cosp = 2.0f * (w * x + y * z);
    const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    const float roll = atan2f(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (w * y - z * x);
    const float pitch = asinf(clampf(sinp, -1.0f, 1.0f));

    const float siny_cosp = 2.0f * (w * z + x * y);
    const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    const float yaw = atan2f(siny_cosp, cosy_cosp);

    imu_common_set_attitude_rad(roll, pitch, yaw);
}

uint16_t get_acc_x_in_D16QN() { return float_to_d16qn(imu.acc_x, IMU_QN_ACC); }
uint16_t get_acc_y_in_D16QN() { return float_to_d16qn(imu.acc_y, IMU_QN_ACC); }
uint16_t get_acc_z_in_D16QN() { return float_to_d16qn(imu.acc_z, IMU_QN_ACC); }

uint16_t get_gyr_x_in_D16QN() { return float_to_d16qn(imu.gyr_x, IMU_QN_GYR); }
uint16_t get_gyr_y_in_D16QN() { return float_to_d16qn(imu.gyr_y, IMU_QN_GYR); }
uint16_t get_gyr_z_in_D16QN() { return float_to_d16qn(imu.gyr_z, IMU_QN_GYR); }

uint16_t get_roll_in_D16QN() { return float_to_d16qn(imu.roll, IMU_QN_EF); }
uint16_t get_pitch_in_D16QN() { return float_to_d16qn(imu.pitch, IMU_QN_EF); }
uint16_t get_yaw_in_D16QN() { return float_to_d16qn(imu.yaw, IMU_QN_EF); }

uint16_t get_linacc_x_in_D16QN() { return float_to_d16qn(imu.linacc_x, IMU_QN_ACC); }
uint16_t get_linacc_y_in_D16QN() { return float_to_d16qn(imu.linacc_y, IMU_QN_ACC); }
uint16_t get_linacc_z_in_D16QN() { return float_to_d16qn(imu.linacc_z, IMU_QN_ACC); }

void get_imu_snapshot_d16qn(struct imu_data_d16qn *out)
{
    portENTER_CRITICAL(&imu_lock);
    struct imu_data_float snapshot = imu;
    portEXIT_CRITICAL(&imu_lock);

    out->acc_x = float_to_d16qn(snapshot.acc_x, IMU_QN_ACC);
    out->acc_y = float_to_d16qn(snapshot.acc_y, IMU_QN_ACC);
    out->acc_z = float_to_d16qn(snapshot.acc_z, IMU_QN_ACC);
    out->gyr_x = float_to_d16qn(snapshot.gyr_x, IMU_QN_GYR);
    out->gyr_y = float_to_d16qn(snapshot.gyr_y, IMU_QN_GYR);
    out->gyr_z = float_to_d16qn(snapshot.gyr_z, IMU_QN_GYR);
    out->roll = float_to_d16qn(snapshot.roll, IMU_QN_EF);
    out->pitch = float_to_d16qn(snapshot.pitch, IMU_QN_EF);
    out->yaw = float_to_d16qn(snapshot.yaw, IMU_QN_EF);
    out->linacc_x = float_to_d16qn(snapshot.linacc_x, IMU_QN_ACC);
    out->linacc_y = float_to_d16qn(snapshot.linacc_y, IMU_QN_ACC);
    out->linacc_z = float_to_d16qn(snapshot.linacc_z, IMU_QN_ACC);
}

void print_imu()
{
    struct imu_data_float snapshot;

    portENTER_CRITICAL(&imu_lock);
    snapshot = imu;
    portEXIT_CRITICAL(&imu_lock);

    printf("\n%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t",
           snapshot.acc_x,
           snapshot.acc_y,
           snapshot.acc_z,
           snapshot.gyr_x,
           snapshot.gyr_y,
           snapshot.gyr_z,
           snapshot.roll,
           snapshot.pitch,
           snapshot.yaw,
           snapshot.linacc_x,
           snapshot.linacc_y,
           snapshot.linacc_z);
}
