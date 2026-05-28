#ifndef UART_IMU_H
#define UART_IMU_H

#include <stdint.h>

int imu_init();
int parse_IMU_data();
void print_imu();

struct imu_data_d16qn
{
    uint16_t acc_x;
    uint16_t acc_y;
    uint16_t acc_z;
    uint16_t gyr_x;
    uint16_t gyr_y;
    uint16_t gyr_z;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t linacc_x;
    uint16_t linacc_y;
    uint16_t linacc_z;
};

void get_imu_snapshot_d16qn(struct imu_data_d16qn *out);

uint16_t get_acc_x_in_D16QN();
uint16_t get_acc_y_in_D16QN();
uint16_t get_acc_z_in_D16QN();

uint16_t get_gyr_x_in_D16QN();
uint16_t get_gyr_y_in_D16QN();
uint16_t get_gyr_z_in_D16QN();

uint16_t get_roll_in_D16QN();
uint16_t get_pitch_in_D16QN();
uint16_t get_yaw_in_D16QN();

uint16_t get_linacc_x_in_D16QN();
uint16_t get_linacc_y_in_D16QN();
uint16_t get_linacc_z_in_D16QN();

#endif
