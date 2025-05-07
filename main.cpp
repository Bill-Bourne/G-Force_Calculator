/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"

// Define MPU6050 I2C address
#define MPU6050_ADDR 0x68 << 1 // 7-bit address shifted left for 8-bit format

// MPU6050 register addresses
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

I2C i2c(PB_7, PB_6);    // SDA, SCL
Serial pc(USBTX, USBRX); // USB serial interface for debugging

void mpu6050_write(uint8_t reg, uint8_t data) {
    char cmd[2] = {reg, data};
    i2c.write(MPU6050_ADDR, cmd, 2);
}

void mpu6050_read(uint8_t reg, char *buffer, int length) {
    i2c.write(MPU6050_ADDR, (char *)&reg, 1, true); // Write register address
    i2c.read(MPU6050_ADDR, buffer, length);        // Read data
}

void mpu6050_init() {
    mpu6050_write(MPU6050_PWR_MGMT_1, 0x00); // Wake up MPU6050
    pc.printf("MPU6050 initialized.\n");
}

void read_accel_gyro() {
    char data[14];
    mpu6050_read(MPU6050_ACCEL_XOUT_H, data, 14);

    // Extract accelerometer and gyroscope data
    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];
    int16_t gx = (data[8] << 8) | data[9];
    int16_t gy = (data[10] << 8) | data[11];
    int16_t gz = (data[12] << 8) | data[13];

    pc.printf("Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d\n", ax, ay, az, gx, gy, gz);
}

int main() {
    pc.baud(115200); // Set serial baud rate
    pc.printf("Starting MPU6050 Test...\n");

    i2c.frequency(400000); // Set I2C frequency to 400kHz
    mpu6050_init();

    while (true) {
        read_accel_gyro();
        ThisThread::sleep_for(500ms);
    }
}
