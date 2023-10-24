#include "MPU6050.hpp"

MPU6050::MPU6050() {
    this->addr = MPU_DEFAULT_I2C_ADDR;
    this->init();
}

MPU6050::MPU6050(uint8_t addr) {
    this->addr = addr;
    this->init();
}

void MPU6050::init() {
    this->write_to_register(REG_PWR_MGMT_1, 0b00000000); // PWR_MGMT_1
    this->write_to_register(REG_GYRO_CONFIG, 0b00010000); // GYRO_CONFIG
    this->write_to_register(REG_ACCEL_CONFIG, 0b00000000); // ACCEL_CONFIG
    this->write_to_register(REG_CONFIG, 0b00000110); // CONFIG

    this->config.gyro_offset.x = 0;
    this->config.gyro_offset.y = 0;
    this->config.gyro_offset.z = 0;
}

void MPU6050::calibrate(uint16_t samples) {
    float_vector3_t gyro_sum = {0, 0, 0};
    for (uint16_t i = 0; i < samples; i++) {
        this->read_raw_gyro();
        gyro_sum.x += this->raw_gyro[0];
        gyro_sum.y += this->raw_gyro[1];
        gyro_sum.z += this->raw_gyro[2];
        sleep_ms(5);
    }
    this->config.gyro_offset.x = gyro_sum.x / samples;
    this->config.gyro_offset.y = gyro_sum.y / samples;
    this->config.gyro_offset.z = gyro_sum.z / samples;
}

void MPU6050::set_gyro_scale(mpu_6050_scale scale) {
    switch (scale)
    {
    case mpu_6050_scale::MPU6050_SCALE_250DPS:
        this->config.dps_per_digit = 131.0f;
        break;
    case mpu_6050_scale::MPU6050_SCALE_500DPS:
        this->config.dps_per_digit = 65.5f;
        break;
    case mpu_6050_scale::MPU6050_SCALE_1000DPS:
        this->config.dps_per_digit = 32.8f;
        break;
    case mpu_6050_scale::MPU6050_SCALE_2000DPS:
        this->config.dps_per_digit = 16.4f;
        break;
    }
    this->config.scale = scale;
    this->write_to_register(REG_GYRO_CONFIG, scale << 3);
}

void MPU6050::set_accel_range(mpu_6050_range range) {
    switch (range)
    {
    case mpu_6050_range::MPU6050_RANGE_2G:
        this->config.range_per_digit = 16384.0f;
        break;
    case mpu_6050_range::MPU6050_RANGE_4G:
        this->config.range_per_digit = 8192.0f;
        break;
    case mpu_6050_range::MPU6050_RANGE_8G:
        this->config.range_per_digit = 4096.0f;
        break;
    case mpu_6050_range::MPU6050_RANGE_16G:
        this->config.range_per_digit = 2048.0f;
        break;
    }
    this->config.range = range;
    this->write_to_register(REG_ACCEL_CONFIG, range << 3);
}

void MPU6050::update_all_data() {
    uint8_t* data = this->read_from_register(REG_ACCEL_XOUT_H, 14);
    this->raw_acc[0] = (data[0] << 8) | data[1];
    this->raw_acc[1] = (data[2] << 8) | data[3];
    this->raw_acc[2] = (data[4] << 8) | data[5];
    this->raw_temp = (data[6] << 8) | data[7];
    this->raw_gyro[0] = (data[8] << 8) | data[9];
    this->raw_gyro[1] = (data[10] << 8) | data[11];
    this->raw_gyro[2] = (data[12] << 8) | data[13];
    delete[] data;

    this->acc.x = (float)(this->raw_acc[0] / this->config.range_per_digit);
    this->acc.y = (float)(this->raw_acc[1] / this->config.range_per_digit);
    this->acc.z = (float)(this->raw_acc[2] / this->config.range_per_digit);
    this->temp = ((int16_t)this->raw_temp) / 340 + 36.53f;
    this->gyro.x = (((float)this->raw_gyro[0])- this->config.gyro_offset.x) / this->config.dps_per_digit;
    this->gyro.y = (((float)this->raw_gyro[1]) - this->config.gyro_offset.y) / this->config.dps_per_digit;
    this->gyro.z = (((float)this->raw_gyro[2]) - - this->config.gyro_offset.z) / this->config.dps_per_digit;
}

void MPU6050::update_only_acc() {
    this->read_raw_acc();
    this->acc.x = (float)(this->raw_acc[0] / this->config.range_per_digit);
    this->acc.y = (float)(this->raw_acc[1] / this->config.range_per_digit);
    this->acc.z = (float)(this->raw_acc[2] / this->config.range_per_digit);
}

void MPU6050::update_only_temp() {
    this->read_raw_temp();
    this->temp = ((int16_t)this->raw_temp) / 340 + 36.53f;
}

void MPU6050::update_only_gyro() {
    this->read_raw_gyro();
    this->gyro.x = (((float)this->raw_gyro[0])- this->config.gyro_offset.x) / this->config.dps_per_digit;
    this->gyro.y = (((float)this->raw_gyro[1]) - this->config.gyro_offset.y) / this->config.dps_per_digit;
    this->gyro.z = (((float)this->raw_gyro[2]) - - this->config.gyro_offset.z) / this->config.dps_per_digit;    
}

void MPU6050::read_raw_acc() {
    uint8_t* data = this->read_from_register(REG_ACCEL_XOUT_H, 6);
    this->raw_acc[0] = (data[0] << 8) | data[1];
    this->raw_acc[1] = (data[2] << 8) | data[3];
    this->raw_acc[2] = (data[4] << 8) | data[5];
}
void MPU6050::read_raw_temp() {
    uint8_t* data = this->read_from_register(REG_TEMP_OUT_H, 2);
    this->raw_temp = (data[0] << 8) | data[1];
}
void MPU6050::read_raw_gyro() {
    uint8_t* data = this->read_from_register(REG_GYRO_XOUT_H, 6);
    this->raw_gyro[0] = (data[0] << 8) | data[1];
    this->raw_gyro[1] = (data[2] << 8) | data[3];
    this->raw_gyro[2] = (data[4] << 8) | data[5];
}

uint8_t* MPU6050::read_from_register(uint8_t reg, uint8_t len) {
    uint8_t* data = new uint8_t[len];
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    i2c_read_blocking(i2c_default, this->addr, data, len, false);
    return data;
}

void MPU6050::write_to_register(uint8_t reg, uint8_t data) {
    uint8_t buf[] = {reg, data};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

bool MPU6050::test_connection() {
    uint8_t* data = this->read_from_register(REG_WHO_AM_I, 1);
    return *data == this->addr;
}
