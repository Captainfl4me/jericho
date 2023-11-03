#include "MPU6050.hpp"

MPU6050::MPU6050(): I2cSensor(MPU_DEFAULT_I2C_ADDR, MPU_DEFAULT_I2C_FREQ) { this->init(); }
MPU6050::MPU6050(uint8_t addr, uint8_t freq): I2cSensor(addr, freq) { this->init(); }
MPU6050::MPU6050(uint8_t addr, i2c_inst_t *i2c_port): I2cSensor(addr, MPU_DEFAULT_I2C_FREQ, i2c_port) { this->init(); }

void MPU6050::init() {
    this->write_to_register(MPU_REG_PWR_MGMT_1, 0b00000000); // PWR_MGMT_1
    this->write_to_register(MPU_REG_GYRO_CONFIG, 0b00010000); // GYRO_CONFIG
    this->write_to_register(MPU_REG_ACCEL_CONFIG, 0b00000000); // ACCEL_CONFIG
    this->write_to_register(MPU_REG_CONFIG, 0b00000001); // CONFIG

    this->config.gyro_offset.x = 0;
    this->config.gyro_offset.y = 0;
    this->config.gyro_offset.z = 0;
}

void MPU6050::calibrate(uint16_t samples) {
    vector3<int16_t> gyro_sum = {0, 0, 0};
    for (uint16_t i = 0; i < samples; i++) {
        vector3<int16_t> raw_gyro;
        uint8_t* data = this->read_from_register(MPU_REG_GYRO_XOUT_H, 6);
        raw_gyro.x = (data[0] << 8) | data[1];
        raw_gyro.y = (data[2] << 8) | data[3];
        raw_gyro.z = (data[4] << 8) | data[5];

        gyro_sum.x += raw_gyro.x;
        gyro_sum.y += raw_gyro.y;
        gyro_sum.z += raw_gyro.z;
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
    this->write_to_register(MPU_REG_GYRO_CONFIG, scale << 3);
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
    this->write_to_register(MPU_REG_ACCEL_CONFIG, range << 3);
}

bool MPU6050::update() {
    if (!I2cSensor::update()) return false;

    vector3<int16_t> raw_acc, raw_gyro;
    int16_t raw_temp;
    uint8_t* data = this->read_from_register(MPU_REG_ACCEL_XOUT_H, 14);
    raw_acc.x = (data[0] << 8) | data[1];
    raw_acc.y = (data[2] << 8) | data[3];
    raw_acc.z = (data[4] << 8) | data[5];
    raw_temp = (data[6] << 8) | data[7];
    raw_gyro.x = (data[8] << 8) | data[9];
    raw_gyro.y = (data[10] << 8) | data[11];
    raw_gyro.z = (data[12] << 8) | data[13];

    this->data.acc.x = (float)(raw_acc.x / this->config.range_per_digit);
    this->data.acc.y = (float)(raw_acc.y / this->config.range_per_digit);
    this->data.acc.z = (float)(raw_acc.z / this->config.range_per_digit);
    this->data.temp = raw_temp / 340 + 36.53f;
    this->data.gyro.x = (((float)raw_gyro.x)- this->config.gyro_offset.x) / this->config.dps_per_digit;
    this->data.gyro.y = (((float)raw_gyro.y) - this->config.gyro_offset.y) / this->config.dps_per_digit;
    this->data.gyro.z = (((float)raw_gyro.z) - - this->config.gyro_offset.z) / this->config.dps_per_digit;
    return true;
}

void MPU6050::update_only_acc() {
    vector3<int16_t> raw_acc;
    uint8_t* data = this->read_from_register(MPU_REG_ACCEL_XOUT_H, 6);
    raw_acc.x = (data[0] << 8) | data[1];
    raw_acc.y = (data[2] << 8) | data[3];
    raw_acc.z = (data[4] << 8) | data[5];

    this->data.acc.x = (float)(raw_acc.x / this->config.range_per_digit);
    this->data.acc.y = (float)(raw_acc.y / this->config.range_per_digit);
    this->data.acc.z = (float)(raw_acc.z / this->config.range_per_digit);
}

void MPU6050::update_only_temp() {
    int16_t raw_temp;
    uint8_t* data = this->read_from_register(MPU_REG_TEMP_OUT_H, 2);

    raw_temp = (data[0] << 8) | data[1];
    this->data.temp = ((int16_t)raw_temp) / 340 + 36.53f;
}

void MPU6050::update_only_gyro() {
    vector3<int16_t> raw_gyro;
    uint8_t* data = this->read_from_register(MPU_REG_GYRO_XOUT_H, 6);
    raw_gyro.x = (data[0] << 8) | data[1];
    raw_gyro.y = (data[2] << 8) | data[3];
    raw_gyro.z = (data[4] << 8) | data[5];

    this->data.gyro.x = (((float)raw_gyro.x)- this->config.gyro_offset.x) / this->config.dps_per_digit;
    this->data.gyro.y = (((float)raw_gyro.y) - this->config.gyro_offset.y) / this->config.dps_per_digit;
    this->data.gyro.z = (((float)raw_gyro.z) - - this->config.gyro_offset.z) / this->config.dps_per_digit;    
}

bool MPU6050::test_connection() {
    uint8_t* data = this->read_from_register(MPU_REG_WHO_AM_I, 1);
    return *data == this->addr;
}
