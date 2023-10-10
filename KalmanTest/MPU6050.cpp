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
    this->writeToRegister(REG_PWR_MGMT_1, 0x00); // PWR_MGMT_1
    this->writeToRegister(REG_GYRO_CONFIG, 0x10); // GYRO_CONFIG
    this->writeToRegister(REG_ACCEL_CONFIG, 0x00); // ACCEL_CONFIG
    this->writeToRegister(REG_CONFIG, 0x06); // CONFIG
}

void MPU6050::updateData() {
    uint8_t* data = this->readFromRegister(REG_ACCEL_XOUT_H, 14);
    this->raw_acc[0] = (data[0] << 8) | data[1];
    this->raw_acc[1] = (data[2] << 8) | data[3];
    this->raw_acc[2] = (data[4] << 8) | data[5];
    this->raw_temp = (data[6] << 8) | data[7];
    this->raw_gyro[0] = (data[8] << 8) | data[9];
    this->raw_gyro[1] = (data[10] << 8) | data[11];
    this->raw_gyro[2] = (data[12] << 8) | data[13];
    delete[] data;

    this->acc[0] = (float)(this->raw_acc[0] / 16384.0);
    this->acc[1] = (float)(this->raw_acc[1] / 16384.0);
    this->acc[2] = (float)(this->raw_acc[2] / 16384.0);
    this->temp = ((int16_t)this->raw_temp) / 340.0 + 36.53;
    this->gyro[0] = (float)(this->raw_gyro[0] / 32.8);
    this->gyro[1] = (float)(this->raw_gyro[1] / 32.8);
    this->gyro[2] = (float)(this->raw_gyro[2] / 32.8);
}

uint8_t* MPU6050::readFromRegister(uint8_t reg, uint8_t len) {
    uint8_t* data = new uint8_t[len];
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    i2c_read_blocking(i2c_default, this->addr, data, len, false);
    return data;
}

void MPU6050::writeToRegister(uint8_t reg, uint8_t data) {
    uint8_t buf[] = {reg, data};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

bool MPU6050::testConnection() {
    uint8_t* data = this->readFromRegister(REG_WHO_AM_I, 1);
    return *data == this->addr;
}
