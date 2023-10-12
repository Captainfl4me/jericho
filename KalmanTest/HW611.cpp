#include "HW611.hpp"
#include <iostream>

HW611::HW611() {
    this->addr = 0x76;
    this->init();
}

HW611::HW611(uint8_t addr) {
    this->addr = addr;
    this->init();
}

void HW611::init() {
    this->writeToRegister(REG_CTRL_MEAS, 0b00101111);
    this->writeToRegister(REG_CONFIG, 0x00001000);
    this->fetchCalibParams();
}

void HW611::fetchCalibParams() {
    int response;
    uint8_t* data = this->readFromRegister(REG_DIG_T1_LSB, 24, &response);

#ifdef DEBUG
    if (response == PICO_ERROR_GENERIC) {
        printf("Error fetching calibration parameters\n");
        return;
    }
#endif

    this->calib_param.dig_T1 = (uint16_t)(data[1] << 8) | data[0];
    this->calib_param.dig_T2 = (int16_t)(data[3] << 8) | data[2];
    this->calib_param.dig_T3 = (int16_t)(data[5] << 8) | data[4];

    this->calib_param.dig_P1 = (uint16_t)(data[7] << 8) | data[6];
    this->calib_param.dig_P2 = (int16_t)(data[9] << 8) | data[8];
    this->calib_param.dig_P3 = (int16_t)(data[11] << 8) | data[10];
    this->calib_param.dig_P4 = (int16_t)(data[13] << 8) | data[12];
    this->calib_param.dig_P5 = (int16_t)(data[15] << 8) | data[14];
    this->calib_param.dig_P6 = (int16_t)(data[17] << 8) | data[16];
    this->calib_param.dig_P7 = (int16_t)(data[19] << 8) | data[18];
    this->calib_param.dig_P8 = (int16_t)(data[21] << 8) | data[20];
    this->calib_param.dig_P9 = (int16_t)(data[23] << 8) | data[22];
}

int32_t HW611::fineResTemperature() {
    int32_t var1, var2;
    var1 = ((((this->raw_temp >> 3) - ((int32_t)this->calib_param.dig_T1 << 1))) * ((int32_t)this->calib_param.dig_T2)) >> 11;
    var2 = (((((this->raw_temp >> 4) - ((int32_t)this->calib_param.dig_T1)) * ((this->raw_temp >> 4) - ((int32_t)this->calib_param.dig_T1))) >> 12) * ((int32_t)this->calib_param.dig_T3)) >> 14;
    return var1 + var2;
}

int32_t HW611::compensatePressure() {
    int64_t var1, var2, p;
    var1 = ((int64_t)this->fineResTemperature()) - 128000;
    var2 = (var1 * var1 * ((int64_t)this->calib_param.dig_P6));
    var2 += ((var1 * ((int64_t)this->calib_param.dig_P5)) << 17);
    var2 += (((int64_t)this->calib_param.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)this->calib_param.dig_P3) >> 8) + ((var1 * (int32_t)this->calib_param.dig_P2) << 12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)this->calib_param.dig_P1)>>33;
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p = 1048576-this->raw_pressure;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)this->calib_param.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 =  (((int64_t)this->calib_param.dig_P8) * p) >> 19;
    return (uint32_t)((p + var1 + var2) >> 8) + (((int64_t)this->calib_param.dig_P7)<<4);
}

void HW611::updateData() {
    uint8_t* data = this->readFromRegister(REG_PRESSURE_MSB, 6);

    this->raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    this->raw_temp = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    delete[] data;

    // Convert temperature calibration data to 32-bits
    this->temp = ((this->fineResTemperature() * 5 + 128) >> 8) / 100;
    this->pressure = this->compensatePressure() / 256.0;
}

uint8_t* HW611::readFromRegister(uint8_t reg, uint8_t len) {
    uint8_t* data = new uint8_t[len];
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    i2c_read_blocking(i2c_default, this->addr, data, len, false);
    return data;
}

uint8_t* HW611::readFromRegister(uint8_t reg, uint8_t len, int* res) {
    uint8_t* data = new uint8_t[len];
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    *res = i2c_read_blocking(i2c_default, this->addr, data, len, false);
    return data;
}

int HW611::writeToRegister(uint8_t reg, uint8_t data) {
    uint8_t buf[] = {reg, data};
    return i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

bool HW611::testConnection() {
    uint8_t* data = this->readFromRegister(REG_ID, 1);
    return *data == 0x58;
}
