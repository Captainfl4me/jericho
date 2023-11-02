#include "BMP280.hpp"
#include <iostream>

BMP280::BMP280(): I2cSensor(0x76, 100) {
    this->addr = 0x76;
    this->init();
}

BMP280::BMP280(uint8_t addr): I2cSensor(addr, 100) {
    this->addr = addr;
    this->init();
}

void BMP280::init() {
    this->write_to_register(BMP280_REG_CTRL_MEAS, 0b00101111); // Normal mode | oversampling press x4 | oversampling temp x1
    this->write_to_register(BMP280_REG_CONFIG, 0b00001000); // Standby time 0.5ms | IIR filter x4
    this->fetchCalibParams();
}

void BMP280::fetchCalibParams() {
    int response;
    uint8_t* data = this->read_from_register(BMP280_REG_DIG_T1_LSB, 24);

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

int32_t BMP280::compute_fine_res_temperature(int32_t raw_temp) {
    int32_t var1, var2;
    var1 = ((((raw_temp >> 3) - ((int32_t)this->calib_param.dig_T1 << 1))) * ((int32_t)this->calib_param.dig_T2)) >> 11;
    var2 = (((((raw_temp >> 4) - ((int32_t)this->calib_param.dig_T1)) * ((raw_temp >> 4) - ((int32_t)this->calib_param.dig_T1))) >> 12) * ((int32_t)this->calib_param.dig_T3)) >> 14;
    return var1 + var2;
}

int32_t BMP280::compensate_pressure(int32_t raw_pressure, int32_t fine_temp) {
    int64_t var1, var2, p;
    var1 = ((int64_t)fine_temp) - 128000;
    var2 = (var1 * var1 * ((int64_t)this->calib_param.dig_P6));
    var2 += ((var1 * ((int64_t)this->calib_param.dig_P5)) << 17);
    var2 += (((int64_t)this->calib_param.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)this->calib_param.dig_P3) >> 8) + ((var1 * (int32_t)this->calib_param.dig_P2) << 12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)this->calib_param.dig_P1)>>33;
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p = 1048576-raw_pressure;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)this->calib_param.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 =  (((int64_t)this->calib_param.dig_P8) * p) >> 19;
    return (uint32_t)((p + var1 + var2) >> 8) + (((int64_t)this->calib_param.dig_P7)<<4);
}

bool BMP280::update() {
    if (!this->I2cSensor::update()) return false;

    int32_t raw_pressure, raw_temp;
    uint32_t* data = this->read_from_24bregister_LE(BMP280_REG_PRESSURE_MSB, 2);

    raw_pressure = data[0] >> 4;
    raw_temp = data[1] >> 4;

    // Convert temperature calibration data to 32-bits
    int32_t fine_temp = this->compute_fine_res_temperature(raw_temp);
    this->data.temp = ((fine_temp * 5 + 128) >> 8) / 100;
    this->data.pressure = this->compensate_pressure(raw_pressure, fine_temp) / 256.0;
    return true;
}

bool BMP280::test_connection() {
    uint8_t* data = this->read_from_register(BMP280_REG_ID, 1);
    return *data == 0x58;
}
