#ifndef I2C_SENSOR_HPP
#define I2C_SENSOR_HPP

#include <cstdlib>
#include <stdexcept>
#include "pico/types.h"
#include "hardware/i2c.h"
#include "vector.hpp"

template <class T>
class I2cSensor {
protected:
    uint8_t addr;
    uint16_t freq;
    i2c_inst_t *i2c_port;

    uint32_t last_update_time = 0;

public:
    T data;
    I2cSensor(uint8_t addr, uint16_t freq);
    I2cSensor(uint8_t addr, uint16_t freq, i2c_inst_t *i2c_port);

    virtual void init() = 0;
    bool update();
    virtual bool test_connection() = 0;

protected:
    void write_to_register(uint8_t reg, uint8_t data);
    uint8_t* read_from_register(uint8_t reg, uint8_t len);

    void write_to_16bregister_LE(uint8_t reg, uint16_t data);
    uint16_t* read_from_16bregister_LE(uint8_t reg, uint8_t len);

    void write_to_16bregister_BE(uint8_t reg, uint16_t data);
    uint16_t* read_from_16bregister_BE(uint8_t reg, uint8_t len);

    void write_to_24bregister_LE(uint8_t reg, uint32_t data);
    uint32_t* read_from_24bregister_LE(uint8_t reg, uint8_t len);

    void write_to_24bregister_BE(uint8_t reg, uint32_t data);
    uint32_t* read_from_24bregister_BE(uint8_t reg, uint8_t len);

    void write_to_32bregister_LE(uint8_t reg, uint32_t data);
    uint32_t* read_from_32bregister_LE(uint8_t reg, uint8_t len);

    void write_to_32bregister_BE(uint8_t reg, uint32_t data);
    uint32_t* read_from_32bregister_BE(uint8_t reg, uint8_t len);
};

template <class T>
I2cSensor<T>::I2cSensor(uint8_t addr, uint16_t freq) {
    this->addr = addr;
    this->freq = freq;
    this->i2c_port = i2c_default;
}

template <class T>
I2cSensor<T>::I2cSensor(uint8_t addr, uint16_t freq, i2c_inst_t *i2c_port) {
    this->addr = addr;
    this->freq = freq;
    this->i2c_port = i2c_port;
}

template <class T>
bool I2cSensor<T>::update() {
    if ((time_us_32() - this->last_update_time) < 1000000 / this->freq) return false;
    this->last_update_time = time_us_32();
    return true;
}

template <class T>
void I2cSensor<T>::write_to_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(this->i2c_port, this->addr, buf, 2, false);
}
template <class T>
uint8_t* I2cSensor<T>::read_from_register(uint8_t reg, uint8_t len) {
    uint8_t* data = new uint8_t[len];
    i2c_write_blocking(this->i2c_port, this->addr, &reg, 1, true);
    i2c_read_blocking(this->i2c_port, this->addr, data, len, false);
    return data;
}

template <class T>
void I2cSensor<T>::write_to_16bregister_LE(uint8_t reg, uint16_t data) {
    uint8_t buf[3] = {reg, (uint8_t)(data & 0xff), (uint8_t)(data >> 8)};
    i2c_write_blocking(this->i2c_port, this->addr, buf, 3, false);
}
template <class T>
uint16_t* I2cSensor<T>::read_from_16bregister_LE(uint8_t reg, uint8_t len) {
    uint8_t* data = new uint8_t[len * 2];
    i2c_write_blocking(this->i2c_port, this->addr, &reg, 1, true);
    i2c_read_blocking(this->i2c_port, this->addr, data, len * 2, false);
    uint16_t* data_16b = new uint16_t[len];
    for (uint8_t i = 0; i < len; i++) {
        data_16b[i] = (data[i * 2] << 8) | data[i * 2 + 1];
    }
    delete[] data;
    return data_16b;
}

template <class T>
void I2cSensor<T>::write_to_16bregister_BE(uint8_t reg, uint16_t data) {
    uint8_t buf[3] = {reg, (uint8_t)(data >> 8), (uint8_t)(data & 0xff)};
    i2c_write_blocking(this->i2c_port, this->addr, buf, 3, false);
}
template <class T>
uint16_t* I2cSensor<T>::read_from_16bregister_BE(uint8_t reg, uint8_t len) {
    uint8_t* data = new uint8_t[len * 2];
    i2c_write_blocking(this->i2c_port, this->addr, &reg, 1, true);
    i2c_read_blocking(this->i2c_port, this->addr, data, len * 2, false);
    uint16_t* data_16b = new uint16_t[len];
    for (uint8_t i = 0; i < len; i++) {
        data_16b[i] = (data[i * 2 + 1] << 8) | data[i * 2];
    }
    delete[] data;
    return data_16b;
}

template <class T>
void I2cSensor<T>::write_to_24bregister_LE(uint8_t reg, uint32_t data) {
    uint8_t buf[4] = {reg, (uint8_t)(data & 0xff), (uint8_t)(data >> 8), (uint8_t)(data >> 16)};
    i2c_write_blocking(this->i2c_port, this->addr, buf, 4, false);
}
template <class T>
uint32_t* I2cSensor<T>::read_from_24bregister_LE(uint8_t reg, uint8_t len) {
    uint8_t* data = new uint8_t[len * 3];
    i2c_write_blocking(this->i2c_port, this->addr, &reg, 1, true);
    i2c_read_blocking(this->i2c_port, this->addr, data, len * 3, false);
    uint32_t* data_24b = new uint32_t[len];
    for (uint8_t i = 0; i < len; i++) {
        data_24b[i] = (data[i * 3] << 16) | (data[i * 3 + 1] << 8) | data[i * 3 + 2];
    }
    delete[] data;
    return data_24b;
}

template <class T>
void I2cSensor<T>::write_to_24bregister_BE(uint8_t reg, uint32_t data) {
    uint8_t buf[4] = {reg, (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)(data & 0xff)};
    i2c_write_blocking(this->i2c_port, this->addr, buf, 4, false);
}
template <class T>
uint32_t* I2cSensor<T>::read_from_24bregister_BE(uint8_t reg, uint8_t len) {
    uint8_t* data = new uint8_t[len * 3];
    i2c_write_blocking(this->i2c_port, this->addr, &reg, 1, true);
    i2c_read_blocking(this->i2c_port, this->addr, data, len * 3, false);
    uint32_t* data_24b = new uint32_t[len];
    for (uint8_t i = 0; i < len; i++) {
        data_24b[i] = (data[i * 3 + 2] << 16) | (data[i * 3 + 1] << 8) | data[i * 3];
    }
    delete[] data;
    return data_24b;
}

template <class T>
void I2cSensor<T>::write_to_32bregister_LE(uint8_t reg, uint32_t data) {
    uint8_t buf[5] = {reg, (uint8_t)(data & 0xff), (uint8_t)(data >> 8), (uint8_t)(data >> 16), (uint8_t)(data >> 24)};
    i2c_write_blocking(this->i2c_port, this->addr, buf, 5, false);
}
template <class T>
uint32_t* I2cSensor<T>::read_from_32bregister_LE(uint8_t reg, uint8_t len) {
    uint8_t* data = new uint8_t[len * 4];
    i2c_write_blocking(this->i2c_port, this->addr, &reg, 1, true);
    i2c_read_blocking(this->i2c_port, this->addr, data, len * 4, false);
    uint32_t* data_32b = new uint32_t[len];
    for (uint8_t i = 0; i < len; i++) {
        data_32b[i] = (data[i * 4] << 24) | (data[i * 4 + 1] << 16) | (data[i * 4 + 2] << 8) | data[i * 4 + 3];
    }
    delete[] data;
    return data_32b;
}

template <class T>
void I2cSensor<T>::write_to_32bregister_BE(uint8_t reg, uint32_t data) {
    uint8_t buf[5] = {reg, (uint8_t)(data >> 24), (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)(data & 0xff)};
    i2c_write_blocking(this->i2c_port, this->addr, buf, 5, false);
}
template <class T>
uint32_t* I2cSensor<T>::read_from_32bregister_BE(uint8_t reg, uint8_t len) {
    uint8_t* data = new uint8_t[len * 4];
    i2c_write_blocking(this->i2c_port, this->addr, &reg, 1, true);
    i2c_read_blocking(this->i2c_port, this->addr, data, len * 4, false);
    uint32_t* data_32b = new uint32_t[len];
    for (uint8_t i = 0; i < len; i++) {
        data_32b[i] = (data[i * 4 + 3] << 24) | (data[i * 4 + 2] << 16) | (data[i * 4 + 1] << 8) | data[i * 4];
    }
    delete[] data;
    return data_32b;
}
#endif