#include "HW611.hpp"

HW611::HW611() {
    this->addr = 0x58;
}

HW611::HW611(uint8_t addr) {
    this->addr = addr;
}

uint8_t* HW611::readFromRegister(uint8_t reg, uint8_t len) {
    uint8_t* data = new uint8_t[len];
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    i2c_read_blocking(i2c_default, this->addr, data, len, false);
    return data;
}

void HW611::writeToRegister(uint8_t reg, uint8_t data) {
    uint8_t buf[] = {reg, data};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

bool HW611::testConnection() {
    uint8_t* data = this->readFromRegister(REG_WHO_AM_I, 1);
    return *data == this->addr;
}
