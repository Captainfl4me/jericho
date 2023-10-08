#include <cstdlib>
#include "pico/types.h"
#include "hardware/i2c.h"

class MPU6050 {
    uint8_t addr = 0x68;

    public:
    uint16_t raw_acc[3]; // x, y, z
    uint16_t raw_gyro[3]; // x, y, z
    uint16_t raw_temp;

    float acc[3]; // x, y, z
    float gyro[3]; // x, y, z
    float temp;

    MPU6050();
    MPU6050(uint8_t addr);
    void init();

    void updateData();
    bool testConnection();

    private:
    uint8_t* readFromRegister(uint8_t reg, uint8_t len);
    void writeToRegister(uint8_t reg, uint8_t data);
};
