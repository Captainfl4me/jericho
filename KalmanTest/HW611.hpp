#include <cstdlib>
#include "pico/types.h"
#include "hardware/i2c.h"

#define REG_WHO_AM_I 0xD0

class HW611 {
    uint8_t addr;
    
    public:
    uint16_t raw_pressure;

    HW611();
    HW611(uint8_t addr);
    
    bool testConnection();

    private:
    uint8_t* readFromRegister(uint8_t reg, uint8_t len);
    void writeToRegister(uint8_t reg, uint8_t data);
};
