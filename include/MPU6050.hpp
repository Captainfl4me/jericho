#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "i2c_sensor.hpp"

#define MPU_REG_SELF_TEST_X 0x0D
#define MPU_REG_SELF_TEST_Y 0x0E
#define MPU_REG_SELF_TEST_Z 0x0F
#define MPU_REG_SELF_TEST_A 0x10
#define MPU_REG_SMPLRT_DIV 0x19
#define MPU_REG_CONFIG 0x1A
#define MPU_REG_GYRO_CONFIG 0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_FIFO_EN 0x23
#define MPU_REG_I2C_MST_CTRL 0x24
#define MPU_REG_I2C_SLV0_ADDR 0x25
#define MPU_REG_I2C_SLV0_REG 0x26
#define MPU_REG_I2C_SLV0_CTRL 0x27
#define MPU_REG_I2C_SLV1_ADDR 0x28
#define MPU_REG_I2C_SLV1_REG 0x29
#define MPU_REG_I2C_SLV1_CTRL 0x2A
#define MPU_REG_I2C_SLV2_ADDR 0x2B
#define MPU_REG_I2C_SLV2_REG 0x2C
#define MPU_REG_I2C_SLV2_CTRL 0x2D
#define MPU_REG_I2C_SLV3_ADDR 0x2E
#define MPU_REG_I2C_SLV3_REG 0x2F
#define MPU_REG_I2C_SLV3_CTRL 0x30
#define MPU_REG_I2C_SLV4_ADDR 0x31
#define MPU_REG_I2C_SLV4_REG 0x32
#define MPU_REG_I2C_SLV4_DO 0x33
#define MPU_REG_I2C_SLV4_CTRL 0x34
#define MPU_REG_I2C_SLV4_DI 0x35
#define MPU_REG_I2C_MST_STATUS 0x36
#define MPU_REG_INT_PIN_CFG 0x37
#define MPU_REG_INT_ENABLE 0x38
#define MPU_REG_INT_STATUS 0x3A
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_ACCEL_XOUT_L 0x3C
#define MPU_REG_ACCEL_YOUT_H 0x3D
#define MPU_REG_ACCEL_YOUT_L 0x3E
#define MPU_REG_ACCEL_ZOUT_H 0x3F
#define MPU_REG_ACCEL_ZOUT_L 0x40
#define MPU_REG_TEMP_OUT_H 0x41
#define MPU_REG_TEMP_OUT_L 0x42
#define MPU_REG_GYRO_XOUT_H 0x43
#define MPU_REG_GYRO_XOUT_L 0x44
#define MPU_REG_GYRO_YOUT_H 0x45
#define MPU_REG_GYRO_YOUT_L 0x46
#define MPU_REG_GYRO_ZOUT_H 0x47
#define MPU_REG_GYRO_ZOUT_L 0x48
#define MPU_REG_EXT_SENS_DATA_00 0x49
#define MPU_REG_EXT_SENS_DATA_01 0x4A
#define MPU_REG_EXT_SENS_DATA_02 0x4B
#define MPU_REG_EXT_SENS_DATA_03 0x4C
#define MPU_REG_EXT_SENS_DATA_04 0x4D
#define MPU_REG_EXT_SENS_DATA_05 0x4E
#define MPU_REG_EXT_SENS_DATA_06 0x4F
#define MPU_REG_EXT_SENS_DATA_07 0x50
#define MPU_REG_EXT_SENS_DATA_08 0x51
#define MPU_REG_EXT_SENS_DATA_09 0x52
#define MPU_REG_EXT_SENS_DATA_10 0x53
#define MPU_REG_EXT_SENS_DATA_11 0x54
#define MPU_REG_EXT_SENS_DATA_12 0x55
#define MPU_REG_EXT_SENS_DATA_13 0x56
#define MPU_REG_EXT_SENS_DATA_14 0x57
#define MPU_REG_EXT_SENS_DATA_15 0x58
#define MPU_REG_EXT_SENS_DATA_16 0x59
#define MPU_REG_EXT_SENS_DATA_17 0x5A
#define MPU_REG_EXT_SENS_DATA_18 0x5B
#define MPU_REG_EXT_SENS_DATA_19 0x5C
#define MPU_REG_EXT_SENS_DATA_20 0x5D
#define MPU_REG_EXT_SENS_DATA_21 0x5E
#define MPU_REG_EXT_SENS_DATA_22 0x5F
#define MPU_REG_EXT_SENS_DATA_23 0x60
#define MPU_REG_I2C_SLV0_DO 0x63
#define MPU_REG_I2C_SLV1_DO 0x64
#define MPU_REG_I2C_SLV2_DO 0x65
#define MPU_REG_I2C_SLV3_DO 0x66
#define MPU_REG_RL 0x67
#define MPU_REG_SIGNAL_PATH_RES ET 0x68
#define MPU_REG_USER_CTRL 0x6A
#define MPU_REG_USER_CTRL 0x6A
#define MPU_REG_PWR_MGMT_1 0x6B
#define MPU_REG_PWR_MGMT_2 0x6C
#define MPU_REG_FIFO_COUNTH 0x72
#define MPU_REG_FIFO_COUNTL 0x73
#define MPU_REG_FIFO_R_W 0x74
#define MPU_REG_WHO_AM_I 0x75

#define MPU_DEFAULT_I2C_ADDR 0x68
#define MPU_DEFAULT_I2C_FREQ 1000
#define MPU_AD0_I2C_ADDR 0x69

enum mpu_6050_scale
{
    MPU6050_SCALE_250DPS = 0,
    MPU6050_SCALE_500DPS = 1,
    MPU6050_SCALE_1000DPS = 2,
    MPU6050_SCALE_2000DPS = 3
};

enum mpu_6050_range
{
    MPU6050_RANGE_2G = 0,
    MPU6050_RANGE_4G = 1,
    MPU6050_RANGE_8G = 2,
    MPU6050_RANGE_16G = 3,
};

struct mpu_6050_config
{
    mpu_6050_scale scale;
    float dps_per_digit;

    mpu_6050_range range;
    float range_per_digit;

    vector3<float> gyro_offset;
} typedef mpu6050_config_t;

struct MPU6050_DATA
{
    vector3<float> acc;
    vector3<float> gyro;
    float temp;
} typedef MPU6050_DATA;


class MPU6050: public I2cSensor<MPU6050_DATA>{
    mpu6050_config_t config;

public:
    MPU6050();
    MPU6050(uint8_t addr, uint8_t freq);
    MPU6050(uint8_t addr, i2c_inst_t *i2c_port);
    void init() override;
    bool update();
    bool test_connection() override;

    void calibrate(uint16_t samples);

    void update_only_acc();
    void update_only_temp();
    void update_only_gyro();

    void set_gyro_scale(mpu_6050_scale scale);
    void set_accel_range(mpu_6050_range range);
};

#endif