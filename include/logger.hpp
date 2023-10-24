# ifndef LOGGER_HPP
# define LOGGER_HPP

#include <cstdint>
#include <string.h>
#include <stdexcept>
#include "pico/stdlib.h"

#include "f_util.h"
#include "ff.h"
#include "rtc.h"
#include "diskio.h" /* Declarations of disk functions */
#include "hw_config.h"

#include "vector.hpp"

#define FIFO_SIZE 50

void add_spi(spi_t *spi);
void add_sd_card(sd_card_t *sd_card);

struct data_t{
    uint32_t time;
    float_vector3_t acc;
    float_vector3_t gyro;
};

class Logger {
    private:
    uint8_t _run;
    char* dir_name;
    sd_card_t* sd_card;

    data_t fifo[FIFO_SIZE];
    uint8_t fifo_head;
    uint8_t fifo_tail;

    public:
    Logger(uint8_t miso_gpio, u_int8_t ss_gpio, uint8_t sck_gpio, uint8_t mosi_gpio, uint32_t baud_rate, spi_inst_t* hw_inst);
    ~Logger();

    void write_log(const char* message);
    void write_error(const char* message);
    void write_data(uint32_t time, int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z);
    
    void push_data_to_fifo(data_t* data);
    bool is_fifo_empty();
    int write_all_data_from_fifo();

    static Logger* logger;
    
    const char* log_filename = "log.txt";
    const char* data_filename = "data.csv";
};

#endif