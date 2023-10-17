#include "logger.hpp"

Logger* Logger::logger = nullptr;

Logger::Logger(uint8_t miso_gpio, uint8_t ss_gpio, uint8_t sck_gpio, uint8_t mosi_gpio, uint32_t baud_rate, spi_inst_t* hw_inst) {
    Logger::logger = this;

    spi_t* p_spi = new spi_t;
    memset(p_spi, 0, sizeof(spi_t));
    if (!p_spi) printf("Logger: p_spi is null");
    p_spi->hw_inst = hw_inst;  // SPI component
    p_spi->miso_gpio = miso_gpio;  // GPIO number (not pin number)
    p_spi->mosi_gpio = mosi_gpio;
    p_spi->sck_gpio = sck_gpio;
    p_spi->baud_rate = 12500 * 1000; 
    add_spi(p_spi);

    // Hardware Configuration of the SD Card "object"
    this->sd_card = new sd_card_t;
    if (!sd_card) printf("Logger: sd_card is null");
    memset(sd_card, 0, sizeof(sd_card_t));
    sd_card->pcName = "0:";  // Name used to mount device
    sd_card->spi = p_spi;    // Pointer to the SPI driving this card
    sd_card->ss_gpio = ss_gpio;   // The SPI slave select GPIO for this SD card
    sd_card->use_card_detect = false;
    sd_card->card_detect_gpio = -1;  // Card detect
    sd_card->card_detected_true = -1;
    add_sd_card(sd_card);

    FRESULT fr = f_mount(&this->sd_card->fatfs, this->sd_card->pcName, 1);
    if (FR_OK != fr) printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);

    fr = f_chdrive(this->sd_card->pcName);
    if (FR_OK != fr) printf("f_chdrive error: %s (%d)\n", FRESULT_str(fr), fr);

#ifdef DEBUG
    printf("Listing folder...\n");
#endif
    // Check if run folder exists
    DIR dj;      /* Directory object */
    FILINFO fno; /* File information */
    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);
    fr = f_findfirst(&dj, &fno, "/", "*");
    this->_run = 0;

    while (fr == FR_OK && fno.fname[0]) { /* Repeat while an item is found */
        /* Create a string that includes the file name, the file size and the
         attributes string. */
        /* Point pcAttrib to a string that describes the file. */
        if (fno.fattrib & AM_DIR) {
#ifdef DEBUG
            printf("Found: %s [size=%llu]\n", fno.fname, fno.fsize);
#endif
            this->_run++;
        }
        fr = f_findnext(&dj, &fno); /* Search for next item */
    }
    f_closedir(&dj);

#ifdef DEBUG
    printf("Creating folder...\n");
#endif
    // Create new run folder
    char dir_name[10];
    int str_length = sprintf(dir_name, "run_%d", this->_run);
    fr = f_mkdir(dir_name);
    if (FR_OK != fr) printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
    this->dir_name = (char*)malloc(str_length + 1);
    strcpy(this->dir_name, dir_name);

#ifdef DEBUG
    printf("Create data file...\n");
#endif
    FIL file;
    // Create new data file
    char filename[20];
    sprintf(filename, "%s/%s", dir_name, this->data_filename);
    fr = f_open(&file, filename, FA_WRITE|FA_CREATE_NEW);
    if (FR_OK != fr) printf("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    if (f_printf(&file, "time,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n") < 0) printf("f_printf failed\n");
    f_close(&file);

#ifdef DEBUG
    printf("Create log file...\n");
#endif
    // Create new log file
    sprintf(filename, "%s/%s", dir_name, this->log_filename);
    fr = f_open(&file, filename, FA_WRITE|FA_CREATE_NEW);
    if (FR_OK != fr) printf("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    if (f_printf(&file, "---------- LOG ----------\n") < 0) printf("f_printf failed\n");
    f_close(&file);
}

void Logger::write_log(const char* message) {
    FIL file;
    char filename[20];
    sprintf(filename, "%s/%s", this->dir_name, this->log_filename);
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND);
    if (FR_OK != fr) printf("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    if (f_printf(&file, "%d us [LOG] : %s\n", time_us_32(), message) < 0) printf("f_printf failed\n");
#ifdef DEBUG
    printf("%d us [LOG] : %s\n", time_us_32(), message);
#endif
    f_close(&file);
}

void Logger::write_error(const char* message) {
    FIL file;
    char filename[20];
    sprintf(filename, "%s/%s", this->dir_name, this->log_filename);
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND);
    if (FR_OK != fr) printf("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    if (f_printf(&file, "%d us [ERR] : %s\n", time_us_32(), message) < 0) printf("f_printf failed\n");
#ifdef DEBUG
    printf("%d us [ERR] : %s\n", time_us_32(), message);
#endif
    f_close(&file);
}

void Logger::write_data(uint32_t time, int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z) {
    FIL file;
    char filename[20];
    sprintf(filename, "%s/%s", this->dir_name, this->data_filename);
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND);
    if (FR_OK != fr) printf("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    if (f_printf(&file, "%d,%d,%d,%d,%d,%d,%d\n", time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z) < 0) printf("f_printf failed\n");
#ifdef DEBUG
    printf("%d,%d,%d,%d,%d,%d,%d\n", time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
#endif
    f_close(&file);
}

Logger::~Logger() {
    f_unmount(this->sd_card->pcName);
}
