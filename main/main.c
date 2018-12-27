#include <stdio.h>
#include "driver/i2c.h"
#include "bme280.h"
#include "ssd1306.h"

/**
 * TEST CODE BRIEF
 *
 * - read external i2c sensor, here we use a bme280  sensor for instance.
 * - Place read data on OLED
 *
 * Pin assignment:
 *
 *    GPIO21 is assigned as the data signal of i2c master port
 *    GPIO22 is assigned as the clock signal of i2c master port
 *    BME280_driver: https://github.com/BoschSensortec/BME280_driver
 *
 */

#define I2C_SCL_PIN_NUM 22          /*!< gpio number for I2C master clock */
#define I2C_SDA_PIN_NUM 21          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1    /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
    vTaskDelay(period / portTICK_PERIOD_MS);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int ret = 0; /* Return 0 for Success, non-zero for failure */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_id << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_id << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, reg_data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int ret; /* Return 0 for Success, non-zero for failure */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_id << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, len, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void print_sensor_data(struct bme280_data *comp_data)
{
    // #ifdef BME280_FLOAT_ENABLE
    //     printf("Temperature:%0.2f, Pressure:%0.2f, Humidity:%0.2f\r\n", comp_data->temperature, comp_data->pressure, comp_data->humidity);
    // #else
    //     printf("Temperature: %d, Pressure:%d, Humidity:%d\r\n", comp_data->temperature, comp_data->pressure, comp_data->humidity);
    // #endif
    char buff[512];
    ssd1306_clearScreen();
    snprintf(buff, sizeof(buff), "Temp    :%0.2f", comp_data->temperature);
    ssd1306_printFixed(0, 8, buff, STYLE_NORMAL);

    snprintf(buff, sizeof(buff), "Pressure:%0.2f", comp_data->pressure);
    ssd1306_printFixed(0, 16, buff, STYLE_NORMAL);

    snprintf(buff, sizeof(buff), "Humidity:%0.2f", comp_data->humidity);
    ssd1306_printFixed(0, 24, buff, STYLE_NORMAL);
}

int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;
    dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, dev);
    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

    while (1)
    {
        /* Delay while the sensor completes a measurement */
        dev->delay_ms(1000);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        print_sensor_data(&comp_data);
    }

    return rslt;
}

void app_main()
{

    ssd1306_setFixedFont(ssd1306xled_font6x8);

    /*
    *   default use I2C_NUM_1
    *   SDA --> 21 
    *   SCL --> 22
    */
    ssd1306_128x64_i2c_init();

    ssd1306_clearScreen();

    ssd1306_printFixed(0, 8, "BME280 TEST Project", STYLE_NORMAL);

    delay(3000);

    /*
    *
    * The default SSD130 library uses 21, 22 as SDA, SCL, 
    * and uses I2C_NUM_1. The TWI has been initialized in the ssd1306_128x64_i2c_init function, 
    * so there is no need to initialize it again here.
    */
   /*
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN_NUM;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_PIN_NUM;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
    */
    struct bme280_dev dev;
    int8_t rslt = BME280_OK;

    dev.dev_id = BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    rslt = bme280_init(&dev);
    if (rslt != BME280_OK)
    {
        printf("BME280 INIT FAIL rslt:%d\n", rslt);
        return;
    }
    stream_sensor_data_normal_mode(&dev);
}
