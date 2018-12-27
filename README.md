# BME280 Example

* Pin assignment:

   *    GPIO21 is assigned as the data signal of i2c master port
   *    GPIO22 is assigned as the clock signal of i2c master port
   *    [BME280_driver](https://github.com/BoschSensortec/BME280_driver)
   *    [SSD1306_driver](https://github.com/lexus2k/ssd1306)
*    Change the OELD configuration file, the path is in `components\ssd1306\src\ssd1306_hal\UserSettings.h`, and close the macro definition except `CONFIG_ADAFRUIT_GFX_ENABLE`, `CONFIG_PLATFORM_I2C_ENABLE`.
 