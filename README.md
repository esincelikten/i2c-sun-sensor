# i2c-sun-sensor
STM32 HAL driver code for interfacing with the ADS7138 ADC over I2C (sun sensor data acquisition).
This code uses HAL functions written by stm32cubeID

*Names of functions and their work*

-static HAL_StatusTypeDef ads_write(uint8_t reg, uint8_t val)
  Write a single byte value to sun sensor's register
  
-static HAL_StatusTypeDef ads_read_frame(uint8_t *buf, uint16_t nbytes)
  Read a frame of -nbytes- from sensor through I2C and calls a buffer to store data

-static uint8_t i2c_scan_find(uint8_t want_addr7)
  Scans for I2C bus and reports if it finds any

-static void ads_init_manual_mode(void)
  For setting msanual mode

-__weak void adcs_send(uint8_t *data, uint16_t len)
  Hook for sending the measured ADC sample to somewhere else


I²C addressing
7-bit, using 0x13 (actual wire addr 0x26)


I²C speed	
Likely 100 kHz (default), adjustable to 400 kHz

Operation mode
I have set 't to manuel can be switched to sequence


Timeout
50 ms per transaction






















