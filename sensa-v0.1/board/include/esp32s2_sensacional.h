#ifndef _ESP32S2_SENSACIONAL_H
#define _ESP32S2_SENSACIONAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "rom/ets_sys.h"
#include "bme280.h"
//#include "sensirion_config.h"
#include "sgp40_i2c.h"


#define I2C_MASTER_SCL_IO                  2  
#define I2C_MASTER_SDA_IO                  1  

#define UART_NUM                    UART_NUM_1
    
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000


#define BME280_ADDR                 BME280_I2C_ADDR_PRIM
#define SGP40_ADDR                  0x59

#define LED_PIN 9

uint16_t BSP_GetVersion(void);

void I2C_init(void);
void I2C_deinit(void);
void I2C_scan(void);

void BSP_UART_Init();
void BSP_UART_Send(char* command);
int8_t BSP_UART_Read(uint8_t* buff_data);

void BSP_LED_Init(uint8_t pin);
void BSP_LED_Write(uint8_t pin, uint8_t value);

void BSP_BME280_Init(void);
void BSP_BME280_Deinit(void);
void BSP_BME280_Get_Temperature(double *temp_ptr);
void BSP_BME280_Get_Pressure(double *pres_ptr);
void BSP_BME280_Get_Humidity(double *hum_ptr);

void BSP_SGP40_Init();
void BSP_SGP40_Deinit();
/**
 * sgp40_measure_raw_signal() - This command starts/continues the VOC
 * measurement mode
 *
 * @param relative_humidity Leaves humidity compensation disabled by sending the
 * default value 0x8000 (50%RH) or enables humidity compensation when sending
 * the relative humidity in ticks (ticks = %RH * 65535 / 100)
 *
 * @param temperature Leaves humidity compensation disabled by sending the
 * default value 0x6666 (25 degC) or enables humidity compensation when sending
 * the temperature in ticks (ticks = (degC + 45) * 65535 / 175)
 *
 * @param sraw_voc u16 unsigned integer directly provides the raw signal
 * SRAW_VOC in ticks which is proportional to the logarithm of the resistance of
 * the sensing element.
 *
 * @return 0 on success, an error code otherwise
 */
void BSP_SGP40_Get_Raw(uint16_t relative_humidity,
    uint16_t temperature, uint16_t* sraw_voc);

/**
* sgp40_execute_self_test() - This command triggers the built-in self-test
* checking for integrity of the hotplate and MOX material and returns the
* result of this test as 2 bytes
*
* @param test_result 0xD4 00: all tests passed successfully or 0x4B 00: one or
* more tests have failed
*
* @return 0 on success, an error code otherwise
*/
void BSP_SGP40_Self_Test(uint16_t* test_result);

/**
* sgp40_turn_heater_off() - This command turns the hotplate off and stops the
* measurement. Subsequently, the sensor enters the idle mode.
*
* @return 0 on success, an error code otherwise
*/
void BSP_SGP40_Turn_Heater_Off(void);

/**
* sgp40_get_serial_number() - This command provides the decimal serial number
* of the SGP40 chip by returning 3x2 bytes.
*
* @param serial_number 48-bit unique serial number
*
* @return 0 on success, an error code otherwise
*/
void BSP_SGP40_Get_Serial_Number(uint16_t* serial_number,
   uint8_t serial_number_size);

#ifdef __cplusplus
}
#endif


#endif