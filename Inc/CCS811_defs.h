#ifndef CCS811_DEFS_H_
#define CCS811_DEFS_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#define CCS811_ADDR 0x5A //7-bit unshifted default I2C Address
#define CSS811_RAW_DATA 0x03
#define CSS811_ENV_DATA 0x05
#define CSS811_NTC 0x06
#define CSS811_THRESHOLDS 0x10
#define CSS811_BASELINE 0x11
#define CSS811_HW_VERSION 0x21
#define CSS811_FW_BOOT_VERSION 0x23
#define CSS811_FW_APP_VERSION 0x24
#define CSS811_SW_RESET 0xFF

//Codes
#define CCS811_OK 0x00
#define CCS811_COMM_FAIL 0x01

//Default values //TODO BUGAS, need to mask fuckers
#define CSS811_HW_ID 0x81
#define CSS811_MODE0 0x00 //Measurements disabled
#define CSS811_MODE1 0x10 //Measurements every 1s
#define CSS811_MODE2 0x20 //Measurements every 10s
#define CSS811_MODE3 0x30 //Measurements every 60s
#define CSS811_MODE4 0x40 //Measurements every 250ms

//Erorrs
#define CS811_ERROR_0 0x1  // Write Reg Invalid
#define CS811_ERROR_1 0x2  // Read Reg Invalid
#define CS811_ERROR_2 0x4  // Measurement mode Invalid
#define CS811_ERROR_3 0x8  // Max resistance/out of range
#define CS811_ERROR_4 0x10 // Heater fault current
#define CS811_ERROR_5 0x20 // Heater supply voltage

//Registers
#define CSS811_HW_ID_ADDR 0x20
#define CSS811_HW_Version_ADDR 0x21
#define CSS811_STATUS_ADDR 0x00
#define CSS811_APP_START_ADDR 0xF4
#define CSS811_MEAS_MODE_ADDR 0x01
#define CSS811_ALG_RESULT_ADDR 0x02
#define CSS811_ERROR_ID_ADDR 0xE0

//Register addresses
#define DATA_EEPROM_BASE 0x0
#define FIRST_BASELINE_ADDRESS_VAL *((uint32_t *)(DATA_EEPROM_BASE + 2))
#define BURN_IN_TIME_ADDRESS_VAL *((uint32_t *)(DATA_EEPROM_BASE + 3))
#define BASELINE_EARLYLIFE_PERIOD_ADDRESS_VAL *((uint32_t *)(DATA_EEPROM_BASE + 4))

#define FIRST_BASELINE_ADDRESS (DATA_EEPROM_BASE + 2)
#define BURN_IN_TIME_ADDRESS (DATA_EEPROM_BASE + 3)
#define BASELINE_EARLYLIFE_PERIOD_ADDRESS (DATA_EEPROM_BASE + 4)

#define BURN_IN_TIME ((48 * 60 * 60) / APPLICATION_RUN_CYCLE)               //48 Hours
#define RUN_IN_TIME ((20 * 60) / APPLICATION_RUN_CYCLE)                     //20 Minutes
#define NEW_MODE_RUN_IN_TIME ((10 * 60) / APPLICATION_RUN_CYCLE)            //10 Minutes
#define BASELINE_EARLYLIFE_PERIOD ((500 * 60 * 60) / APPLICATION_RUN_CYCLE) //500 Hours
#define BASELINE_EL_STORE_PERIOD ((24 * 60 * 60) / APPLICATION_RUN_CYCLE)   //24 Hours
#define BASELINE_AEL_STORE_PERIOD ((7 * BASELINE_EL_STORE_PERIOD) / APPLICATION_RUN_CYCLE)
#define CALIB_TEMP_HUM ((20 * 60) / APPLICATION_RUN_CYCLE) //20 Minutes

/*!
 * @brief Type definitions
 */
typedef int8_t (*ccs811_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*ccs811_delay_fptr_t)(uint32_t period);

struct ccs811_data
{
    uint32_t eCO2;
    uint32_t VOCS;
};

struct ccs811_settings
{
    uint8_t mode;
};
struct ccs811_compensation_data
{
    uint32_t temperature;
    uint32_t humidity;
};

struct ccs811_dev
{
    /*! Chip Id */
    uint8_t chip_id;

    /*! Device Id */
    uint8_t dev_id;

    /*! Read function pointer */
    ccs811_com_fptr_t read;

    /*! Write function pointer */
    ccs811_com_fptr_t write;

    /*! Delay function pointer */
    ccs811_delay_fptr_t delay_ms;

    /*! Sensor settings */
    struct ccs811_settings settings;

    struct ccs811_compensation_data comp_data;
};

#endif
