#include "CCS811.h"

int8_t ccs811_init(struct ccs811_dev *dev)
{
    int8_t rslt;
    uint8_t data[4] = {0x11, 0xE5, 0x72, 0x8A}; //Reset key

    /* chip id read try count */
    uint8_t try_count = 5;
    uint8_t chip_id = 0;

    while (try_count)
    {
        dev->delay_ms(50);
        rslt = dev->read(dev->dev_id, CSS811_HW_ID_ADDR, &chip_id, 1);

        /* Check for chip id validity */
        if ((rslt == CCS811_OK) && (chip_id == CSS811_HW_ID))
        {
            dev->chip_id = chip_id;
            // sw reset

            rslt = dev->write(dev->dev_id, CSS811_SW_RESET, data, 4);
            dev->delay_ms(100);
            /* Start sensor */

            rslt = dev->write(dev->dev_id, CSS811_APP_START_ADDR, 0, 0);
            dev->delay_ms(70);

            if (rslt == CCS811_OK)
            {
                /* Set mode */
                rslt = dev->write(dev->dev_id, CSS811_MEAS_MODE_ADDR, dev->settings.mode, 1);
                rslt = dev->read(dev->dev_id, CSS811_MEAS_MODE_ADDR, &chip_id, 1);
            }
            break;
        }

        /* Wait for 1 ms */
        dev->delay_ms(100);
        --try_count;
    }

    /* Chip id check failed */
    if (!try_count)
    {
        rslt = 5; //BME280_E_DEV_NOT_FOUND;
    }

    return rslt;
}

int8_t ccs811_get_data(struct ccs811_dev *dev, struct ccs811_data *data)
{
    int8_t rslt;
    bool dataReady = false;
    uint32_t tVOC = 0;
    uint32_t CO2 = 0;

    //! Debug Variables
    static uint32_t datareadycntr = 0;
    static uint32_t tryReadCntr = 0;
    //!

    /* Array to store the eco2, tvoc  */
    uint8_t alg_data[4] = {0};

    //! Debug Variables
    tryReadCntr++;

    /* Read the pressure and temperature data from the sensor */

    dataReady = isDataReady(dev);

    if ((dataReady))
    {
        rslt = dev->read(dev->dev_id, CSS811_ALG_RESULT_ADDR, alg_data, 4);

        uint8_t co2MSB = alg_data[0];
        uint8_t co2LSB = alg_data[1];
        uint8_t tvocMSB = alg_data[2];
        uint8_t tvocLSB = alg_data[3];
        CO2 = ((uint32_t)co2MSB << 8) | co2LSB;
        tVOC = ((uint32_t)tvocMSB << 8) | tvocLSB;
        data->eCO2 = CO2;
        data->VOCS = tVOC;

        //! Debug Variables
        datareadycntr++;
        /* Parse the read data from the sensor */
    }

    dev->delay_ms(1);
    return rslt;
}

bool ccs811_check_error(struct ccs811_dev *dev)
{
    uint8_t reg_data[4] = {0};
    uint8_t statusReg = 0;
    uint8_t errorId = 0;
    bool isErrorClear = true;
    static uint32_t errorCounter = 0;

    dev->read(dev->dev_id, CSS811_STATUS_ADDR, &statusReg, 1);
    if (((statusReg & 0x1) << 0) == 1)
    {
        dev->read(dev->dev_id, CSS811_ERROR_ID_ADDR, &errorId, 1);
        isErrorClear = false;
        errorCounter++;
    }

    return isErrorClear;
}
static bool isDataReady(struct ccs811_dev *dev)
{
    bool dataReady = false;
    uint8_t rslt = 0;
    uint8_t status = 0;

    rslt = dev->read(dev->dev_id, CSS811_STATUS_ADDR, &status, 1);
    if (ccs811_check_error)
    {
        dataReady = (status >> 3) & (0x1);
    }

    return dataReady;
}