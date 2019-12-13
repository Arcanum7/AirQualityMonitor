#include "CCS811_defs.h"

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and checks if connection is OK.
 *
 *  @param[in,out] dev : Structure instance of ccs811_dev
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error //TODO not implemented
 */
int8_t ccs811_init(struct ccs811_dev *dev);

int8_t ccs811_compansate_envoromental(float relativeHumidity, float temperature); //Todo pass struct
int8_t ccs811_get_data(struct ccs811_dev *dev, struct ccs811_data *data);
bool ccs811_check_error(struct ccs811_dev *dev);
static bool isDataReady(struct ccs811_dev *dev);

void readAlgorithmResults(void);
void configureCCS811(float humidity, float temperature);
// FlagStatus checkForError(void);
unsigned int getBaseline(void);
// FlagStatus dataAvailable(void);
// FlagStatus appValid(void);
void enableInterrupts(void);
void disableInterrupts(void);
void setDriveMode(uint8_t mode);
uint8_t readRegister(uint8_t addr);
void writeRegister(uint8_t addr, uint8_t val);
void Init_I2C_CCS811(void);
void softRest(void);
void sleep(void);
uint32_t get_Sensor_Resistance(void);
void restore_Baseline(void);
