/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bme280.h"
#include "bme280_defs.h"
#include "CCS811.h"
#include "CCS811_defs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t BME280_TestBuffer[10] = {0};
uint8_t CCS811_TestBuffer[10] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int8_t user_bme280_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_ccs811_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t user_bme280_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_ccs811_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t BME280_get_data(struct bme280_dev *dev, struct bme280_data *data);
void prepare_uart_packet(struct ccs811_data *ccs811_samples, struct bme280_data *bme280_samples, uint8_t dataElements, uint8_t *packet);
void user_delay_ms(uint32_t period);
void uint32_To_Buffer(uint32_t data, uint8_t *buffer, uint8_t index);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t I2C_REG = 0xD0;

  int bme_data[10] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  uint8_t bmePacket[PAKCET_SIZE_BYTES] = {0};

  struct bme280_dev bme280;
  struct bme280_data bme280Samples;

  volatile int8_t rslt = BME280_OK;

  bme280.dev_id = BME280_I2C_ADDR_PRIM;
  bme280.intf = BME280_I2C_INTF;
  bme280.write = user_bme280_write;
  bme280.read = user_bme280_read;
  bme280.delay_ms = user_delay_ms;

  rslt = bme280_init(&bme280);
  if (rslt != BME280_OK)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
    while (1)
    {

      /* code */
    }
  }

  rslt = CCS811_OK;
  struct ccs811_dev ccs811;
  struct ccs811_data ccs811Samples;
  ccs811.dev_id = CCS811_ADDR;
  ccs811.read = user_ccs811_read;
  ccs811.write = user_ccs811_write;
  ccs811.delay_ms = user_delay_ms;
  ccs811.settings.mode = CSS811_MODE1;

  rslt = ccs811_init(&ccs811);

  if (rslt != CCS811_OK)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
    while (1)
    {

      /* code */
    }
  }

  uint32_t start, end, diff = 0;
  float ms = 0.0f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_Delay(100);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    start = SysTick->VAL;

    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
    BME280_get_data(&bme280, &bme280Samples);
    ccs811_get_data(&ccs811, &ccs811Samples);

    //Todo yra problema su i2c??? kazkodel nedraugauja vienas su kitu, quick fix butu perkeist kanala i2c jei noresim naudoti LCD monitoriuka
    prepare_uart_packet(&ccs811Samples, &bme280Samples, PACKET_ELEMENTS_UINT32, &bmePacket);

    HAL_UART_Transmit(&huart1, bmePacket, sizeof(bmePacket), 100);

    HAL_Delay(1000);

    end = SysTick->VAL;
    diff = end - start;
    ms = (float)((diff / (SystemCoreClock * 1.0f)) * 1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//Todo makefile optimizes my shit :(
//Todo is this decoupling good option?

int8_t user_bme280_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{

  HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(id << 1), reg_addr, 1, (uint8_t *)data, len, 100);
  return 0;
}
int8_t user_bme280_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  // int8_t *buf;

  // buf = malloc(len + 1);
  // buf[0] = reg_addr;
  // memcpy(buf + 1, data, len);
  if (HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(id << 1), reg_addr, 1, (uint8_t *)data, len, 1000) != HAL_OK)
    return BME280_E_COMM_FAIL;
  // free(buf);
  return BME280_OK;
}

int8_t user_ccs811_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{

  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(id << 1), reg_addr, 1, (uint8_t *)data, len, 100);
  return 0;
}
int8_t user_ccs811_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  // int8_t *buf;

  // buf = malloc(len + 1);
  // buf[0] = reg_addr;
  // memcpy(buf + 1, data, len);

  if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t *)(id << 1), reg_addr, 1, (uint8_t *)&data, len, 1000) != HAL_OK)
    return CCS811_COMM_FAIL;
  // free(buf);
  return CCS811_OK;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}

int8_t BME280_get_data(struct bme280_dev *dev, struct bme280_data *data)
{
  volatile int8_t rslt;
  uint8_t settings_sel;
  struct bme280_data comp_data;

  float temp, press, hum;

  /* Recommended mode of operation: Indoor navigation */
  dev->settings.osr_h = BME280_OVERSAMPLING_1X;
  dev->settings.osr_p = BME280_OVERSAMPLING_1X;
  dev->settings.osr_t = BME280_OVERSAMPLING_1X;
  dev->settings.filter = BME280_FILTER_COEFF_OFF;

  settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

  rslt = bme280_set_sensor_settings(settings_sel, dev);
  if (rslt != BME280_OK)
  {
    // fprintf(stderr, "Failed to set sensor settings (code %+d).", rslt);
    // return rslt;
  }

  /* Get  sensor data */

  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
  if (rslt != BME280_OK)
  {
    // fprintf(stderr, "Failed to set sensor mode (code %+d).", rslt);
  }
  /* Wait for the measurement to complete and print data @25Hz */
  dev->delay_ms(1);
  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
  if (rslt != BME280_OK)
  {
    // fprintf(stderr, "Failed to get sensor data (code %+d).", rslt);
  }
  temp = comp_data.temperature; // 0.01f * comp_data.temperature;
  press = comp_data.pressure;   // 0.0001f * comp_data.pressure;
  hum = comp_data.humidity;     // 1.0f / 1024.0f * comp_data.humidity;

  data->temperature = comp_data.temperature;
  data->pressure = comp_data.pressure;
  data->humidity = comp_data.humidity;

  return rslt;
}

void prepare_uart_packet(struct ccs811_data *ccs811_samples, struct bme280_data *bme280_samples, uint8_t dataElements, uint8_t *packet)
{
  static uint32_t packetIndex = 0;
  packetIndex++;
  packet[0] = 0xAA;
  uint32_To_Buffer(bme280_samples->temperature, packet, 1);
  uint32_To_Buffer(bme280_samples->humidity, packet, 5);
  uint32_To_Buffer(bme280_samples->pressure, packet, 9);
  uint32_To_Buffer(ccs811_samples->eCO2, packet, 13);
  uint32_To_Buffer(ccs811_samples->VOCS, packet, 17);
  uint32_To_Buffer(packetIndex, packet, 21);
  packet[25] = 0xFF;
}

void uint32_To_Buffer(uint32_t data, uint8_t *buffer, uint8_t index)
{
  buffer[index + 0] = data >> 24;
  buffer[index + 1] = data >> 16;
  buffer[index + 2] = data >> 8;
  buffer[index + 3] = data;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
