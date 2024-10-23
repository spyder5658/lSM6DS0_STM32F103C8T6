/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @author         :Sarthak Chaudhary
  * @file           : main.c
  * @brief          : Main program body
  * @date           : Oct 23, 2024
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define LSM6DS0_ADDR        0xD6 // Change this if your I2C address is different D7 for read and D6 for write
#define WHO_AM_I            0x0F
#define CTRL_REG2_G         0x11
#define CTRL_REG1_XL        0x10
#define OUT_X_L_G           0x22    //22
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2A
#define OUT_Y_H_XL          0x2B
#define OUT_Z_L_XL          0x2C
#define OUT_Z_H_XL          0x2D


#define STATUS_REG          0x1E

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

int LSM6DS0_begin(void){
  uint8_t data=0;

    // Check WHO_AM_I register
  HAL_I2C_Mem_Read(&hi2c1, 0xD7, WHO_AM_I, 1, &data, 1, HAL_MAX_DELAY);
  if (data != 0x6C) {
      // Handle error
      // HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,SET);
    return 0;
  
  }
  return 1;
}
void LSM6DS0_Init(void) {
    uint8_t data=0;

    // Initialize Gyroscope
    data = 0x6C; // 416 Hz, 2000 dps
    HAL_I2C_Mem_Write(&hi2c1, LSM6DS0_ADDR, CTRL_REG2_G, 1, &data, 1, HAL_MAX_DELAY);

    // Initialize Accelerometer
    // HAL_I2C_Mem_Write(&hi2c1, LSM6DS0_ADDR, CTRL_REG1_XL, 1, &data1, 1, HAL_MAX_DELAY);
    data = 0x60; // 416 Hz, +/- 2g
    HAL_I2C_Mem_Write(&hi2c1, LSM6DS0_ADDR, CTRL_REG1_XL, 1, &data, 1, HAL_MAX_DELAY);
}
int check_Gyro_Init(){
  uint8_t data=0;

    // Check WHO_AM_I register
  HAL_I2C_Mem_Read(&hi2c1, 0xD7, CTRL_REG2_G, 1, &data, 1, HAL_MAX_DELAY);
  if(data !=0x6C){
    return 0;
  }else{
    return 1;
  }

}
int check_XL_Init(){
  uint8_t data=0;

    // Check WHO_AM_I register
  HAL_I2C_Mem_Read(&hi2c1, 0xD7, CTRL_REG1_XL, 1, &data, 1, HAL_MAX_DELAY);
  if(data !=0x60){
    return 0;
  }else{
    return 1;
  }

}


void LSM6DS0_ReadGyro(int16_t *gyro_data) {
    uint8_t data[6];

    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, OUT_X_L_G, 1, data, 6, HAL_MAX_DELAY);

    gyro_data[0] = (int16_t)(data[0] | (data[1] << 8));
    gyro_data[1] = (int16_t)(data[2] | (data[3] << 8));
    gyro_data[2] = (int16_t)(data[4] | (data[5] << 8));
}
float LSM6DS0_ReadAccel_X(void) {
    // uint8_t data[6];
    uint8_t Out_X_XL_L = 0;
	  uint8_t Out_X_XL_H = 0;
    int16_t Raw_X = 0;
    float Acceleration_X = 0;

    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, OUT_X_L_XL, I2C_MEMADD_SIZE_8BIT, &Out_X_XL_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, OUT_X_H_XL, I2C_MEMADD_SIZE_8BIT, &Out_X_XL_H, 1, HAL_MAX_DELAY);

    Raw_X = ((Out_X_XL_H << 8) | Out_X_XL_L);
    Acceleration_X = (float)Raw_X*0.061f;
    return (Acceleration_X);
}
float LSM6DS0_ReadAccel_Y(void) {
    // uint8_t data[6];
    uint8_t Out_Y_XL_L = 0;
	  uint8_t Out_Y_XL_H = 0;
    int16_t Raw_Y = 0;
    float Acceleration_Y = 0;

    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, OUT_Y_L_XL, I2C_MEMADD_SIZE_8BIT, &Out_Y_XL_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, OUT_Y_H_XL, I2C_MEMADD_SIZE_8BIT, &Out_Y_XL_H, 1, HAL_MAX_DELAY);

    Raw_Y = ((Out_Y_XL_H << 8) | Out_Y_XL_L);
    Acceleration_Y = (float)Raw_Y*0.061f;
    return (Acceleration_Y);
}
float LSM6DS0_ReadAccel_Z(void) {
    // uint8_t data[6];
    uint8_t Out_Z_XL_L = 0;
	  uint8_t Out_Z_XL_H = 0;
    int16_t Raw_Z = 0;
    float Acceleration_Z = 0;

    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, OUT_Z_L_XL, I2C_MEMADD_SIZE_8BIT, &Out_Z_XL_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, OUT_Z_H_XL, I2C_MEMADD_SIZE_8BIT, &Out_Z_XL_H, 1, HAL_MAX_DELAY);

    Raw_Z = ((Out_Z_XL_H << 8) | Out_Z_XL_L);
    Acceleration_Z = (float)Raw_Z*0.061f;
    return (Acceleration_Z);
}

 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */
  if(LSM6DS0_begin()==0){
        printf("LSM_not_found\r\n");
  }else{
        printf("LSM_found..\r\n");
        printf("initializing six axis sensor....\r\n");
   }
  LSM6DS0_Init();
  if (check_Gyro_Init()!=1){
    printf("gyro_init_failed\r\n");
  }else{
    printf("gyro_init_successful\r\n");
  }
  if(check_XL_Init()!=1){
    printf("XL_init_failed\r\n");
  }
  else{
    printf("XL_init_successfull\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  

    int16_t gyro_data[3];
    float accel_data[3];
    // uint8_t status;
    // HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, STATUS_REG, 1, &status, 6, HAL_MAX_DELAY);
  while (1)
  {
    // if(LSM6DS0_begin()==0){
    //     printf("LSM_not_found\r\n");
    // }else{
    //     printf("LSM_found..\r\n");
    //     printf("initializing six axis sensor....\r\n");
    // }
  
    // printf("hellow\r\n");
      accel_data[0]=LSM6DS0_ReadAccel_X();
      accel_data[1]=LSM6DS0_ReadAccel_Y();
      accel_data[2]=LSM6DS0_ReadAccel_Z();


      // accel_data = (int)accel_data;
      // printf("Gyro: X=%d\r\n",gyro_data);
      printf("Accel: X=%d Y=%d Z=%d\r\n",(int)accel_data[0],(int)accel_data[1],(int)accel_data[2]);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
