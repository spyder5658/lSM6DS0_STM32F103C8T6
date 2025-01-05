#include "lsm6ds0.h"


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

float LSM6DS0_ReadTemperature_C() {

	  uint8_t Out_Temp_L = 0;
	  uint8_t Out_Temp_H = 0;
    uint16_t Raw_Temp=0;
    float degreeCelsius =0;

    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR,OUT_TEMP_L, I2C_MEMADD_SIZE_8BIT, &Out_Temp_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR,OUT_TEMP_H, I2C_MEMADD_SIZE_8BIT, &Out_Temp_H, 1, HAL_MAX_DELAY);
    Raw_Temp = ((Out_Temp_H<<8)| Out_Temp_L);
    degreeCelsius = (float)Raw_Temp/256;
    return (degreeCelsius);
}

float LSM6DS0_ReadGyro_X() {
    uint8_t Out_X_G_L = 0;
	  uint8_t Out_X_G_H = 0;
    int16_t Raw_X = 0;
    float Gyro_X = 0;

    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, LSM6DS0_OUT_X_G_L, I2C_MEMADD_SIZE_8BIT, &Out_X_G_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, LSM6DS0_OUT_X_G_H, I2C_MEMADD_SIZE_8BIT, &Out_X_G_H, 1, HAL_MAX_DELAY);
    Raw_X = ((Out_X_G_H<<8)| Out_X_G_L);
    Gyro_X = (float)Raw_X*70;
    return (Gyro_X);
}
float LSM6DS0_ReadGyro_Y() {
    uint8_t Out_Y_G_L = 0;
	  uint8_t Out_Y_G_H = 0;
    int16_t Raw_Y = 0;
    float Gyro_Y = 0;

    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, LSM6DS0_OUT_Y_G_L, I2C_MEMADD_SIZE_8BIT, &Out_Y_G_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, LSM6DS0_OUT_Y_G_H, I2C_MEMADD_SIZE_8BIT, &Out_Y_G_H, 1, HAL_MAX_DELAY);
    Raw_Y = ((Out_Y_G_H<<8)| Out_Y_G_L);
    Gyro_Y = (float)Raw_Y*70;
    return (Gyro_Y);
}
float LSM6DS0_ReadGyro_Z() {
    uint8_t Out_Z_G_L = 0;
	  uint8_t Out_Z_G_H = 0;
    int16_t Raw_Z = 0;
    float Gyro_Z = 0;

    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, LSM6DS0_OUT_Z_G_L, I2C_MEMADD_SIZE_8BIT, &Out_Z_G_L, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, LSM6DS0_OUT_Z_G_H, I2C_MEMADD_SIZE_8BIT, &Out_Z_G_H, 1, HAL_MAX_DELAY);
    Raw_Z = ((Out_Z_G_H<<8)| Out_Z_G_L);
    Gyro_Z = (float)Raw_Z*70;
    return (Gyro_Z);
}


float LSM6DS0_ReadAccel_X(void) {
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