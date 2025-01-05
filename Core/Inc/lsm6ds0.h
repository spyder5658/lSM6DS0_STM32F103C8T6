#ifndef LSM6DS0_H
#define LSM6DS0_H

#include "i2c.h"  // Include the necessary HAL libraries
#include "usart.h"
#include <math.h>
#include <stdint.h>

#define LSM6DS0_ADDR        0xD6 // Change this if your I2C address is different D7 for read and D6 for write
#define WHO_AM_I            0x0F
#define CTRL_REG2_G         0x11
#define CTRL_REG1_XL        0x10


/* Accelerometer*/
#define OUT_X_L_G           0x22    //22
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2A
#define OUT_Y_H_XL          0x2B
#define OUT_Z_L_XL          0x2C
#define OUT_Z_H_XL          0x2D

/* Gyroscope*/
#define LSM6DS0_OUT_X_G_L							0x22	 
#define LSM6DS0_OUT_X_G_H							0x23	 
#define LSM6DS0_OUT_Y_G_L							0x24		 
#define LSM6DS0_OUT_Y_G_H							0x25		 
#define LSM6DS0_OUT_Z_G_L							0x26		
#define LSM6DS0_OUT_Z_G_H							0x27	

/* Temperature*/
#define OUT_TEMP_L          0X20
#define OUT_TEMP_H          0X21


/* Function Prototypes */
int LSM6DS0_begin(void);
void LSM6DS0_Init(void);
int check_Gyro_Init(void);
int check_XL_Init(void);
float LSM6DS0_ReadTemperature_C(void);
float LSM6DS0_ReadGyro_X(void);
float LSM6DS0_ReadGyro_Y(void);
float LSM6DS0_ReadGyro_Z(void);
float LSM6DS0_ReadAccel_X(void);
float LSM6DS0_ReadAccel_Y(void);
float LSM6DS0_ReadAccel_Z(void);

#endif /* LSM6DS0_H */
