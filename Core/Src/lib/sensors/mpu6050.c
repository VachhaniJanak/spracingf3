#include "../Core/Inc/main.h"
#include "stm32f3xx_hal.h"

// #define MPU6050_ADDR 0x68
#define MPU6050_ADDR (0x68 << 1)
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_DATA_READY_BIT 0x01 // Data ready interrupt bit

void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);

void MPU6050_Init(void)
{
  uint8_t Data;
  HAL_Delay(100);
  // Wake up the MPU6050
  // power management register 0X6B we should write all 0's to wake the sensor up
  Data = 0;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &Data, 1, 1000);

  // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
  Data = 0x07;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &Data, 1, 1000);

  // Set accelerometer configuration in ACCEL_CONFIG Register
  Data = 0x00; // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &Data, 1, 1000);

  // Set Gyroscopic configuration in GYRO_CONFIG Register
  Data = 0x00; // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 ̐/s
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &Data, 1, 1000);
}

void MPU6050_Read_Accel(int *acc_xyz)
{
  uint8_t Rec_Data[6];
  // int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;

  // Read 6 bytes from ACCEL_XOUT_H register
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, Rec_Data, 6, 1000);

  // Combine high and low bytes
  // Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  // Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  // Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

  // Convert to G values (divide by sensitivity scale factor)
  acc_xyz[0] = ((Rec_Data[0] << 8 | Rec_Data[1]) / 16384.0) * 100;
  acc_xyz[1] = ((Rec_Data[2] << 8 | Rec_Data[3]) / 16384.0) * 100;
  acc_xyz[2] = ((Rec_Data[4] << 8 | Rec_Data[5]) / 16384.0) * 100;

  // // Output final values
  // char msg[128];
  // snprintf(msg, sizeof(msg), "Acc: X=%d.%02d Y=%d.%02d Z=%d.%02d\r\n", Ax / 100, abs(Ax % 100), Ay / 100, abs(Ay % 100), Az / 100, abs(Az % 100));
  // HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
}

void MPU6050_Read_Gyro(int *gyro_xyz)
{
  uint8_t Rec_Data[6];
  // int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

  // Read 6 bytes from GYRO_XOUT_H register
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT, Rec_Data, 6, 1000);

  // Combine high and low bytes
  // Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  // Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  // Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

  // Convert to degrees/second
  gyro_xyz[0] = ((Rec_Data[0] << 8 | Rec_Data[1]) / 131.0) * 100;
  gyro_xyz[1] = ((Rec_Data[2] << 8 | Rec_Data[3]) / 131.0) * 100;
  gyro_xyz[2] = ((Rec_Data[4] << 8 | Rec_Data[5]) / 131.0) * 100;

  // Output final values
  // char msg[128];
  // snprintf(msg, sizeof(msg), "Gyro: X=%d.%02d Y=%d.%02d Z=%d.%02d\r\n", Gx / 100, abs(Gx % 100), Gy / 100, abs(Gy % 100), Gz / 100, abs(Gz % 100));
  // HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
}