#define BMP180_ADDRESS (0x77 << 1) // BMP180 I2C address
#define TEMP_PRESSURE_REG 0xF6     // Register for raw data
#define CONTROL_REG 0xF4           // Control register
#define READ_TEMP_CMD 0x2E         // Command to read temperature
#define READ_PRESS_CMD 0x34        // Command to read pressure
#define atmPress 101325 // Pa

// BMP180 calibration data
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;


void readBMP180CalibrationData(void);
int32_t readRawTemperature(void);
int32_t readRawPressure(void);
float calculateTemperature(int32_t UT);
float calculatePressure(int32_t UP);
int32_t BMP180_GetAlt(int32_t UP);


// Read BMP180 calibration data
void readBMP180CalibrationData(void)
{
  uint8_t calib_data[22];
  HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, 0xAA, I2C_MEMADD_SIZE_8BIT, calib_data, 22, HAL_MAX_DELAY);

  AC1 = (calib_data[0] << 8) | calib_data[1];
  AC2 = (calib_data[2] << 8) | calib_data[3];
  AC3 = (calib_data[4] << 8) | calib_data[5];
  AC4 = (calib_data[6] << 8) | calib_data[7];
  AC5 = (calib_data[8] << 8) | calib_data[9];
  AC6 = (calib_data[10] << 8) | calib_data[11];
  B1 = (calib_data[12] << 8) | calib_data[13];
  B2 = (calib_data[14] << 8) | calib_data[15];
  MB = (calib_data[16] << 8) | calib_data[17];
  MC = (calib_data[18] << 8) | calib_data[19];
  MD = (calib_data[20] << 8) | calib_data[21];
}

// Read raw temperature
int32_t readRawTemperature(void)
{
  uint8_t temp_cmd = READ_TEMP_CMD;
  uint8_t data[2];
  HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDRESS, CONTROL_REG, I2C_MEMADD_SIZE_8BIT, &temp_cmd, 1, HAL_MAX_DELAY);
  HAL_Delay(5); // Wait for conversion
  HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, TEMP_PRESSURE_REG, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
  return (data[0] << 8) | data[1];
}

// Read raw pressure
int32_t readRawPressure(void)
{
  uint8_t press_cmd = READ_PRESS_CMD + (3 << 6);
  HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDRESS, CONTROL_REG, I2C_MEMADD_SIZE_8BIT, &press_cmd, 1, HAL_MAX_DELAY);
  HAL_Delay(26); // Wait for conversion
  uint8_t data[3];
  HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, TEMP_PRESSURE_REG, I2C_MEMADD_SIZE_8BIT, data, 3, HAL_MAX_DELAY);
  return ((data[0] << 16) | (data[1] << 8) | data[2]) >> (8 - 3);
}

float calculateTemperature(int32_t UT)
{
  int32_t X1 = ((UT - AC6) * AC5) >> 15;
  int32_t X2 = (MC << 11) / (X1 + MD);
  int32_t B5 = X1 + X2;
  return ((B5 + 8) >> 4) / 10.0;
}

// Calculate true pressure in Pascals
float calculatePressure(int32_t UP)
{
  int32_t X1, X2, X3, B3, B5, B6, p;
  uint32_t B4, B7;

  // Assuming previous calculation of B5 in calculateTemperature()
  int32_t UT = readRawTemperature();
  X1 = ((UT - AC6) * AC5) >> 15;
  X2 = (MC << 11) / (X1 + MD);
  B5 = X1 + X2;

  B6 = B5 - 4000;
  X1 = (B2 * (B6 * B6 >> 12)) >> 11;
  X2 = AC2 * B6 >> 11;
  X3 = X1 + X2;
  B3 = (((AC1 * 4 + X3) << 3) + 2) >> 2;
  X1 = AC3 * B6 >> 13;
  X2 = (B1 * (B6 * B6 >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = AC4 * (uint32_t)(X3 + 32768) >> 15;
  B7 = ((uint32_t)UP - B3) * (50000 >> 3);

  if (B7 < 0x80000000)
    p = (B7 * 2) / B4;
  else
    p = (B7 / B4) * 2;

  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  return p + ((X1 + X2 + 3791) >> 4);
}


int32_t BMP180_GetAlt(int32_t UP)
{
  return 44330 * (1 - pow(calculatePressure(UP) / atmPress, 0.19029495718));
}