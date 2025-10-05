/**
 ** Company: "-----"
 ** Author: Liamachka Feadosi
 ** Created on: 25.09.2025
 **/

/***************************Includes**********************************/
#include "LSM6DS3.h"
/*************************End Includes********************************/

/**************************Variables**********************************/
/*---------------Extern-------------------*/
extern I2C_HandleTypeDef hi2c1;

/*-------------End Extern-----------------*/

/*---------------General------------------*/

/*-------------End General----------------*/
/************************End Variables********************************/

/**************************Functions**********************************/
/** ===========================================
 **  Function name : LSM6DS3_Init()
 **  Description   : Top–level initialization for the LSM6DS3 device.
 **                  Selects device address and bus interface, then
 **                  initializes the accelerometer and gyroscope blocks.
 **  Parametrs     : None
 **  Return        : None
 ** ============================================*/
void LSM6DS3_Init(void)
{
  LSM6DS3_SET_ADRESS();
  LSM6DS3_SET_INTERFACE();
  LSM6DS3_ACCEL_Init();
  LSM6DS3_GYRO_Init();
}

/** ===========================================
 **  Function name : LSM6DS3_ACCEL_Init()
 **  Description   : Configures the accelerometer:
 **                  - Enables auto–increment for multi-byte accesses
 **                  - Enables Block Data Update (BDU)
 **                  - Sets FIFO to bypass mode
 **                  - Sets ODR to power-down, selects ±2g full scale
 **                  - Enables X/Y/Z axes
 **                  - Finally sets ODR to 104 Hz
 **  Parametrs     : None
 **  Return        : None
 ** ============================================*/
void LSM6DS3_ACCEL_Init(void)
{
  uint8_t value = 0;

  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL3_C);
  value &= ~LSM6DS3_ACC_GYRO_IF_INC_MASK;
  value |= LSM6DS3_ACC_GYRO_IF_INC_ENABLED;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL3_C, value);
  // BDU
  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL3_C);
  value &= ~LSM6DS3_ACC_GYRO_BDU_MASK;
  value |= LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL3_C, value);
  // FIFO
  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5);
  value &= ~LSM6DS3_ACC_GYRO_FIFO_MODE_MASK;
  value |= LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, value);
  // (ODR_XL = 0000)
  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL1_XL);
  value &= ~LSM6DS3_ACC_GYRO_ODR_XL_MASK;
  value |= LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, value);
  // Full scale selection 2G
  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL1_XL);
  value &= ~LSM6DS3_ACC_GYRO_FS_XL_MASK;
  value |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, value);

  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL9_XL);
  value &= ~( LSM6DS3_ACC_GYRO_XEN_XL_MASK |\
              LSM6DS3_ACC_GYRO_YEN_XL_MASK |\
              LSM6DS3_ACC_GYRO_ZEN_XL_MASK);
  value |= (  LSM6DS3_ACC_GYRO_XEN_XL_ENABLED |\
              LSM6DS3_ACC_GYRO_YEN_XL_ENABLED |\
              LSM6DS3_ACC_GYRO_ZEN_XL_ENABLED);
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL9_XL, value);

  // Sets data rate 119
  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL1_XL);
  value &= ~LSM6DS3_ACC_GYRO_ODR_XL_MASK;
  value |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, value);
}

/** ===========================================
 **  Function name : LSM6DS3_ACCEL_Read()
 **  Description   : Reads raw X/Y/Z accelerometer samples and prints
 **                  them to the console as signed 16-bit integers.
 **  Parametrs     : None
 **  Return        : None
 ** ============================================*/
void LSM6DS3_ACCEL_Read(void)
{
  int16_t buffer[3] = { 0 };
  int16_t axval, ayval, azval;

  LSM6DS3_ACCEL_GetXYZ(buffer);
  axval = buffer[0];
  ayval = buffer[1];
  azval = buffer[2];
  printf("axval = %d; ayval = %d; azval = %d\n", axval, ayval, azval);
}

/** ===========================================
 **  Function name : LSM6DS3_ACCEL_GetXYZ()
 **  Description   : Performs a burst read of the 6 accelerometer
 **                  output registers and packs the bytes into
 **                  three signed 16-bit values (X, Y, Z).
 **  Parametrs     : pData - Pointer to an array of 3 int16_t where
 **                           the read X/Y/Z samples will be stored.
 **  Return        : None
 ** ============================================*/
void LSM6DS3_ACCEL_GetXYZ(int16_t *pData)
{
  uint8_t buffer[6];
  uint8_t i = 0;

  buffer[0] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTX_L_XL);
  buffer[1] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTX_H_XL);
  buffer[2] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTY_L_XL);
  buffer[3] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTY_H_XL);
  buffer[4] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTZ_L_XL);
  buffer[5] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTZ_H_XL);
  for (i = 0; i < 3; i++)
  {
    pData[i] = ((int16_t) ((uint16_t) buffer[2 * i + 1] << 8) + buffer[2 * i]);
  }
}

/** ===========================================
 **  Function name : LSM6DS3_GYRO_Init()
 **  Description   : Configures the gyroscope:
 **                  - Enables auto–increment and BDU
 **                  - Sets FIFO to bypass mode
 **                  - Sets gyro ODR to power-down, selects 500 dps full scale
 **                  - Enables X/Y/Z axes
 **                  - Finally sets ODR (see code path) before running
 **  Parametrs     : None
 **  Return        : None
 ** ============================================*/
void LSM6DS3_GYRO_Init(void)
{
  uint8_t value = 0;

  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL3_C);
  value &= ~LSM6DS3_ACC_GYRO_IF_INC_MASK;
  value |= LSM6DS3_ACC_GYRO_IF_INC_ENABLED;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL3_C, value);
  // BDU
  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL3_C);
  value &= ~LSM6DS3_ACC_GYRO_BDU_MASK;
  value |= LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL3_C, value);
  // FIFO
  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5);
  value &= ~LSM6DS3_ACC_GYRO_FIFO_MODE_MASK;
  value |= LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, value);
  // (ODR_G = 0000)
  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL2_G);
  value &= ~LSM6DS3_ACC_GYRO_ODR_G_MASK;
  value |= LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL2_G, value);
  // Full scale selection 500 dps
  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL2_G);
  value &= ~LSM6DS3_ACC_GYRO_FS_G_500dps;
  value |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL2_G, value);
  //
  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL10_C);
  value &= ~( LSM6DS3_ACC_GYRO_XEN_G_MASK |\
              LSM6DS3_ACC_GYRO_YEN_G_MASK |\
              LSM6DS3_ACC_GYRO_ZEN_G_MASK);
  value |= (  LSM6DS3_ACC_GYRO_XEN_G_ENABLED |\
              LSM6DS3_ACC_GYRO_YEN_G_ENABLED |\
              LSM6DS3_ACC_GYRO_ZEN_G_ENABLED);
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL10_C, value);
  // Data Rate 833
  value = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_CTRL2_G);
  value &= ~LSM6DS3_ACC_GYRO_ODR_G_833Hz;
  value |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
  LSM6DS3_WirteRegister(LSM6DS3_ACC_GYRO_CTRL2_G, value);
}

/** ===========================================
 **  Function name : LSM6DS3_Gyro_Read()
 **  Description   : Reads raw X/Y/Z gyroscope samples, applies simple
 **                  per-axis offsets and saturation guards, and prints
 **                  the corrected values as signed 16-bit integers.
 **  Parametrs     : None
 **  Return        : None
 ** ============================================*/
void LSM6DS3_Gyro_Read(void)
{
  int16_t buffer[3] = { 0 };
  int16_t gxval, gyval, gzval;

  LSM6DS3_GYRO_GetXYZ(buffer);
  gxval = buffer[0] - 88;
  if (gxval < -32768) gxval = -32768;
  gyval = buffer[1] + 325;
  if (gyval > 32767) gyval = 32767;
  gzval = buffer[2] + 135;
  if (gzval > 32767) gzval = 32767;
  printf("gxval = %d; gyval = %d; gzval = %d\n", gxval, gyval, gzval);
}

/** ===========================================
 **  Function name : LSM6DS3_GYRO_GetXYZ()
 **  Description   : Performs a burst read of the 6 gyroscope
 **                  output registers and packs the bytes into
 **                  three signed 16-bit values (X, Y, Z).
 **  Parametrs     : pData - Pointer to an array of 3 int16_t where
 **                           the read X/Y/Z samples will be stored.
 **  Return        : None
 ** ============================================*/
void LSM6DS3_GYRO_GetXYZ(int16_t *pData)
{
  uint8_t buffer[6];
  uint8_t i = 0;

  buffer[0] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTX_L_G);
  buffer[1] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTX_H_G);
  buffer[2] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTY_L_G);
  buffer[3] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTY_H_G);
  buffer[4] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTZ_L_G);
  buffer[5] = LSM6DS3_ReadRegister(LSM6DS3_ACC_GYRO_OUTZ_H_G);
  for (i = 0; i < 3; i++)
  {
    pData[i] = ((int16_t) ((uint16_t) buffer[2 * i + 1] << 8) + buffer[2 * i]);
  }
}

/** ===========================================
 **  Function name : LSM6DS3_ReadRegister()
 **  Description   : Reads one 8-bit register from the LSM6DS3 over I²C.
 **                  Calls the error handler on HAL failure.
 **  Parametrs     : reg    - 8-bit register address to read.
 **  Return        : value  - read from the given register.
 ** ============================================*/
uint8_t LSM6DS3_ReadRegister(uint8_t reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;

  status = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_DEVICE_ADRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
  if (status != HAL_OK)
    LSM6DS3_Error();
  return value;
}

/** ===========================================
 **  Function name : LSM6DS3_WirteRegister()
 **  Description   : Writes one 8-bit value to an LSM6DS3 register over I²C.
 **                  Calls the error handler on HAL failure.
 **  Parametrs     : reg    - 8-bit register address to write.
 **                  value  - 8-bit value to be written to the register.
 **  Return        : None
 ** ============================================*/
void LSM6DS3_WirteRegister(uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&hi2c1, LSM6DS3_DEVICE_ADRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
  if (status != HAL_OK)
    LSM6DS3_Error();
}

/** ===========================================
 **  Function name : LSM6DS3_Error()
 **  Description   : Simple error handler for LSM6DS3 transactions.
 **                  Currently signals an error using the board LED macro.
 **  Parametrs     : None
 **  Return        : None
 ** ============================================*/
void LSM6DS3_Error(void)
{
  LSM6DS3_LED_BOARD(LSM6DS3_ON);
}
/************************End Functions********************************/
