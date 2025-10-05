/**
 ** Company: "-----"
 ** Author: Liamachka Feadosi
 ** Created on: 25.09.2025
 **/

#ifndef INC_LSM6DS3_H_
#define INC_LSM6DS3_H_

/**************************INCLUDES***********************************/
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdio.h>
/*************************END INCLUDES********************************/

/*************************ENUMERATIONS********************************/

/***********************END ENUMERATIONS******************************/

/***************************STRUCTS***********************************/

/*************************END STRUCTS*********************************/

/**************************DEFINES************************************/
#define LSM6DS3_SELECT_SPI_OR_I2C           1U    // SPI - 0U, I2C - 1U
#define LSM6DS3_SELECT_ADRESS               0U    // (SAD=0) - 0U, (SAD=1) = 1U
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_CTRL1_XL           0x10
#define LSM6DS3_ACC_GYRO_CTRL2_G            0x11
#define LSM6DS3_ACC_GYRO_CTRL3_C            0x12
#define LSM6DS3_ACC_GYRO_FIFO_CTRL5         0x0A
#define LSM6DS3_ACC_GYRO_CTRL9_XL           0x18
#define LSM6DS3_ACC_GYRO_CTRL10_C           0x19
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_IF_INC_DISABLED    0x00
#define LSM6DS3_ACC_GYRO_IF_INC_ENABLED     0x04
#define LSM6DS3_ACC_GYRO_IF_INC_MASK        0x04
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_BDU_CONTINUOS      0x00
#define LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE   0x40
#define LSM6DS3_ACC_GYRO_BDU_MASK           0x40
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS   0x00
#define LSM6DS3_ACC_GYRO_FIFO_MODE_FIFO     0x01
#define LSM6DS3_ACC_GYRO_FIFO_MODE_STREAM   0x02
#define LSM6DS3_ACC_GYRO_FIFO_MODE_STF      0x03
#define LSM6DS3_ACC_GYRO_FIFO_MODE_BTS      0x04
#define LSM6DS3_ACC_GYRO_FIFO_MODE_DN_STRM  0x05
#define LSM6DS3_ACC_GYRO_FIFO_MODE_DN_STRM2 0x06
#define LSM6DS3_ACC_GYRO_FIFO_MODE_BTF      0x07
#define LSM6DS3_ACC_GYRO_FIFO_MODE_MASK     0x07
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN  0x00
#define LSM6DS3_ACC_GYRO_ODR_XL_13Hz        0x10
#define LSM6DS3_ACC_GYRO_ODR_XL_26Hz        0x20
#define LSM6DS3_ACC_GYRO_ODR_XL_52Hz        0x30
#define LSM6DS3_ACC_GYRO_ODR_XL_104Hz       0x40
#define LSM6DS3_ACC_GYRO_ODR_XL_208Hz       0x50
#define LSM6DS3_ACC_GYRO_ODR_XL_416Hz       0x60
#define LSM6DS3_ACC_GYRO_ODR_XL_833Hz       0x70
#define LSM6DS3_ACC_GYRO_ODR_XL_1660Hz      0x80
#define LSM6DS3_ACC_GYRO_ODR_XL_3330Hz      0x90
#define LSM6DS3_ACC_GYRO_ODR_XL_6660Hz      0xA0
#define LSM6DS3_ACC_GYRO_ODR_XL_13330Hz     0xB0
#define LSM6DS3_ACC_GYRO_ODR_XL_MASK        0xF0
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN   0x00
#define LSM6DS3_ACC_GYRO_ODR_G_13Hz         0x10
#define LSM6DS3_ACC_GYRO_ODR_G_26Hz         0x20
#define LSM6DS3_ACC_GYRO_ODR_G_52Hz         0x30
#define LSM6DS3_ACC_GYRO_ODR_G_104Hz        0x40
#define LSM6DS3_ACC_GYRO_ODR_G_208Hz        0x50
#define LSM6DS3_ACC_GYRO_ODR_G_416Hz        0x60
#define LSM6DS3_ACC_GYRO_ODR_G_833Hz        0x70
#define LSM6DS3_ACC_GYRO_ODR_G_1660Hz       0x80
#define LSM6DS3_ACC_GYRO_ODR_G_MASK         0xF0
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_FS_XL_2g           0x00
#define LSM6DS3_ACC_GYRO_FS_XL_16g          0x04
#define LSM6DS3_ACC_GYRO_FS_XL_4g           0x08
#define LSM6DS3_ACC_GYRO_FS_XL_8g           0x0C
#define LSM6DS3_ACC_GYRO_FS_XL_MASK         0x0C
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_FS_G_245dps        0x00
#define LSM6DS3_ACC_GYRO_FS_G_500dps        0x04
#define LSM6DS3_ACC_GYRO_FS_G_1000dps       0x08
#define LSM6DS3_ACC_GYRO_FS_G_2000dps       0x0C
#define LSM6DS3_ACC_GYRO_FS_G_MASK          0x0C
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_XEN_XL_MASK        0x08
#define LSM6DS3_ACC_GYRO_YEN_XL_MASK        0x10
#define LSM6DS3_ACC_GYRO_ZEN_XL_MASK        0x20
#define LSM6DS3_ACC_GYRO_XEN_XL_ENABLED     0x08
#define LSM6DS3_ACC_GYRO_YEN_XL_ENABLED     0x10
#define LSM6DS3_ACC_GYRO_ZEN_XL_ENABLED     0x20
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_XEN_G_DISABLED     0x00
#define LSM6DS3_ACC_GYRO_XEN_G_ENABLED      0x08
#define LSM6DS3_ACC_GYRO_YEN_G_DISABLED     0x00
#define LSM6DS3_ACC_GYRO_YEN_G_ENABLED      0x10
#define LSM6DS3_ACC_GYRO_ZEN_G_DISABLED     0x00
#define LSM6DS3_ACC_GYRO_ZEN_G_ENABLED      0x20
#define LSM6DS3_ACC_GYRO_XEN_G_MASK         0x08
#define LSM6DS3_ACC_GYRO_YEN_G_MASK         0x10
#define LSM6DS3_ACC_GYRO_ZEN_G_MASK         0x20
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_OUTX_L_XL          0X28
#define LSM6DS3_ACC_GYRO_OUTX_H_XL          0X29
#define LSM6DS3_ACC_GYRO_OUTY_L_XL          0X2A
#define LSM6DS3_ACC_GYRO_OUTY_H_XL          0X2B
#define LSM6DS3_ACC_GYRO_OUTZ_L_XL          0X2C
#define LSM6DS3_ACC_GYRO_OUTZ_H_XL          0X2D
//------------------------------------------------
#define LSM6DS3_ACC_GYRO_OUTX_L_G           0X22
#define LSM6DS3_ACC_GYRO_OUTX_H_G           0X23
#define LSM6DS3_ACC_GYRO_OUTY_L_G           0X24
#define LSM6DS3_ACC_GYRO_OUTY_H_G           0X25
#define LSM6DS3_ACC_GYRO_OUTZ_L_G           0X26
#define LSM6DS3_ACC_GYRO_OUTZ_H_G           0X27
//------------------------------------------------
#if LSM6DS3_SELECT_SPI_OR_I2C
  #define LSM6DS3_SET_INTERFACE() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)    // I2C
#else
  #define LSM6DS3_SET_INTERFACE() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)  // SPI
#endif
//------------------------------------------------
#if LSM6DS3_SELECT_ADRESS
  #define LSM6DS3_SET_ADRESS()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)     // Set adress 0xD6
  #define LSM6DS3_DEVICE_ADRESS   0xD6
#else
  #define LSM6DS3_SET_ADRESS()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)   // Set adress 0xD4
  #define LSM6DS3_DEVICE_ADRESS   0xD4
#endif
//------------------------------------------------
#define LSM6DS3_ON                GPIO_PIN_SET
#define LSM6DS3_OFF               GPIO_PIN_RESET
#define LSM6DS3_LED_BOARD(state)  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, state)
/************************END DEFINES**********************************/

/*********************PROTOTYPE FUNCTIONS*****************************/
void LSM6DS3_Init(void);
void LSM6DS3_ACCEL_Init(void);
void LSM6DS3_ACCEL_Read(void);
void LSM6DS3_ACCEL_GetXYZ(int16_t *pData);
void LSM6DS3_GYRO_Init(void);
void LSM6DS3_Gyro_Read(void);
void LSM6DS3_GYRO_GetXYZ(int16_t *pData);
uint8_t LSM6DS3_ReadRegister(uint8_t reg);
void LSM6DS3_WirteRegister(uint8_t reg, uint8_t value);
void LSM6DS3_Error(void);
/*******************END PROTOTYPE FUNCTIONS***************************/

#endif /* INC_LSM6DS3_H_ */
