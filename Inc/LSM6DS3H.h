

#include "stdint.h"
#include "stm32l4xx_hal.h"

/**
  * @brief  LSM Status structures definition
  */
	
typedef enum
{
  LSM_OK       = 0x00,
  LSM_ERROR    = 0x01,
  LSM_BUSY     = 0x02,
  LSM_TIMEOUT  = 0x03
} LSM_StatusTypeDef;


/** 
  * @brief  Axix Structure definition  
  */ 
typedef struct
{
	
  uint8_t  OUTX_L;
	uint8_t  OUTX_H;
	
	uint8_t  OUTY_L;
	uint8_t  OUTY_H;
	
	uint8_t  OUTZ_L;
	uint8_t  OUTZ_H;

}Axis_TypeDef;


/** 
  * @brief  Temperature Structure definition  
  */ 
typedef struct
{
	
  uint8_t  OUT_L;
	uint8_t  OUT_H;
	
}Temperature_TypeDef;



/**
  * @brief  LSM function declaration
  */

LSM_StatusTypeDef LSM_Who_am_I(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,uint8_t *pData);

LSM_StatusTypeDef LSM_Init(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);

LSM_StatusTypeDef LSM_ReadTemp(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData);

LSM_StatusTypeDef LSM_ReadAcc(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData);

LSM_StatusTypeDef LSM_ReadGyro(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData);

LSM_StatusTypeDef LSM_REG_WRITE(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t Reg, uint8_t Value);

LSM_StatusTypeDef LSM_REG_READ_MULTIPLE(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t Reg, uint8_t size, uint8_t *pData);


#define HIGH_ADDRESS        ((uint16_t)0x006B)  // 0b1101011x
#define LOW_ADDRESS         ((uint16_t)0x006A)  // 0b1101010x

//#define HIGH_ADDRESS_READ   ((uint16_t) (HIGH_ADDRESS <<1) | 1)  // 0b11010111
//#define HIGH_ADDRESS_WRITE  ((uint16_t) (HIGH_ADDRESS <<1) | 0)  // 0b11010110
//#define LOW_ADDRESS_READ    ((uint16_t) (LOW_ADDRESS <<1)  | 1)  // 0b11010101
//#define LOW_ADDRESS_WRITE   ((uint16_t) (LOW_ADDRESS <<1)  | 0)  // 0b11010100

#define FUNC_CFG_ACCESS     ((uint8_t)0x01)

#define FIFO_CTRL1          ((uint8_t)0x06)
#define FIFO_CTRL2          ((uint8_t)0x07)
#define FIFO_CTRL3          ((uint8_t)0x08)
#define FIFO_CTRL4          ((uint8_t)0x09)
#define FIFO_CTRL5          ((uint8_t)0x0A)
#define ORIENT_CFG_G        ((uint8_t)0x0B)

#define INT1_CTRL           ((uint8_t)0x0D)
#define INT2_CTRL           ((uint8_t)0x0E)
#define WHO_AM_I            ((uint8_t)0x0F)
#define CTRL1_XL            ((uint8_t)0x10)
#define CTRL2_G             ((uint8_t)0x11)
#define CTRL3_C             ((uint8_t)0x12)
#define CTRL4_C             ((uint8_t)0x13)
#define CTRL5_C             ((uint8_t)0x14)
#define CTRL6_C             ((uint8_t)0x15)
#define CTRL7_G             ((uint8_t)0x16)
#define CTRL8_XL            ((uint8_t)0x17)
#define CTRL9_XL            ((uint8_t)0x18)
#define CTRL10_C            ((uint8_t)0x19)

#define WAKE_UP_SRC         ((uint8_t)0x1B)
#define TAP_SRC             ((uint8_t)0x1C)
#define D6D_SRC             ((uint8_t)0x1D)
#define STATUS_REG          ((uint8_t)0x1E)

#define OUT_TEMP_L          ((uint8_t)0x20)
#define OUT_TEMP_H          ((uint8_t)0x21)
#define OUTX_L_G            ((uint8_t)0x22)
#define OUTX_H_G            ((uint8_t)0x23)
#define OUTY_L_G            ((uint8_t)0x24)
#define OUTY_H_G            ((uint8_t)0x25)
#define OUTZ_L_G            ((uint8_t)0x26)
#define OUTZ_H_G            ((uint8_t)0x27)
#define OUTX_L_XL           ((uint8_t)0x28)
#define OUTX_H_XL           ((uint8_t)0x29)
#define OUTY_L_XL           ((uint8_t)0x2A)
#define OUTY_H_XL           ((uint8_t)0x2B)
#define OUTZ_L_XL           ((uint8_t)0x2C)
#define OUTZ_H_XL           ((uint8_t)0x2D)

#define FIFO_STATUS1        ((uint8_t)0x3A)
#define FIFO_STATUS2        ((uint8_t)0x3B)
#define FIFO_STATUS3        ((uint8_t)0x3C)
#define FIFO_STATUS4        ((uint8_t)0x3D)
#define FIFO_DATA_OUT_L     ((uint8_t)0x3E)
#define FIFO_DATA_OUT_H     ((uint8_t)0x3F)
#define TIMESTAMP0_REG      ((uint8_t)0x40)
#define TIMESTAMP1_REG      ((uint8_t)0x41)
#define TIMESTAMP2_REG      ((uint8_t)0x42)

#define STEP_TIMESTAMP_L    ((uint8_t)0x49)
#define STEP_TIMESTAMP_H    ((uint8_t)0x4A)
#define STEP_COUNTER_L      ((uint8_t)0x4B)
#define STEP_COUNTER_H      ((uint8_t)0x4C)

#define FUNC_SRC            ((uint8_t)0x53)

#define TAP_CFG             ((uint8_t)0x58)
#define TAP_THS_6D          ((uint8_t)0x59)
#define INT_DUR2            ((uint8_t)0x5A)
#define WAKE_UP_THS         ((uint8_t)0x5B)
#define WAKE_UP_DUR         ((uint8_t)0x5C)
#define FREE_FALL           ((uint8_t)0x5D)
#define MD1_CFG             ((uint8_t)0x5E)
#define MD2_CFG             ((uint8_t)0x5F)
