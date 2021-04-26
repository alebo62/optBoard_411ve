#include "gpio.h"
#include "stm32f4xx_hal_i2c.h"

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;

I2C_HandleTypeDef hi2c;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);
void MPU6050_GetAllData(int16_t *Data);

void I2Cx_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  __HAL_RCC_I2C1_CLK_ENABLE();

   __HAL_RCC_GPIOB_CLK_ENABLE();

  /* I2Cx SD1 & SCK pin configuration */
  GPIO_InitStructure.Pin = GPIO_PIN_7   | GPIO_PIN_8  ;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;
  
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Force the I2C peripheral clock reset */
  __HAL_RCC_I2C1_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  __HAL_RCC_I2C1_RELEASE_RESET();

  /* Enable and set I2Cx Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

  /* Enable and set I2Cx Interrupt to the lowest priority */
//  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0x0F, 0);
//  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn); 
}

void I2Cx_Init(void)
{
  if(HAL_I2C_GetState(&hi2c) == HAL_I2C_STATE_RESET)
  {
    hi2c.Instance = I2C1;
    hi2c.Init.OwnAddress1 =  0;
    hi2c.Init.ClockSpeed = 100000;
    hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c.Init.OwnAddress2 = 0x00;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;	
		
    /* Init the I2C */
    I2Cx_MspInit(&hi2c);
    HAL_I2C_Init(&hi2c);
		MPU6050_Init(&hi2c);
		__HAL_I2C_DISABLE_IT(&hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
  }
}

void I2C_WriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer, uint8_t TXBUFFERSIZE) {
    while(HAL_I2C_Master_Transmit(&hi2c, (uint16_t)I2C_ADDRESS<<1, (uint8_t*)aTxBuffer, (uint16_t)TXBUFFERSIZE, (uint32_t)1000)!= HAL_OK){
        if (HAL_I2C_GetError(&hi2c) != HAL_I2C_ERROR_AF){
            //_Error_Handler(__FILE__, aTxBuffer[0]);
        }
 
    }
 
      while (HAL_I2C_GetState(&hi2c) != HAL_I2C_STATE_READY){}
}

void I2C_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE){
 
    I2C_WriteBuffer(I2C_ADDRESS, &RegAddr, 1);
 
    while(HAL_I2C_Master_Receive(&hi2c, (uint16_t)I2C_ADDRESS<<1, aRxBuffer, (uint16_t)RXBUFFERSIZE, (uint32_t)1000) != HAL_OK){
        if (HAL_I2C_GetError(&hi2c) != HAL_I2C_ERROR_AF){
            //_Error_Handler(__FILE__, __LINE__);
        }
    }
 
    while (HAL_I2C_GetState(&hi2c) != HAL_I2C_STATE_READY){}
}

uint8_t check;
uint8_t Data;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    

    // check device ID WHO_AM_I
	//I2C_ReadBuffer(MPU6050_ADDR, WHO_AM_I_REG, &check, 1);
	//HAL_I2C_EnableListen_IT(&hi2c);
	//HAL_I2C_Mem_Read_IT( I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1);
    HAL_I2C_Mem_Read_IT(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1);

    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 250 ?/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

//void I2Cx_Init(void)
//{
//	I2Cx_MspInit(&hi2c);
//	
//}

//void MPU6050_Init(void){
//     
//    uint8_t buffer[7];
// 
//    // включение/побудка модул€
//    buffer[0] = MPU6050_RA_PWR_MGMT_1;
//    buffer[1] = 0x00;
//    I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW, buffer, 2);
// 
//    // конфиг гироскопа на ±500∞/с
//    buffer[0] = MPU6050_RA_GYRO_CONFIG;
//    buffer[1] = 0x8;
//    I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW, buffer, 2);
// 
//    // конфиг акселерометра на ±8g
//    buffer[0] = MPU6050_RA_ACCEL_CONFIG;
//    buffer[1] = 0x10;
//    I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW, buffer, 2);
//}

//int32_t fGX_Cal, fGY_Cal, fGZ_Cal;

//void MPU6050_Calibrate(void){
//     
//  int16_t mpu6050data[6];
//  uint16_t iNumCM = 1000;
//	
//  for (int i = 0; i < iNumCM ; i ++){                  
//    MPU6050_GetAllData(mpu6050data);                                            
//    fGX_Cal += mpu6050data[3];
//    fGY_Cal += mpu6050data[4];                                       
//    fGZ_Cal += mpu6050data[5];                                        
//    HAL_Delay(3); // 3 сек на калибровку                                                    
//  }
//	
//  fGX_Cal /= iNumCM;                                                  
//  fGY_Cal /= iNumCM;                                                  
//  fGZ_Cal /= iNumCM; 
//  
//	//isinitialized = 1;
//}

//void MPU6050_GetAllData(int16_t *Data){
//   
//  uint8_t accelbuffer[14];
// 
//  // с 0x3B 14 следующих регистров содержат данные измерени€ модул€
//  I2C_ReadBuffer(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_XOUT_H, accelbuffer, 14);
// 
//  /* Registers 59 to 64 Ц Accelerometer Measurements */
//  for (int i = 0; i< 3; i++)
//      Data[i] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);
//   
//  /* Registers 65 and 66 Ц Temperature Measurement */
//  //пока пропускаем Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
// 
//  /* Registers 67 to 72 Ц Gyroscope Measurements */
//  for (int i = 4; i < 7; i++)
//      Data[i - 1] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);
// 
//}
