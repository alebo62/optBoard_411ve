#include "gpio.h"
#include "stm32f4xx_hal_i2c.h"
/*
 mpu6050: пины 17-20 смотр€т вверх

  алибровка: 
 Acc: 989 1723 24600 корпус вверху
 Gyro: 0 87 0 
 Acc: 1510 1593 -8496 корпус внизу
 Gyro: 0 80 -17


*/
//#define ONLY_Z 1

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
uint16_t flag_tc;
uint8_t MPU6050_RA_ACCEL_XOUT_H = 0x3B;
uint8_t accelbuffer[14];
int16_t Data[6];
//int16_t gData[3];
int32_t fGX_Cal, fGY_Cal, fGZ_Cal;
int32_t fAX_Cal, fAY_Cal, fAZ_Cal;
I2C_HandleTypeDef hi2c;

void MPU6050_Calibrate(void);
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);
uint8_t MPU6050_GetAllData(int16_t *Data);

void I2Cx_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  __HAL_RCC_I2C1_CLK_ENABLE();

   __HAL_RCC_GPIOB_CLK_ENABLE();

  /* I2Cx SD1 & SCK pin configuration */
  GPIO_InitStructure.Pin = GPIO_PIN_7   | GPIO_PIN_8  ;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
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
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn); 
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
    I2Cx_MspInit(&hi2c);// configure pin7,8 portB
    if (HAL_I2C_Init(&hi2c) != HAL_OK)
		{
			Error_Handler();
		}
		//__HAL_I2C_DISABLE_IT(&hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
		
		MPU6050_Init(&hi2c);// test
		
		//MPU6050_Calibrate();
		//printf("Acc: %d %d %d\n", fAX_Cal, fAY_Cal, fAZ_Cal);
		//printf("Gyro: %d %d %d\n", fGX_Cal, fGY_Cal, fGZ_Cal);
		//while(1)
		//{
		//MPU6050_GetAllData(Data);
		//printf("Acc: %d %d %d\n", Data[0], Data[1], Data[2]);
		//printf("Gyro: %d %d %d\n", Data[3], Data[4], Data[5]);
		//HAL_Delay(1000);
		//}
  }
}

//void I2C_WriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer, uint8_t TXBUFFERSIZE) {
//    while(HAL_I2C_Master_Transmit(&hi2c, (uint16_t)I2C_ADDRESS<<1, (uint8_t*)aTxBuffer, (uint16_t)TXBUFFERSIZE, (uint32_t)1000)!= HAL_OK){
//        if (HAL_I2C_GetError(&hi2c) != HAL_I2C_ERROR_AF){
//            //_Error_Handler(__FILE__, aTxBuffer[0]);
//        }
// 
//    }
// 
//      while (HAL_I2C_GetState(&hi2c) != HAL_I2C_STATE_READY){}
//}

//void I2C_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE){
// 
//    I2C_WriteBuffer(I2C_ADDRESS, &RegAddr, 1);
// 
//    while(HAL_I2C_Master_Receive(&hi2c, (uint16_t)I2C_ADDRESS<<1, aRxBuffer, (uint16_t)RXBUFFERSIZE, (uint32_t)1000) != HAL_OK){
//        if (HAL_I2C_GetError(&hi2c) != HAL_I2C_ERROR_AF){
//            //_Error_Handler(__FILE__, __LINE__);
//        }
//    }
// 
//    while (HAL_I2C_GetState(&hi2c) != HAL_I2C_STATE_READY){}
//}



uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
	uint8_t check;
	 uint8_t Data;  
    // check device ID WHO_AM_I
	 
		HAL_I2C_Mem_Read_IT(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1);
		while(hi2c.State != HAL_I2C_STATE_READY);

    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1);
				while(hi2c.State != HAL_I2C_STATE_READY);
        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1);
				while(hi2c.State != HAL_I2C_STATE_READY);
        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g
        Data = 0x00;
        HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1);
				while(hi2c.State != HAL_I2C_STATE_READY);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 250 ?/s
        Data = 0x00;
        HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1);
				while(hi2c.State != HAL_I2C_STATE_READY);
				GPIOA->ODR |= 0x200;
        return 0;
    }
    return 1;
}



void MPU6050_Calibrate(void){
     
  int16_t mpu6050data[6];
  uint16_t iNumCM = 1000;
	
  for (int i = 0; i < iNumCM ; i ++){                  
    MPU6050_GetAllData(mpu6050data);
		fAX_Cal += (int32_t)mpu6050data[0];
    fAY_Cal += (int32_t)mpu6050data[1];                                       
    fAZ_Cal += (int32_t)mpu6050data[2];
		
    fGX_Cal += (int32_t)mpu6050data[3];
    fGY_Cal += (int32_t)mpu6050data[4];                                       
    fGZ_Cal += (int32_t)mpu6050data[5];                                        
    HAL_Delay(3); // 3 сек на калибровку                                                    
  }
	fAX_Cal /= iNumCM;                                                  
  fAY_Cal /= iNumCM;                                                  
  fAZ_Cal /= iNumCM;
	
  fGX_Cal /= iNumCM;                                                  
  fGY_Cal /= iNumCM;                                                  
  fGZ_Cal /= iNumCM; 
  
}


//void MPU6050_GetAllData(int16_t *Data){
//   int i;
  // с 0x3B 14 следующих регистров содержат данные измерени€ модул€
//	HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, 14, accelbuffer, 14);
//  HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, 1, accelbuffer, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
//	HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+1, 1, accelbuffer+1, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
//	HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+2, 1, accelbuffer+2, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
//	HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+3, 1, accelbuffer+3, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
//	HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+4, 1, accelbuffer+4, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
//	HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+5, 1, accelbuffer+5, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
//  /* Registers 59 to 64 Ц Accelerometer Measurements */
//  
//	/* Registers 65 and 66 Ц Temperature Measurement */
//  //пока пропускаем Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53

//  HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H +8, 1, accelbuffer+8, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
//	HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+9, 1, accelbuffer+9, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
//	HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+10, 1, accelbuffer+10, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
//	HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+11, 1, accelbuffer+11, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
//	HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+12, 1, accelbuffer+12, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
//	HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+13, 1, accelbuffer+13, 1);
//  while(hi2c.State != HAL_I2C_STATE_READY);
  /* Registers 67 to 72 Ц Gyroscope Measurements */
//	for (i = 0; i< 3; i++)// accel
//      Data[i] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);
//	
//  for (i = 4; i < 7; i++)// gyro
//      Data[i] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);
// 
//  
//}

uint8_t MPU6050_GetAllData(int16_t *Data){
   int i;
	 static uint8_t state = 0;
	 if(hi2c.State == HAL_I2C_STATE_READY){
	 // accel
	 #ifdef ONLY_Z
		if(state == 0){
			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+4, 1, accelbuffer+4, 1);
			state = 1;
			}
		else if(state == 1){
			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+5, 1, accelbuffer+5, 1);
			state = 0;
		  
			Data[0] = ((int16_t) ((uint16_t) accelbuffer[0] << 8) + accelbuffer[1]);
			Data[1] = ((int16_t) ((uint16_t) accelbuffer[2] << 8) + accelbuffer[3]);
			Data[2] = ((int16_t) ((uint16_t) accelbuffer[4] << 8) + accelbuffer[5]);
		}
	 #else
		if(state == 0){
//			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, 1, accelbuffer, 1);
//			state = 1;
//		}
//		else if(state == 1){
//			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+1, 1, accelbuffer+1, 1);
//			state = 2;
//		}
//		else if(state == 2){
			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+2, 1, accelbuffer+2, 1);
			state = 3;
		}
		else if(state == 3){
			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+3, 1, accelbuffer+3, 1);
			state = 4;
		}
		else if(state == 4){
			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+4, 1, accelbuffer+4, 1);
			state = 5;
		}
		else if(state == 5){
			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+5, 1, accelbuffer+5, 1);
			state = 0;
//		}
		// gyro
//		else if(state == 8){
//			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+8, 1, accelbuffer+8, 1);
//			state = 9;
//		}
//		else if(state == 9){
//			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+9, 1, accelbuffer+9, 1);
//			state = 10;
//		}
//		else if(state == 10){
//			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+10, 1, accelbuffer+10, 1);
//			state = 11;
//		}
//		else if(state == 11){
//			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+11, 1, accelbuffer+11, 1);
//			state = 12;
//		}
//		else if(state == 12){
//			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+12, 1, accelbuffer+12, 1);
//			state = 13;
//		}		
//		else if(state == 13){
//			HAL_I2C_Mem_Read_IT(&hi2c,MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H+13, 1, accelbuffer+13, 1);
//			state = 0;
			
			for (i = 1; i< 3; i++)// accel
				Data[i] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);
	
//			for (i = 4; i < 7; i++)// gyro
//				Data[i] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);
}		
		}
		#endif 
		return state;
// } 
}

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
