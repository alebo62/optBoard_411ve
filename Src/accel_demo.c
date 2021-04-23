#include "stddef.h"
#include "stdint.h"
#include "main.h"
#include "accelerometr.h"
//#inlcude "core_mc4.h"
//#inlcude "stm32f4xx_hal_def.h"

extern uint8_t BSP_ACCELERO_Init(void);
extern void Error_Handler(void);

uint8_t Counter  = 0x00;
__IO uint16_t MaxAcceleration = 0;
uint8_t *buf;
  uint16_t Temp_X, Temp_Y = 0x00;
  uint16_t NewARR_X, NewARR_Y = 0x00;
/* Variables used for accelerometer */
__IO int16_t X_Offset, Y_Offset;
int16_t Buffer[3];
/* MEMS thresholds {Low/High} */
static int16_t ThreadholdAcceleroLow = -1500, ThreadholdAcceleroHigh = 1500;

void accel_demo(void)
{
//		if(BSP_ACCELERO_Init() != HAL_OK)
//  {
//    Error_Handler();
//  }
/* Read Acceleration*/
//   while(1)
	 {
		BSP_ACCELERO_GetXYZ(Buffer);

		/* Set X and Y positions */
		X_Offset = Buffer[0];
		Y_Offset = Buffer[1];
		//printf("hello\n");
    printf("%d   %d\n", X_Offset, Y_Offset);
		/* Update New autoreload value in case of X or Y acceleration*/
		/* Basic acceleration X_Offset and Y_Offset are divide by 40 to fir with ARR range */
		//NewARR_X = TIM_ARR - ABS(X_Offset/40);
		//NewARR_Y = TIM_ARR - ABS(Y_Offset/40);

		/* Calculation of Max acceleration detected on X or Y axis */
//		Temp_X = ABS(X_Offset/40);
//		Temp_Y = ABS(Y_Offset/40);
//		MaxAcceleration = MAX_AB(Temp_X, Temp_Y);
//		HAL_Delay(1000);
		
		}
		
		
}