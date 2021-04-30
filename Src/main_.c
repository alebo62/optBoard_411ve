/*
PE7-  TIM1 ETR input   from ssi clk
PA0-  TIM2 ETR input   from ssi clk
PA5   SCK 		 input   from ssi clk

PA1-  EXT_INT1 input   from ssi frame (for start T1, T2)

PA2-  TIM2 CH3 output  to NSS ---> PA4 NSS input 



PA6   MISO 		 output  data to ssi tx --->
PE13- TIM1_CH3 output  to control ssi tx(open output) 0-ACTIVE ---> 

PA7   MOSI 		 input   data from ssi rx  --->

System CLock 64 MHz
*/
/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "stdio.h"
#include "tim.h"
//#include "i2c.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
это было!!!!!! CN7  5-7 CLOSE for BOOT

PЕ7- TIM1 ETR input   from ssi clk             (SB12 CLOSE !!!!)
PA0-  TIM2 ETR input   from ssi clk  
PA5   SCK 		 input   from ssi clk 

PA1-  EXT_INT1 input   from ssi frame (for start T1, T2)

PA2-  TIM2 CH3 output  to NSS ---> PA4 NSS input 

PA6   MISO 		 output  data to ssi tx 
PЕ9   TIM1_CH1 output  to control ssi tx(open output) 0-ACTIVE ---> 

PA7   MOSI 		 input   data from ssi rx 

System CLock 80 MHz

125us 8000
*/
typedef uint16_t U16;
typedef uint8_t U8;
typedef int16_t S16;
typedef int8_t S8;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USE_EXT_UART 0

#define CTRL_MSG_FROM_UART 1
#define NO_MSG_RCV 0
#define IS_MSG_RCV 1
const U16 MAX_LEN_BUF = 322;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t MPU6050_GetAllData(int16_t *Data);
extern int16_t Data[];
extern I2C_HandleTypeDef hi2c;
extern void I2Cx_Init(void);
extern volatile uint16_t flag;
extern volatile uint16_t rx_buf[8];
extern void conn_process(void);
extern uint16_t ssc_txFrame_buf[];
extern volatile U8 idle_frame_tx;
extern uint8_t is_connect;
extern U8 tract_id_my;
extern U8 xcmp_dev_num;
extern U16 mask;
extern U16 ssc_rcv_buf[];
extern U16 XNL_DATA_MSG_ACK[];
extern U16 XNL_DATA_MSG_REPLY[];
extern U16 ssc_tx_msg_length;
extern volatile U16 ssc_tx_msg_counter;
extern U16 ssc_idle_frame[];
extern U16 ssc_txData_buf[];
extern U16 ob_adress;
extern U16 xnl_flags;
extern U16 tract_id_my_hi;
extern U8 uart_ctrl_msg[64];       // = {0xC0,0}; передаются управляющие команды
extern U16 ctrl_msg_uart_ssi[400]; // = {0xABCD,0x0000,0x0000,0x000B,0x0000,0x0006};
extern U16 ssc_rcv_ctrl_msg_len;   // длинна сообщения
extern U16 rcv_msg_counter;        // счетчик принятых данных
extern U16 ssc_rcv_ctrl_msg[180];
extern uint8_t rcv_msg_state; // 1- being receive
extern U8 control_msg_ssi_uart_len;
extern void get_check_sum(U16 *);
U16 i;
extern U8 temp;
U8 control_msg_ssi_uart_len;
U8 uart1_rx_buf[128];
U16 copy_length;
U16 ssc_length;
U16 remain_len;
extern U16 tract_id_reply; // for reply
U8 temp_uart_buf[128];
U16 uart1_rx_msg_len;
U8 ssc_tx_sound; // должен идти звук
U8 need_reply;
U8 need_ack;
extern U8 first_time; // for audio route
U8 digital_mode;

U16 accel_counter;
U8 mean_counter; // среднее значение отсчетов
//int samplesX[8];
//int samplesY[8];
//int samplesZ[8];
int Z;
U8 md_counter; // счетчик горизонтального положения
U8 message_start;
volatile uint8_t uart1_rx_flag;
extern U8 sendMdMsg[];
U8 cnt84, cntB4, cntACK;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern volatile uint16_t flag;
extern volatile uint16_t rx_buf[8];
extern void conn_process(void);
extern uint16_t ssc_txFrame_buf[];
extern uint16_t idle_frame_tx_buf[];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
   /* USER CODE BEGIN 1 */
   int counter = 0;
   /* USER CODE END 1 */

   /* MCU Configuration--------------------------------------------------------*/

   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

   /* USER CODE BEGIN Init */

   /* USER CODE END Init */

   /* Configure the system clock */
   SystemClock_Config();

   /* USER CODE BEGIN SysInit */

   /* USER CODE END SysInit */

   /* Initialize all configured peripherals */

   MX_GPIO_Init();
   MX_TIM2_Init();
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
   MX_TIM1_Init();
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   MX_SPI1_Init();
   __HAL_SPI_ENABLE_IT(&hspi1, SPI_IT_RXNE | SPI_IT_TXE);
   //__HAL_I2C_ENABLE_IT(&hi2c, I2C_IT_EVT);
   memcpy((char *)ssc_txFrame_buf, (char *)idle_frame_tx_buf, 16);
   I2Cx_Init();
	 
	 configureCCS811();
		while(1);
   while (1)
   {
      if (flag)
      {
         flag = 0;
         if (is_connect == 1)
         {
            switch (rcv_msg_state)
            {

            case NO_MSG_RCV: // начало приема сообщения
               mask = ssc_rcv_buf[3] & 0xF000;
               if (mask == 0x4000)
               {
                  switch (ssc_rcv_buf[3] & 0x0F00)
                  {
                  case 0x0: //simple message
                     ssc_rcv_ctrl_msg_len = ssc_rcv_buf[3] & 0xff;
                     if (ssc_rcv_ctrl_msg_len % 2)
                        ssc_rcv_ctrl_msg_len++;
                     ssc_rcv_ctrl_msg_len += 2;
                     rcv_msg_counter = 0;
                     rcv_msg_state = IS_MSG_RCV; // теперь надо получить остальное message include checksum ...
                     break;
                  case 0x0100: // first fragment for sound
                     break;
                  case 0x0200: // middle fragment for sound
                     break;
                  case 0x0300: // last fragment
                     break;
                  default:
                     break;
                  }
               }
               else
               {
                  if (need_ack && idle_frame_tx)
                  {
                     need_ack = 0;
                     ssc_tx_msg_length = 10;
                     ssc_tx_msg_counter = 0;
                     memcpy((U8 *)ssc_txData_buf, (U8 *)XNL_DATA_MSG_ACK, 20);
                     idle_frame_tx = 0; //  ssc transmit
                  }
                  if (need_reply && idle_frame_tx)
                  {
                     need_reply = 0;
                     get_check_sum(XNL_DATA_MSG_REPLY);
                     ssc_tx_msg_length = 14;
                     ssc_tx_msg_counter = 0;
                     memcpy((U8 *)ssc_txData_buf, (U8 *)XNL_DATA_MSG_REPLY, 28);
                     idle_frame_tx = 0; //  ssc transmit
                  }
               }
               break; // end NO_MSG_RCV

            case IS_MSG_RCV: //   продолжаем прием сообщения
               memcpy((ssc_rcv_ctrl_msg + (rcv_msg_counter >> 1)), (ssc_rcv_buf + 2), 4);
               rcv_msg_counter += 4;
               if (rcv_msg_counter >= ssc_rcv_ctrl_msg_len)
               { //получили всё сообщение
                  rcv_msg_state = NO_MSG_RCV;

                  switch (ssc_rcv_ctrl_msg[1])
                  {
                  case 0x0B:
                     printf("%x \n", ssc_rcv_ctrl_msg[7]);
                     XNL_DATA_MSG_ACK[4] = ssc_rcv_ctrl_msg[2];
                     XNL_DATA_MSG_ACK[7] = ssc_rcv_ctrl_msg[5];
                     get_check_sum(XNL_DATA_MSG_ACK);
                     cntACK++;
                     if (idle_frame_tx)
                     {
                        need_ack = 0;
                        ssc_tx_msg_length = 10;
                        ssc_tx_msg_counter = 0;
                        memcpy((U8 *)ssc_txData_buf, (U8 *)XNL_DATA_MSG_ACK, 20);
                        idle_frame_tx = 0; //  ssc transmit
                     }
                     else
                        need_ack = 1;
                     break;

                  case 0x0C: // xnl data ack
                     break;
                  default:
                     break;
                  }

                  if (ssc_rcv_ctrl_msg[7] == 0x041D) // control message from usb device
                  {
                     xnl_flags++;
                     if (xnl_flags == 0x0108)
                        xnl_flags = 0x0100;
                     XNL_DATA_MSG_REPLY[4] = xnl_flags; // now prepare reply message
                     XNL_DATA_MSG_REPLY[7] = tract_id_reply;
                     XNL_DATA_MSG_REPLY[8] = 0x0005;
                     XNL_DATA_MSG_REPLY[9] = 0x841D;
                     XNL_DATA_MSG_REPLY[10] = 0x0000 | (ssc_rcv_ctrl_msg[8] >> 8); // result , function
                     XNL_DATA_MSG_REPLY[11] = (ssc_rcv_ctrl_msg[13] & 0xFF00);     // session , .....
                     need_reply = 1;
                     if ((ssc_rcv_ctrl_msg[14] & 0xff) == 0x0055) // if was reload then need audioroute
                     {
                        //                  first_time = 0;
                     }
                     else if ((ssc_rcv_ctrl_msg[14] & 0xff) == 0x00AA) // go to analog mode for rcv
                     {
                        //                  digital_mode = 0;
                     }
                     else if ((ssc_rcv_ctrl_msg[14] & 0xff) == 0x00FF) // go to digital mede for rcv
                     {
                        //                  digital_mode = 1;
                     }
                     else if ((ssc_rcv_ctrl_msg[14] & 0xff) == 0x000A) // ip ferr agent
                     {
                        //                  sound_msg_ssi_fa1[13] = 0x0A02; // 192.168.10.2 for rx sound
                     }
                     else if ((ssc_rcv_ctrl_msg[14] & 0xff) == 0x0014) // ip ferr agent
                     {
                        //                  sound_msg_ssi_fa1[13] = 0x1402; // 192.168.20.2 for rx sound
                     }
                  }
                  if (ssc_rcv_ctrl_msg[7] == 0x841D)
                     cnt84++;
                  if (ssc_rcv_ctrl_msg[7] == 0xB41D)
                     cntB4++;
                  //printf("%x %x %x\n", ssc_rcv_ctrl_msg[7] , ssc_rcv_ctrl_msg[8], ssc_rcv_ctrl_msg[10]);

                  if (ssc_rcv_ctrl_msg[7] == 0xB40E) //  передача!!!
                  {
                     //               if ((ssc_rcv_ctrl_msg[9] & 0xFF00) == 0x1100) // MIC_ENABLED_SEL
                     //               {
                     //                  memset((U8 *)(sound_msg_uart_ssi + 134), 0, 68);
                     //                  start_buff_out = 0;
                     //                  sound_frame_flag = 1;
                     //               }
                     //               else //((ssc_rcv_ctrl_msg[9] & 0xFF00) == 0x0000)// MIC_DISABLED
                     //               {
                     //                  sound_frame_count_in = 0;
                     //                  sound_frame_count_out = 0;
                     //                  sound_frame_flag = 0;
                     //                  memset((U8 *)(sound_msg_uart_ssi + 3), 0, 252);
                     //                  mic_enable = 0;
                     //               }
                  }
                  else if (ssc_rcv_ctrl_msg[7] == 0xB41E) // call control broadcast
                  {
                     if (digital_mode)
                     {
                        //                  if ((ssc_rcv_ctrl_msg[8] & 0xFF) == 0x07) // call in hangtime
                        //                  {
                        //                     //uart1_rx_flag = 0; // stop sound
                        //                     ssi_rx_cnt = 0;
                        //                     rx_enable = 0;
                        //                     call_decoded = 0;
                        //                     sound_frame_flag = 0;
                        //                     cntr = 0;
                        //                  }
                        //                  else if ((ssc_rcv_ctrl_msg[8] & 0xFF) == 0x08) // call decoded
                        //                  {
                        //                     call_decoded = 1;
                        //                     rx_enable = 1;
                        //                     j = 0;
                        //                     ssi_rx_cnt = 0;
                        //                     memset((U8 *)(sound_msg_uart_ssi + 3), 0, 252);
                        //                  }

                        //                  else if ((ssc_rcv_ctrl_msg[8] & 0xFF) == 0x03) // call ended
                        //                  {
                        //                     //uart1_rx_flag = 0; // stop sound
                        //                     ssi_rx_cnt = 0;
                        //                     call_decoded = 0;
                        //                     rx_enable = 0;
                        //                     sound_frame_flag = 0;
                        //                     cntr = 0;
                        //                     memset((U8 *)(sound_msg_ssi_fa1 + 18), 0, 222);
                        //                     memset((U8 *)(sound_msg_ssi_fa2 + 2), 0, 254);
                        //                  }
                     }
                     else // analog mode
                     {
                        //                  if ((ssc_rcv_ctrl_msg[8] & 0xFF) == 0x03) // call ended
                        //                  {
                        //                     //uart1_rx_flag = 0; // stop sound
                        //                     ssi_rx_cnt = 0;
                        //                     rx_enable = 0;
                        //                     call_decoded = 0;
                        //                     sound_frame_flag = 0;
                        //                     cntr = 0;
                        //                     j = 0;
                        //                  }
                        //                  else if ((ssc_rcv_ctrl_msg[8] & 0xFF) == 0x01) // receive!!!!
                        //                  {
                        //                     call_decoded = 1;
                        //                     rx_enable = 1;
                        //                     j = 0;
                        //                     ssi_rx_cnt = 0;
                        //                     counter = 0;
                        //                     sound_frame_flag = 0;
                        //                     delay = 0;
                        //                  }
                     }
                  }
                  else if (ssc_rcv_ctrl_msg[7] == 0xB41C) //
                  {
                     //               if (ssc_rcv_ctrl_msg[8] == 0x0310) // call remote monitor
                     //               {
                     //                  call_decoded = 1;
                     //                  rx_enable = 1;
                     //                  j = 0;
                     //                  ssi_rx_cnt = 0;
                     //                  memset((U8 *)(sound_msg_uart_ssi + 3), 0, 252);
                     //               }
                  }

                  if (!first_time && (ssc_rcv_ctrl_msg[7] == 0xB407)) // end of initialization go to audio route
                  {
                     //printf("%x %x\n", ssc_rcv_ctrl_msg[7], ssc_rcv_ctrl_msg[8]);
                     //               first_time = 1;
                     //               audio_route_req[6] = sound_msg_ssi_fa1[6] = ob_adress;
                     //               TC_Start(TC0, 2); // wait 2ms and write AUD_ROUTE see need_aud_route
                  }

                  break; // end IS_MSG_RCV
               default:
                  break;
               }
            }
            accel_counter++;
            if (accel_counter >= 4000) //1/4 sec{
            {
               //accel_demo();
               if (MPU6050_GetAllData(Data) == 0)
               {
                  Z += Data[2];
                  mean_counter++;
                  if (mean_counter == 8)
                  {
                     mean_counter = 0;
                     GPIOD->ODR ^= 0x1000; // pd12 green
                     Z = Z >> 3;
                     if (((Z < 25600) && (Z > 23600)) || ((Z < -7500) && (Z > -9500)))
                     {
                        //printf("%d %d\n", Z , md_counter);
                        if (md_counter < 7)
                           md_counter++;
                        if (md_counter >= 3)
                        {
                           if (!message_start)
                           {
                              message_start = 1;
                              GPIOA->ODR ^= 0x200;

                              ++xnl_flags;
                              if (xnl_flags == 0x0108)
                                 xnl_flags = 0x0100;
                              ++tract_id_my;
                              tract_id_my_hi += tract_id_my;
                              ctrl_msg_uart_ssi[4] = xnl_flags;
                              ctrl_msg_uart_ssi[6] = ob_adress;
                              ctrl_msg_uart_ssi[7] = tract_id_my_hi;

                              ctrl_msg_uart_ssi[8] = 17;
                              ctrl_msg_uart_ssi[1] = 0x4000 + (12 + 2 + 17);

                              memcpy(ctrl_msg_uart_ssi + 9, sendMdMsg, 17 + 1);
                              ssc_tx_msg_length = 9;                             // (17+1) / 2 длинна теперь в словах
                              ctrl_msg_uart_ssi[ssc_tx_msg_length + 9] = 0x00BA; // +терминатор
                              if (ssc_tx_msg_length % 2)
                              { // для окончания фрейма добавить 0х0000 , если нечетное к-во слов
                                 ctrl_msg_uart_ssi[ssc_tx_msg_length + 10] = 0x0000;
                                 ssc_tx_msg_length++;
                              }
                              get_check_sum(ctrl_msg_uart_ssi);
                              // отправка сформированного сообщения
                              ssc_tx_msg_length = 10 + 10; // ssc_tx_msg_length + 3(ssi)+6(xnml)+1(00BA)
                              ssc_tx_msg_counter = 0;
                              memcpy((ssc_txData_buf), (ctrl_msg_uart_ssi), (ssc_tx_msg_length << 1));
                              idle_frame_tx = 0;
                           }
                        }
                     }
                     else
                     {
                        if (md_counter > 0)
                           md_counter--;
                        if (md_counter < 3)
                        {
                           if (message_start)
                           {
                              GPIOA->ODR &= ~0x200;
                              message_start = 0;
                           }
                        }
                        //printf("Acc: %d %d\n", md_counter, Z);
                     }
                     Z = 0;
                  }
                  accel_counter = 0;
               }
            }
         }
         else
            conn_process();
      }
   }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

   /** Configure the main internal regulator output voltage 
  */
   __HAL_RCC_PWR_CLK_ENABLE();
   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
   /** Initializes the CPU, AHB and APB busses clocks 
  */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = 4;
   RCC_OscInitStruct.PLL.PLLN = 80;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
   RCC_OscInitStruct.PLL.PLLQ = 4;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   {
      Error_Handler();
   }
   /** Initializes the CPU, AHB and APB busses clocks 
  */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
   {
      Error_Handler();
   }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
   /* USER CODE BEGIN Error_Handler_Debug */
   /* User can add his own implementation to report the HAL error return state */

   /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
   /* USER CODE BEGIN 6 */
   /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
   /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/