/*
PE7-  TIM1 ETR input   from ssi clk
PA0-  TIM2 ETR input   from ssi clk
PA5   SCK 		 input   from ssi clk

PA1-  EXT_INT1 input   from ssi frame (for start T1, T2)

PA2-  TIM2 CH3 output  to NSS ---> PA4 NSS input 



PA6   MISO 		 output  data to ssi tx --->
PE9 - TIM1_CH1 output  to control ssi tx(open output) 0-ACTIVE ---> 

PA7   MOSI 		 input   data from ssi rx  --->

System CLock 64 MHz
*/
/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "main.h"
#include "nrf24L01.h"
#include "spi.h"
#include "stdio.h"
#include "tim.h"
//#include "i2c.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
это было!!!!! CN7  5-7 CLOSE for BOOT

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
#define ACCEL
#define NRF

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
U8 i;
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
void init_messages(void);
U16 accel_counter;
U8 mean_counter; // среднее значение отсчетов
//int samplesX[8];
//int samplesY[8];
//int samplesZ[8];
int Z, X, Y;
U8 md_counter; // счетчик горизонтального положения
U8 message_start;
U8 chan_change = 0;
void send_chan_change(U16 ch);
extern U16 chngChanMsg[];
volatile uint8_t uart1_rx_flag;
extern U16 sendMdMsg[];
extern U8 state;
extern U8 send_buf[];
extern U8 rcv_buf[];
extern U8 send_buf_cnt;
extern U8 rcv_buf_cnt;
extern SPI_HandleTypeDef hspi2;
extern U8 send_delay;
U8 payloadNRF;
U8 payloadNRFold = 0;
U8 need_registrate = 0;
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
extern volatile uint8_t nrf_rx_flag;
uint8_t buf1[20] = {0};
uint8_t dt_reg = 0;
uint16_t repiter_channel;
uint16_t current_channel;
uint32_t *source_addr;
uint32_t *dest_addr;
const uint8_t option_cnt = 16;
enum
{
   CHAN_RPT = 0,
   REP_ALRM_TIM,
   MAX_CNT_ACCEL,
   PRG_CNT_ACCEL,
   Z1MAX,
   Z1MIN,
   Z2MAX,
   Z2MIN,
   Y1MAX,
   Y1MIN,
   Y2MAX,
   Y2MIN,
   CH_24G,    // 2.4G
   REG_TIMER, // 2.4G
   INDOOR_POS,
	 CTRL_RAD_NUM
};
int32_t options[option_cnt] = {
    0x0400, //0 chan 4 << 8 
    30,     //1 repeit alarm timer sec
    7,      //2 max cnt accel
    3,      //3 porog cnt accel
    25600,  //4 z1max
    23600,  //5 z1min
    -7500,  //6 z2max
    -9500,  //7 z2min
    16600,  //8 y1max
    15000,  //9 y1min
    -15000, //10 y2max
    -16600, //11 y2min
    76,     //12 ch num 2.4G
    30,     //13 registration timer 2.4G
    0,       //14 indoor registration
		200     // 15control station number
};
uint8_t test[16];

FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t SectorError = 0;

uint32_t Address, AddressW;
uint32_t *pOption;
uint8_t reg_message_start;
uint8_t need_reg_count;// счетчик для перерегистрации
uint8_t prog_opt_cnt;
uint8_t need_prog;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
   /* USER CODE BEGIN 1 */
   //int counter = 0;
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
	 
   MX_TIM3_Init();
	 htim3.Instance->ARR = options[REG_TIMER] * 2000;
   __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
	 
	 MX_SPI2_Init();
   NRF24_ini();

   MX_TIM2_Init();
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
   
	 MX_TIM1_Init();
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	 
	 MX_TIM4_Init();
   __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
	 	 
//	 MX_TIM5_Init();
//   __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC1);
	 
//	 __HAL_TIM_ENABLE(&htim5);
//	 while(1);

   MX_SPI1_Init();
   __HAL_SPI_ENABLE_IT(&hspi1, SPI_IT_RXNE | SPI_IT_TXE);
   
	 I2Cx_Init();
   memcpy((char *)ssc_txFrame_buf, (char *)idle_frame_tx_buf, 16);
   init_messages();

   detect_free(); // где начинается свободная флещ

   if (FA_RPT_CH == AddressW) // если первый раз, то копируем во флеш
   {
      write_flash();
      AddressW += (4 * option_cnt);
   }
   else
      read_flash();
   options[INDOOR_POS] = 1; // !!!!!
	 options[PRG_CNT_ACCEL] = 1;// !!!!!
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
                     //printf("%x \n", ssc_rcv_ctrl_msg[7]);
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
									
									if (ssc_rcv_ctrl_msg[7] == 0xB41E) // call control broadcast
                  {
										if ((ssc_rcv_ctrl_msg[8] & 0xFF) == 0x08) // call decoded
										{
											chan_change = 12; // channel busy
										}
										else if ((ssc_rcv_ctrl_msg[8] & 0xFF) == 0x03) // call ended
                    {
											chan_change = 0; // channel busy
                    }
									}
									
									else if (ssc_rcv_ctrl_msg[7] == 0xB40E) //  передача!!!
                  {
                     if ((ssc_rcv_ctrl_msg[9] & 0xFF00) == 0x1100) // MIC_ENABLED_SEL
                     {
												chan_change = 11; // channel busy
										 }
                     else //((ssc_rcv_ctrl_msg[9] & 0xFF00) == 0x0000)// MIC_DISABLED
                     {
												chan_change = 0; // free channel
                     }
                  }
									                  
                  else if (ssc_rcv_ctrl_msg[7] == 0x841D) //send data reply
                  {
                     //cnt84++;
                  }
									
                  else if (ssc_rcv_ctrl_msg[7] == 0xB41D) // send data broadcast
                  {
                     if ((chan_change == 3) || (chan_change == 8))
                     {
                        chan_change = 4;
                     }
                  }
									
                  else if (ssc_rcv_ctrl_msg[7] == 0xB40D) // chan change broadcast
                  {
                     if (chan_change == 1)
                        chan_change = 2; // канал переключен - можно передавать тревогу
										 else if(chan_change == 6)
                        chan_change = 7; // канал переключен - можно передавать регистрацию		
                     else if (chan_change == 5)
										 {
                        chan_change = 0; // теперь разговоры на этом канале(4)
												//if(payloadNRF < 0x80)// если не использован старший бит сбрасываем необх-сть регистрации
												
												reg_message_start = 0;	
										 }
                     else if (chan_change == 10)// сразу после инициализации борды
                     {
                        current_channel = ssc_rcv_ctrl_msg[9] << 8;
                        printf("CH = 0x%02X ", current_channel);
												chan_change = 0;
                     }
                  }
									
									else if (ssc_rcv_ctrl_msg[7] == 0x041D) // control message from usb device
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
                     //printf("0x%02X\n", ssc_rcv_ctrl_msg[14]);
                     //printf("0x%02X\n", ssc_rcv_ctrl_msg[15]);
                     //printf("0x%02X\n", ssc_rcv_ctrl_msg[16]);
                     if ((ssc_rcv_ctrl_msg[14] & 0xff) == 'P' && (ssc_rcv_ctrl_msg[15] == 0x5247)) // == "PRG"
                     {
                        prog_opt_cnt = (ssc_rcv_ctrl_msg[14] >> 8) - 3;// 0x0Nxx - PRG
                        while (prog_opt_cnt)
                        {
                           options[ssc_rcv_ctrl_msg[16 + i]] = ssc_rcv_ctrl_msg[17 + i];
                           prog_opt_cnt -= 2;
                           i += 2;
                        }
                        need_prog = 1;
                     }
                  }
                  
                  else if (ssc_rcv_ctrl_msg[7] == 0xB41C) //
                  {
                     //if (ssc_rcv_ctrl_msg[8] == 0x0310) // call remote monitor
                  }

                  else if ((ssc_rcv_ctrl_msg[7] == 0xB407) && !first_time ) // end of initialization go to audio route
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
#ifdef NRF
            if (!(GPIOA->IDR & 0x08))
            {
               switch (state)
               {
               case 0:
                  LED_TGL;
                  send_buf[0] = 0;
                  send_buf[1] = STATUS; // проверяем статус
                  CS_ON;
                  hspi2.Instance->DR = send_buf[1];
                  send_buf_cnt = 1;
                  rcv_buf_cnt = 2;
                  __HAL_SPI_ENABLE_IT(&hspi2, SPI_IT_RXNE | SPI_IT_TXE);
                  //status = NRF24_ReadReg(STATUS);
                  //printf("check status ");
                  state = 1;
                  htim4.Instance->CNT = 1;
                  __HAL_TIM_ENABLE(&htim4);
                  break;
               case 1:
                  if (send_delay && (rcv_buf[1] & 0x40))
                  {
                     //printf("0x%02X ", rcv_buf[1]);
                     send_delay = 0;
                     send_buf[0] = 0;
                     send_buf[1] = RD_RX_PLOAD; // читаем данные
                     CS_ON;
                     hspi2.Instance->DR = send_buf[1];
                     send_buf_cnt = 2;
                     rcv_buf_cnt = 4;
                     __HAL_SPI_ENABLE_IT(&hspi2, SPI_IT_RXNE | SPI_IT_TXE);
                     //NRF24_Read_Buf(RD_RX_PLOAD,RX_BUF, TX_PLOAD_WIDTH);
                     //printf("STATUS: 0x%02X\n",RX_BUF[0]);
                     state = 2;
                     htim4.Instance->CNT = 1;
                     __HAL_TIM_ENABLE(&htim4);
                  }
                  break;
               case 2:
                  if (send_delay)
                  {
                     send_delay = 0;
                     if (rcv_buf[2] == (uint8_t)(~rcv_buf[1])) // simple registration
                     {
                        payloadNRF = rcv_buf[2];
                        if (payloadNRF != payloadNRFold)
                           need_registrate = 1;
												payloadNRFold = payloadNRF;	 
                     }
										 /*if (rcv_buf[2] == (uint8_t)(~rcv_buf[1])) // extended registration
                     {
                        payloadNRF = rcv_buf[2];
                        if (payloadNRF != payloadNRFold)
													need_reg_count++;
												else
													need_reg_count = 0;
												if(need_reg_count == 2)
												{	
                           need_registrate = 1;
													 need_reg_count = 0;
													 payloadNRFold = payloadNRF;
												}		
                     }										 */
										 memcpy(test, rcv_buf, 6);
                     //printf("rcv: 0x%02X\n", rcv_buf[1]);
                     send_buf[0] = 0x40;
                     send_buf[1] = STATUS | 0x20;
                     CS_ON;
                     hspi2.Instance->DR = send_buf[1];
                     send_buf_cnt = 1;
                     rcv_buf_cnt = 3;
                     __HAL_SPI_ENABLE_IT(&hspi2, SPI_IT_RXNE | SPI_IT_TXE);
                     //NRF24_WriteReg(STATUS, 0x40);
                     state = 3;
                     htim4.Instance->CNT = 1;
                     __HAL_TIM_ENABLE(&htim4);
                  }
                  break;
               case 3:
                  if (send_delay)
                     state = 0;
               default:
                  break;
               }
            }
#endif
#ifdef ACCEL
            else if (accel_counter >= 4000) //1/4 sec{
            {
               //accel_demo();
               if (MPU6050_GetAllData(Data) == 0)
               {
                  Z += Data[2];
                  Y += Data[1];
                  mean_counter++;
                  if (mean_counter == 64) // 2-4-8-16... отсчетов,чтобы среднее найти просто (>>)
                  {
                     mean_counter = 0;
                     GPIOD->ODR ^= 0x1000; // pd12 green
                     Z = Z >> 6;
                     Y = Y >> 6;
                     printf("Y: %d Z: %d\n", Y, Z);
                     if ((((Z < options[Z1MAX]) && (Z > options[Z1MIN])) || ((Z < options[Z2MAX]) && (Z > options[Z2MIN]))) || (((Y < options[Y1MAX]) && (Y > options[Y1MIN])) || ((Y < options[Y2MAX]) && (Y > options[Y2MIN])))) // horisontal position
                     {
                        //printf("%d %d\n", Z , md_counter);
                        if (md_counter < options[MAX_CNT_ACCEL])
                           md_counter++;
                        if (md_counter >= options[PRG_CNT_ACCEL])
                        {
                           if (!chan_change && !message_start)
                           {
													    message_start = 1; // timer3 
                              send_chan_change(0x0400); // 0x04<<8
                              chan_change = 1;
															payloadNRF |= 0x80; // alarm
                           }
                        }
                     }
                     else
                     {
                        if (md_counter > 0)
                           md_counter--;
                        if (md_counter < options[PRG_CNT_ACCEL])
                        {
                           if (message_start)
                           {
                              GPIOA->ODR &= ~0x200;
                              message_start = 0; // можно передавать тревогу и не только...
                           }
                        }
                        //printf("Acc: %d %d\n", md_counter, Z);
                     }
                     Z = Y = 0;
                  }
                  accel_counter = 0;
               }
            }
#endif			
						if ((chan_change == 2) && !need_ack)// alarm
            {
						   chan_change = 3;
               
							 __HAL_TIM_ENABLE(&htim3); // 20 sec timer message_start reset
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
							 sendMdMsg[7] = 0x0200 + payloadNRF;
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
						else if ((chan_change == 7) && !need_ack)// registration
            {
							 chan_change = 8;
               reg_message_start = 1;
               __HAL_TIM_ENABLE(&htim3); // 20 sec timer message_start reset
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
							 sendMdMsg[7] = 0x0200 + payloadNRF;
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
						
            if ((chan_change == 4) && !need_ack)
            {
               send_chan_change(current_channel); // 0x05 << 8
               chan_change = 5;
            }
            
						if (need_registrate && options[INDOOR_POS] )
            {
               if (!chan_change && !reg_message_start)
               {
									need_registrate = 0;
									send_chan_change(0x0400); // 0x04<<8
                  chan_change = 6;
									payloadNRF &= 0x7F;// not alarm
               }
            }
						
						if(need_prog)
						{
						need_prog = 0;
						write_flash();
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
void send_chan_change(U16 ch)
{
   ++xnl_flags;
   if (xnl_flags == 0x0108)
      xnl_flags = 0x0100;
   ++tract_id_my;
   tract_id_my_hi += tract_id_my;
   ctrl_msg_uart_ssi[4] = xnl_flags;
   ctrl_msg_uart_ssi[6] = ob_adress;
   ctrl_msg_uart_ssi[7] = tract_id_my_hi;

   ctrl_msg_uart_ssi[8] = 7;
   ctrl_msg_uart_ssi[1] = 0x4000 + (12 + 2 + 7);
   chngChanMsg[3] = ch;
   memcpy(ctrl_msg_uart_ssi + 9, chngChanMsg, 7 + 1);
   ssc_tx_msg_length = 4;                             // (7+1) / 2 длинна теперь в словах
   ctrl_msg_uart_ssi[ssc_tx_msg_length + 9] = 0x00BA; // +терминатор
                                                      //	if (ssc_tx_msg_length % 2)
                                                      //	{ // для окончания фрейма добавить 0х0000 , если нечетное к-во слов
                                                      //		 ctrl_msg_uart_ssi[ssc_tx_msg_length + 10] = 0x0000;
                                                      //		 ssc_tx_msg_length++;
                                                      //	}
   get_check_sum(ctrl_msg_uart_ssi);
   // отправка сформированного сообщения
   ssc_tx_msg_length = ssc_tx_msg_length + 10; // ssc_tx_msg_length + 3(ssi)+6(xnml)+1(00BA)
   ssc_tx_msg_counter = 0;
   memcpy((ssc_txData_buf), (ctrl_msg_uart_ssi), (ssc_tx_msg_length << 1));
   idle_frame_tx = 0;
}

void init_messages(void)
{
   sendMdMsg[0] = 0x041D; // xcmp arcontrol  [9]
   sendMdMsg[1] = 0x0150;
   sendMdMsg[2] = 0x0204;
   sendMdMsg[3] = 0x0D00; //dest ip
   sendMdMsg[4] = 0x0514; // dest ip
   sendMdMsg[5] = 0x0FA4; // dest port
   sendMdMsg[6] = 0x0100; // session id
   sendMdMsg[7] = 0x2AA;  // payload len
   sendMdMsg[8] = 0xBB00; // payload
   sendMdMsg[9] = 0x00BA;
   sendMdMsg[10] = 0x0000;

   chngChanMsg[0] = 0x040D; // 9
   chngChanMsg[1] = 0x0600; // 10
   chngChanMsg[2] = 0x0100; // 11
   chngChanMsg[3] = 0x0500; //  12
   chngChanMsg[4] = 0x00BA; //  13
	 
	 chan_change = 10;
}
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