#include "nrf24L01.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_it.h"
#include "main.h"
//------------------------------------------------
extern TIM_HandleTypeDef htim4;
extern SPI_HandleTypeDef hspi2;

#define TX_ADR_WIDTH 5
#define TX_PLOAD_WIDTH 2
uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t RX_BUF[TX_PLOAD_WIDTH] = {0};
uint8_t send_delay; // for 1 msec delay
uint8_t send_buf[2] = {0};
uint8_t rcv_buf[8];
volatile uint8_t send_buf_cnt;
volatile uint8_t rcv_buf_cnt;
//------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
  micros *= (SystemCoreClock / 1000000) / 9;
  while (micros--) ;
}



uint8_t NRF24_ReadReg(uint8_t addr)
{
  uint8_t dt=0, cmd;

  CS_ON;
  HAL_SPI_TransmitReceive(&hspi2,&addr,&dt, 1,1000);
  if(addr != STATUS)//если адрес равен адрес регистра статус то и возварщаем его состояние
  {
    cmd=0xFF;
    HAL_SPI_TransmitReceive(&hspi2,&cmd, &dt,1,1000);
  }
  CS_OFF;
  return dt;
}

void NRF24_WriteReg(uint8_t addr, uint8_t dt)
{
  addr |= W_REGISTER;//включим бит записи в адрес

  CS_ON;
  HAL_SPI_Transmit(&hspi2,&addr,1,1000);//отправим адрес в шину
  HAL_SPI_Transmit(&hspi2,&dt,  1,1000);//отправим данные в шину
  CS_OFF;
}

void NRF24_Read_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  CS_ON;
  HAL_SPI_Transmit(&hspi2,&addr, 1, 1000);//отправим адрес в шину
  HAL_SPI_Receive(&hspi2, pBuf, bytes,1000);//отправим данные в буфер
  CS_OFF;
}

void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  addr |= W_REGISTER;//включим бит записи в адрес
  CS_ON;
  HAL_SPI_Transmit(&hspi2,&addr,1,1000);//отправим адрес в шину
  DelayMicro(1);
  HAL_SPI_Transmit(&hspi2,pBuf,bytes,1000);//отправим данные в буфер
  CS_OFF;
}


void NRF24_FlushRX(void)

{
  uint8_t dt[1] = {FLUSH_RX};
  CS_ON;
  HAL_SPI_Transmit(&hspi2,dt,1,1000);
  DelayMicro(1);
  CS_OFF;
}

void NRF24_FlushTX(void)
{
  uint8_t dt[1] = {FLUSH_TX};
  CS_ON;
  HAL_SPI_Transmit(&hspi2, dt, 1, 1000);
  DelayMicro(1);
  CS_OFF;
}

void NRF24L01_RX_Mode(void)
{
  uint8_t regval=0x00;
  regval = NRF24_ReadReg(CONFIG);
  //разбудим модуль и переведём его в режим приёмника, включив биты PWR_UP и PRIM_RX
	printf("regval %x\n", regval);
  regval |= (1<<PWR_UP)|(1<<PRIM_RX);
	printf("regval %x\n", regval);
  NRF24_WriteReg(CONFIG,regval);
	regval = NRF24_ReadReg(STATUS);
	printf("regval %x\n", regval);
	NRF24_WriteReg(STATUS,regval);
  CE_SET;
  DelayMicro(150); //Задержка минимум 130 мкс
	// Flush buffers
  NRF24_FlushRX();
  NRF24_FlushTX();
}

void NRF24_ToggleFeatures(void)
{
  uint8_t dt[1] = {ACTIVATE};
  CS_ON;
  HAL_SPI_Transmit(&hspi2,dt,1,1000);
  DelayMicro(1);
  dt[0] = 0x73;
  HAL_SPI_Transmit(&hspi2,dt,1,1000);
  CS_OFF;
}


void NRF24_ini(void)
{
	uint8_t regval=0x00;
  	CE_RESET;
  DelayMicro(5000);
	NRF24_WriteReg(CONFIG, 0x32); // Set PWR_UP bit, enable CRC(1 byte) &Prim_RX:0 (Transmitter)
	regval = NRF24_ReadReg(CONFIG);
	printf("regval %02X\n", regval);
  DelayMicro(5000);
	NRF24_WriteReg(EN_AA, 0x00); // Enable Pipe1
	NRF24_WriteReg(EN_RXADDR, 0x01); // Enable Pipe1
	NRF24_WriteReg(SETUP_AW,  0x01); // Setup address width=3 bytes
	NRF24_WriteReg(SETUP_RETR, 0x5F); // // 1500us, 15 retrans
	NRF24_ToggleFeatures();
	NRF24_WriteReg(FEATURE, 0);
	NRF24_WriteReg(DYNPD, 0);
	NRF24_WriteReg(STATUS, 0x70); //Reset flags for IRQ
	NRF24_WriteReg(RF_CH, 76); // частота 2476 MHz
	NRF24_WriteReg(RF_SETUP, 0x26); //TX_PWR:0dBm, Datarate:0.25 Mbps
	//NRF24_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
	NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);
	NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH); //Number of bytes in RX payload in data pipe 1
 //пока уходим в режим приёмника
  NRF24L01_RX_Mode();
  LED_OFF;
}
uint8_t status=0x01;
uint16_t state=0;
void Receive(void){
	
	while(state != 4){
		//printf("STATUS: 0x%02X\n",status);
		switch (state)
    {
    	case 0:
			  LED_TGL;
				send_buf[0] = 0;
				send_buf[1] = STATUS;
				CS_ON;
				hspi2.Instance->DR = send_buf[1];
				send_buf_cnt = 1;
				rcv_buf_cnt = 2;
				__HAL_SPI_ENABLE_IT(&hspi2, SPI_IT_RXNE | SPI_IT_TXE);
			  //status = NRF24_ReadReg(STATUS);
				printf("check status ");
				state = 1;
				htim4.Instance->CNT = 1;
				__HAL_TIM_ENABLE(&htim4);
    		break;
    	case 1:
			  if(send_delay && (rcv_buf[1] & 0x40)){
				  printf("0x%02X ", rcv_buf[1]);
					send_delay = 0;
					send_buf[0] = 0;
					send_buf[1] = RD_RX_PLOAD;
					CS_ON;
					hspi2.Instance->DR = send_buf[1];
					send_buf_cnt = 1;
					rcv_buf_cnt = 3;
					__HAL_SPI_ENABLE_IT(&hspi2, SPI_IT_RXNE | SPI_IT_TXE);
						//NRF24_Read_Buf(RD_RX_PLOAD,RX_BUF, TX_PLOAD_WIDTH);
					//printf("STATUS: 0x%02X\n",RX_BUF[0]);
					state = 2;
					htim4.Instance->CNT = 1;
					__HAL_TIM_ENABLE(&htim4);
				}
    		break;
			case 2:
				if(send_delay){
					send_delay = 0;
					printf("rcv: 0x%02X\n",rcv_buf[1]);
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
				if(send_delay)
					state = 4;
    	default:
    		break;
    }
		
//		 case 0:
//			
//			
//		if(status & 0x40)
//		{
//			NRF24_Read_Buf(RD_RX_PLOAD,RX_BUF, TX_PLOAD_WIDTH);
//			printf("STATUS: 0x%02X\n",RX_BUF[0]);
//			//dt = *(int16_t*)RX_BUF;
//			//dt = *(int16_t*)(RX_BUF+2);
//			
//		}
	}
	state = 0;
}
