#include "messages.h"
#include "gpio.h"
#define NO_MSG_RCV 0
#define IS_MSG_RCV 1
#define CTRL_MSG   1

extern unsigned long u_auth_key[4];
extern unsigned long encrypted_num[2];  /* Used to save the encrypted random number */
extern unsigned char random_num[8];
extern unsigned long * p_rand_num;  


extern U16 ssc_txFrame_buf[8];// = {0,0,0,0,0xABCD,0x5A5A,0,0};
U16 ssc_idle_frame[8] = {0, 0, 0xABCD, 0x5A5A, 0xABCD, 0x5A5A, 0, 0};
U16 ssc_tx_msg_length;
U16 ssc_tx_msg_counter;
U16 ssc_txData_buf[510] = {0};
U16 ssc_sndData_buf[130] = {0xABCD,0};
U16 ssc_rcv_ctrl_msg[180]={0};
U16 AUTH_KEY_REQ[] =     {0xABCD,0x400E,0xFFF6,0x0004,0x0000,0x0006,0x0000,0x0000,0x0000,0x00BA};
U16 XNL_DEVICE_CONN_REQUEST[] = {0xABCD,0x401A,0x189D,0x0006,0x0000,0x0006,0xFFFF,0x0000, // after encipher
                                 0x000C,0x0000,0x0702,0x0D1A,0x2F64,0x24E7,0x7EE5,0x00BA };
U16 XNL_DATA_MSG_ACK[] =   {0xABCD,0x400E,0xFDE8,0x000C,0x0100,0x0006,0x0001,0x0105,0x0000,0x00BA}; 
U16 XNL_DATA_MSG_REPLY[] = {0xABCD,0x4013,0xFDE8,0x000B,0x0100,0x0006,0x0001,0x0105,
	//                          0      1       2      3       4      5     6      7
0x0005,  // length 8
0x841C, // opcode  9 
0x00,   // result  10
0x00,   // function 11
0x00,	  // session id 12
0x00,   // 	
0x00BA,
0x0000 };


U16 XCMP_DEVICE_INIT_STATUS[] = {0xABCD,0x4019, 0x49E2,0x000B, 0x0100,0x0006, 0x0001,0x0105,
                                 0x000B,
                                 0xB400, // device initialization status
                                 0x0200,0x0005, //xcmp version
                                 0x0007,     //  device init status / device type
                                 0x0000,     // device status (power up success)
                                 //                                      0x0400,     // device descriptor size / xcmp dev family
                                 //	                                    0x0002,
                                 0x0000,	//  3th party  appl
                                 0x00BA,0x0000};

U8  rcv_msg_state; // 1- being receive
U8  tract_id_my;
U8  xcmp_dev_num;		
U8  rcv_msg_state; // 1- being receive
U16  rcv_msg_counter; // счетчик принятых данных
U16  ssc_rcv_ctrl_msg_len; // длинна сообщения
U8  auth_key_8t[8];
U16 tract_id_my_hi;																			
U16 tract_id; // ид  транзакции когда принимаем ОхОВ
U16 tract_id_reply;// for reply
U16 xnl_flags; // 0-7 for data message
U16 ob_adress;// новый адрес борды  когда принимаем Ох07
U16 mask;
U16 cnt;                                      
U8 first_time;
U8 is_connect;
volatile U8  idle_frame_tx = 1;
U8 conn_establish_state;
//U16 ssc_rcv_ctrl_msg[180]={0};
extern volatile U16 ssc_rcv_buf[8];
void conn_process(void)
{

  switch(rcv_msg_state){// или принимаем новое сообщение или продолжаем прием

  case NO_MSG_RCV:  // начало приема сообщения
    mask = ssc_rcv_buf[3] & 0xF000;
    if(mask == 0x4000 ){
      cnt++;
      ssc_rcv_ctrl_msg_len = ssc_rcv_buf[3] & 0xff;//  ssc_rcv_buf[2] = 0xABCD
      if(ssc_rcv_ctrl_msg_len % 2)
        ssc_rcv_ctrl_msg_len++;
      ssc_rcv_ctrl_msg_len += 2;
      rcv_msg_counter = 0;
      rcv_msg_state = IS_MSG_RCV;// get message
    }
    break; //case NO_MSG_RCV

  case IS_MSG_RCV:	//
    memcpy((ssc_rcv_ctrl_msg + (rcv_msg_counter>>1)) , (ssc_rcv_buf + 2), 4);
    rcv_msg_counter += 4;
    if(rcv_msg_counter >= ssc_rcv_ctrl_msg_len){ //получили все сообщение
      rcv_msg_state = NO_MSG_RCV;
      switch (ssc_rcv_ctrl_msg[1])
      {
      case 2:
        ssc_tx_msg_length = 10;
        memcpy(ssc_txData_buf , AUTH_KEY_REQ , 20 );
        ssc_tx_msg_counter = 0;
        idle_frame_tx = 0;
        break;

      case 5:
        auth_key_8t[0]= ssc_rcv_ctrl_msg[8] >> 8;// переводим в байтовый массив
        auth_key_8t[1]= ssc_rcv_ctrl_msg[8] & 0xFF;
        auth_key_8t[2]= ssc_rcv_ctrl_msg[9] >> 8;
        auth_key_8t[3]= ssc_rcv_ctrl_msg[9] & 0xFF;
        auth_key_8t[4]= ssc_rcv_ctrl_msg[10] >> 8;
        auth_key_8t[5]= ssc_rcv_ctrl_msg[10] & 0xFF;
        auth_key_8t[6]= ssc_rcv_ctrl_msg[11] >> 8;
        auth_key_8t[7]= ssc_rcv_ctrl_msg[11] & 0xFF;
        memcpy(random_num, auth_key_8t, 8);
        pre_enc();
        encipher(p_rand_num, encrypted_num, u_auth_key);
        post_enc();
        U16 temp = XNL_DEVICE_CONN_REQUEST[11];
        XNL_DEVICE_CONN_REQUEST[11] = XNL_DEVICE_CONN_REQUEST[12];
        XNL_DEVICE_CONN_REQUEST[12] = temp;
        temp = XNL_DEVICE_CONN_REQUEST[13];
        XNL_DEVICE_CONN_REQUEST[13] = XNL_DEVICE_CONN_REQUEST[14];
        XNL_DEVICE_CONN_REQUEST[14] = temp;
        get_check_sum(XNL_DEVICE_CONN_REQUEST);
        ssc_tx_msg_length = 16;
        memcpy((ssc_txData_buf) , (XNL_DEVICE_CONN_REQUEST  ), 32 );
        ssc_tx_msg_counter = 0;
        idle_frame_tx = 0;
        break;
      case 7:
        tract_id_my_hi = (ssc_rcv_ctrl_msg[7] << 8);
        ob_adress = ssc_rcv_ctrl_msg[8];
        conn_establish_state = 5;// ждем приема Ох09
        break;

      case 9:
        if(conn_establish_state == 5){
          conn_establish_state = 7;// ждем приема Ох0В
          if(ssc_rcv_ctrl_msg[7] == 2)
            xcmp_dev_num = ssc_rcv_ctrl_msg[11] >> 8;
          else if(ssc_rcv_ctrl_msg[7] == 3)
            xcmp_dev_num = ssc_rcv_ctrl_msg[13] & 0xFF;
        }
        break;

      case 0x0B:
        if(conn_establish_state == 7){
          XCMP_DEVICE_INIT_STATUS[10] = ssc_rcv_ctrl_msg[8];
          XCMP_DEVICE_INIT_STATUS[11] = ssc_rcv_ctrl_msg[9];
          XNL_DATA_MSG_ACK[4] = xnl_flags = ssc_rcv_ctrl_msg[2];
          XNL_DATA_MSG_ACK[7] = tract_id = ssc_rcv_ctrl_msg[5];
          XNL_DATA_MSG_REPLY[6] = XNL_DATA_MSG_ACK[6] = ob_adress;
          get_check_sum(XNL_DATA_MSG_ACK);
          memcpy(ssc_txData_buf , XNL_DATA_MSG_ACK,  20 );
          XCMP_DEVICE_INIT_STATUS[4] = ++xnl_flags;// надо прибавлять или не надо?????
          XCMP_DEVICE_INIT_STATUS[6] = ob_adress;
          XCMP_DEVICE_INIT_STATUS[7] = tract_id_my_hi + tract_id_my;//_hi | ((tract_id) & 0xFF);//tract_id;
          get_check_sum(XCMP_DEVICE_INIT_STATUS);
          memcpy((ssc_txData_buf + 10) , XCMP_DEVICE_INIT_STATUS, 36 );
          ssc_tx_msg_length = 28;//10+18
          ssc_tx_msg_counter = 0;
          conn_establish_state = 12;
          idle_frame_tx = 0;
        }
        else if(conn_establish_state == 13){
          XNL_DATA_MSG_ACK[4] = xnl_flags = ssc_rcv_ctrl_msg[2];
          XNL_DATA_MSG_ACK[7] = tract_id = ssc_rcv_ctrl_msg[5];
          get_check_sum(XNL_DATA_MSG_ACK);
          ssc_tx_msg_length = 10;
          memcpy((ssc_txData_buf) , (XNL_DATA_MSG_ACK), 20 );
          ssc_tx_msg_counter = 0;
          conn_establish_state = 16;
          idle_frame_tx = 0;
        }
        else if(conn_establish_state == 16){
          conn_establish_state = 17;
          XNL_DATA_MSG_ACK[4] = xnl_flags = ssc_rcv_ctrl_msg[2];
          XNL_DATA_MSG_ACK[7] = tract_id = ssc_rcv_ctrl_msg[5];
          get_check_sum(XNL_DATA_MSG_ACK);
          ssc_tx_msg_length = 10;
          memcpy((ssc_txData_buf) , (XNL_DATA_MSG_ACK), 20 );
          ssc_tx_msg_counter = 0;
					idle_frame_tx = 0;
//  					if(ssc_rcv_ctrl_msg[9] == 0xB407)
//  					{
          is_connect = 1; // connect!!!!!!!!!!!!!!!
						//USART_EnableIt(USART1,US_IER_RXRDY );
 					//memcpy(uart1_rx_buf, AUD_ROUTE, 15);
  				//uart1_rx_msg_len = 15;	
  				//uart1_rx_flag = CTRL_MSG;	
//					}

//          TC_Start( TC0, 1 ); // ~2 sec for start audio route  T0_ch1
					
        }
        break;

      case 0x0C:
        if(conn_establish_state == 12){
          conn_establish_state = 13; // еще надо принять ОхОВ
					GPIOA->ODR &= ~0x200;
					}
        break;

      default:
        break;
      }

    } // end if(rcv_msg_counter >= rcv_msg_length)
    break; //case IS_MSG_RCV:

  default:
    break;
  } // end switch(rcv_msg_state)
  

}