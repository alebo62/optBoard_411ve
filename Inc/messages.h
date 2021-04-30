#ifndef _MESSAGES_
#define _MESSAGES_
#include "stdint.h"

typedef uint16_t U16;
typedef uint8_t  U8;
//#include "board.h"

U16 SHUTDWNREPLAY[] = {0xABCD,0x4013,0x49E2,0x000B,0x0100,0x0006,0x0001,0x0105,
                       0x0005,0x840A,
                       0x0001,0x0000,
                       0x00BA,0x0000};

U16 SHUTDWNBRDCST[] = {0xABCD,0x4012,0x49E2,0x000B,0x0100,0x0006,0x0001,0x0105,
                       0x0004,0xB40A,
                       0x0100,0x00BA};


U16 ChannelZoneSelection[5] = {0x0007,0x040D,
                               0x8000,  	//Function + Query zone and channel
                               0x0000,  	// Query zone and channel + Zone Number or Step Size
                               0x0000}; 	// Channel Number or Step Size


U8 uart_ctrl_msg[64] = {0xC0,0};// передаются управляющие команды
U16 arc_message[] = {0xABCD,0x4017 ,0x49E2,0x000B, 0x0100,0x0006, 0x0001,0x0105,
                     0x0009,
                     0x0414, // xcmp arcontrol
                     0x0100, // update source / numrouting
                     0x010D,//10D,     //  numrouting / prespk auddata
                     0x0C00,//0x0C00,  // opt board input
                     0x0000,//0x0100,
                     0x00BA,0x0000 };

U16 mrc_message[] = {0xABCD,0x4017 ,0x49E2,0x000B, 0x0100,0x0006, 0x0001,0x0105,
                     0x0009,
                     0x0414, // xcmp arcontrol
                     0x0100, // update source / numrouting
                     0x010D,//10D,     //  numrouting / prespk auddata
                     0x0C00,//0x0C00,  // opt board input
                     0x0000,//0x0100,
                     0x00BA,0x0000 };
//U16 mrc_message[] = {0xABCD,0x4013 ,0x49E2,0x000B, 0x0100,0x0006, 0x0001,0x0105,
//																			0x0005,
//																			0x0414, // xcmp arcontrol 
//																			0x0000, // update source / numrouting
//																			0x0000,     //  numrouting / prespk auddata
//																			//0x0C00,//0C,     // opt board input
//																			//0x0100,				
//																			0x00BA,0x0000 };

U16 rad_info[] = { 0xABCD,0x4011,0x49E2,0x000B, 0x0100,0x0006, 0x0001,0x0105,
                   0x0003, // length  [8]
                   0x000E, // xcmp arcontrol  [9]
                   0x0800, //
                   0x00BA};



U16 dev_ctrl_message[32] = { 0xABCD,0x4013,0x49E2,0x000B, 0x0100,0x0006, 0x0001,0x0105,
                             0x0000, // length  [8]
                             0x0421, // xcmp arcontrol  [9]
                             0x0101, //
                             0x0100,     //  numrouting / prespk auddata
                             0x00BA,0x0000 };
U16 dev_ctrl_message_len;



U16 DEVMGMTREQ[] =   { 0xABCD,0x4013,0x49E2,0x000B, 0x0100,0x0006, 0x0001,0x0105,
                       0x0005,	0x0428, 0x0007, 0x0100, 0x00BA, 0x0000};

U16 pttKey[] =   {0xABCD,0x4012,0x0000,0x000B,0x0000,0x0006,0x0000,0x0004,
                  0x0004, // length
                  0x0415, // transmit req control
                  0x0100,
                  //0xFF00,	// key-up
                  0x00BA,0x0000};     // mode- voice

U16 ctrl_msg_ssi_uart[130];

U16 ctrl_msg_uart_ssi[400] = {0xABCD,0x0000,0x0000,0x000B,0x0000,0x0006};

U16 data_session_req[] = {0xABCD,0x401A,0xFFFF,0x000B, // запрос сессии
	                        0x01FF,0x0006,0x00FF,0xFFFF,
												  0x000C,0x041D,0x1050,0x0204,
                          0xC0A8,0x1E02,0x0FA4,0x00BA};
struct structSendMsg{ // len = 15
  U8 commandDataSessionReq[2];//0x041D
  U8 function;//01
  U8 rawData;//0x50
  U8 addressType;// 02 ip
  U8 addressLen; // 04
  U8 destIP[4];
  U8 destPort[2];
  U8 sessionID;
  U16 payloadLen;
  // + payload (ping , data, ...)
};
//U16 sendMdMsg{0xABCD,0x0000,0x0000,0x000B,0x0000,0x0006};
U16 sendMdMsg[] = {//0xABCD,0x4020,0x49E2,0x000B, 0x0100,0x0006, 0x0001,0x0105,
									 //0x0000, // length  [8]
									 0x041D, // xcmp arcontrol  [9]
									 
									 0x0150,
									 0x0204,
									 
									 0x0C00,//dest ip
									 0x0514,// dest ip
									 
									 0x0FA4,// dest port
									 0x0100,// session id
									 
									 0x02AA,// payload len
									 0xBB00,// payload 		
									 
									 0x00BA,
									 0x0000
									  };										

#endif
