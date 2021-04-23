#include <stdlib.h>
#include <string.h>
//extern unsigned char rcv_msg[];
extern unsigned char auth_key_8t[];
extern unsigned short XNL_DEVICE_CONN_REQUEST[];

#define htonl(A) ((((unsigned long)(A) & 0xff000000) >> 24) |  \
                 (((unsigned long)(A) & 0x00ff0000) >> 8) |  \
                 (((unsigned long)(A) & 0x0000ff00) << 8) |  \
                 (((unsigned long)(A) & 0x000000ff) << 24))

#define ntohl    htonl
#define ENCRYPTED_VALUE_POSITION_IN_CONN_REQ 11

//unsigned long u_auth_key[4] = {0x152C7E9D, 0x38BE41C7, 0x71E96CA4,0x6CAC1AFC };
//unsigned long const u_auth_key[4]={0x11223344,0x55667788,0x99AABBCC,0xDDEEFF00};
//unsigned long u_auth_key[4] = {0xC3381C1D, 0x1C8B6323, 0x45B0FCA2, 0xC4CA75D3};// firstkey 
unsigned long u_auth_key[4] = {0xC3381C1D,0x1C8B6323,0x45B0FCA2,0xC4CA75D3};
unsigned char random_num[8];// = {0x72, 0xA4, 0x1f, 0x8f, 0x63, 0x66, 0x01, 0xaa }; 
/* It is the raw data received from the XCMP message */
unsigned long encrypted_num[2] = {0};  /* Used to save the encrypted random number */
unsigned long * p_rand_num = (unsigned long *) random_num;  
/* The input parameter of the encipher function is unsigned long type, so assign
p_rand_num as a unsigned long pointer pointing to the random number array  */
unsigned char xnl_conn_req[24] = {0}; /* XNL connection request message */

void encipher(unsigned long *const v,
              unsigned long *const w,
              const unsigned long *const k)
{
    register unsigned long y=v[0], z=v[1], sum=0;
	  
    register unsigned long delta= 0x9E3779B9;
	//register unsigned long delta= 0x12345678;
    unsigned long a=k[0], b=k[1], c=k[2], d=k[3];
    register unsigned long n=32;

    while(n-->0)
    {
        sum += delta;
        y += ((z << 4) + a) ^ (z + sum) ^ ((z >> 5) + b);
        z += ((y << 4) + c) ^ (y + sum) ^ ((y >> 5) + d);
     }

    w[0]=y; w[1]=z;
}

void pre_enc(void)
{
* p_rand_num = ntohl(*p_rand_num); /* This is for the first 4 bytes switch */
*( p_rand_num + 1) = ntohl(*(p_rand_num + 1));  /* This is for the last 4 bytes switch */
/* The converted random number is 0x8f, 0x1f, 0xA4, 0x72, 0xaa, 0x01, 0x66,0x63 */
/* Invoke the encipher routine to encrypt the 8 byte random number */
}

/* The byte order of the returned value is 0x71, 0x2F, 0x41, 0xB9, 0xCF, 0x38,
0x60, 0xFB */
/* Convert the byte order of the encrypted value to big endian */
void post_enc(void)
{
//encrypted_num[0] = htonl(encrypted_num[0]);
//encrypted_num[1] = htonl(encrypted_num[1]);
/* The converted value is 0xB9, 0x41, 0x2F, 0x71, 0xFB, 0x60, 0x38, 0xCF */
/* copy the encrypted data into the XCMP reply message */
memcpy(&XNL_DEVICE_CONN_REQUEST[11], (unsigned short *)encrypted_num, 8);
}

// в массиве вычисляем контр. сумму и записываем ее в arr[2]
void get_check_sum(unsigned short arr[]){
	unsigned short sum = 0;
	unsigned char i = (arr[1] & 0xFF) - 2;// -crc
	
	if(i % 2)
		i++;
	i >>= 1; // for 16bit
	i += 3;	// begin from arr[3]
	
	for(unsigned char j = 3; j < i; j++)
		sum += arr[j];
	
	sum = ~sum;
	sum++;
	arr[2] = sum;
}
