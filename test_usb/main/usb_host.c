#include <stdint.h>
#include <stddef.h>
#include <sys/cdefs.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp8266/eagle_soc.h"
#include "esp8266/pin_mux_register.h"
#include "esp8266/gpio_struct.h"
#include "esp_clk.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include <sys/time.h>
int lt;
//~ #include "driver/gpio.h"
//~ #include "sdkconfig.h"
//~ #include "driver/timer.h"
//~ #include "soc/soc.h"
//~ #include "soc/rtc.h"
//~ #include <math.h>
//~ #include "esp_heap_caps.h"
/*******************************
*    warning!!!: any copy of this code or his part must include this: 
*  "The original was written by Dima Samsonov @ Israel sdima1357@gmail.com on 3/2021" *
*  Copyright (C) 2021  Dmitry Samsonov *
********************************/

#include "usb_host.h"

#define T_START 	0b00000001
#define T_ACK   	0b01001011
#define T_NACK  	0b01011010
#define T_SOF		0b10100101
#define T_SETUP	0b10110100
#define T_DATA0	0b11000011
#define T_DATA1	0b11010010
#define T_DATA2	0b11100001
#define T_OUT		0b10000111
#define T_IN		0b10010110

#define T_ERR		0b00111100
#define T_PRE		0b00111100
#define T_NYET	0b01101001
#define T_STALL	0b01111000

// local non std
#define T_NEED_ACK   0b01111011
#define T_CHK_ERR    0b01111111

#define USB_LS_K  0
#define USB_LS_J  1
#define USB_LS_S  2

//most counters- uint_8t :  so  prevents overflow...

#define DEF_BUFF_SIZE 0x100

// somethins short  like ACK 
#define SMALL_NO_DATA 36


///cpufreq (must be 240) /8 count = 30MHz  convinient number for measure 1.5MHz  of  low speed USB
//#pragma GCC optimize ("-O2")
inline uint8_t _getCycleCount8d4(void) {
  uint32_t ccount;
  __asm__ __volatile__("rsr %0,ccount":"=a" (ccount));
  return ccount>>2;
}
//#pragma GCC optimize ("-O2")
inline uint32_t _getCycleCount32(void) {
  uint32_t ccount;
  __asm__ __volatile__("rsr %0,ccount":"=a" (ccount));
  return ccount;
}


//timing calibrations which depends CPU_FREQ, calibrated in initPins()

int TRANSMIT_TIME_DELAY = 110;  //delay each bit transmit
int TIME_MULT 		  = 25;    //received time factor delta clocks* TIME_MULT/TIME_SCALE
int TM_OUT 			  = 64;    //receive time out no activity on bus
#define  TIME_SCALE (1024)

//#define TEST
#ifdef TEST
#define TOUT  1000
#else
#define TOUT  (TM_OUT)
#endif	



#define SET_I      {GPIO.enable_w1tc = RD_MASK; }
#define SET_O    { GPIO.enable_w1ts = RD_MASK; }

#define SE_J        { *snd[1][0] = (1 << DM_PIN);*snd[1][1] = (1 << DP_PIN); }
#define SE_0       { GPIO.out_w1tc= RD_MASK; }

//#define READ_BOTH_PINS (((GPIO.in&(0x3*(1<<DP_PIN)))>>(DP_PIN))<<8)
#define READ_BOTH_PINS ((GPIO.in&RD_MASK)>>RD_SHIFT)

//must be setup ech time with setPins
uint16_t DP_PIN;
uint16_t DM_PIN;

uint16_t DM_PIN_M; 
uint16_t DP_PIN_M; 
uint16_t BOTH_PINS; 

uint16_t M_ONE;
uint16_t P_ONE;
uint16_t RD_MASK;
uint16_t RD_SHIFT;
//end must be setup ech time with setPins


// temporary used insize lowlevel
uint8_t received_NRZI_buffer_bytesCnt;
uint16_t 	received_NRZI_buffer[DEF_BUFF_SIZE];

uint8_t transmit_bits_buffer_store_cnt;


//uint8_t transmit_bits_buffer_store[DEF_BUFF_SIZE];
// share same memory as received_NRZI_buffer
uint8_t*  transmit_bits_buffer_store = (uint8_t*)&received_NRZI_buffer[0];


uint8_t transmit_NRZI_buffer_cnt;
uint8_t  transmit_NRZI_buffer[DEF_BUFF_SIZE];

uint8_t decoded_receive_buffer_head;
uint8_t decoded_receive_buffer_tail;
uint8_t decoded_receive_buffer[DEF_BUFF_SIZE];
// end temporary used insize lowlevel

#if 0
#define TNOP1  { __asm__ __volatile__("   nop"); }
#define TNOP2  {TNOP1 TNOP1}
#define TNOP4  {TNOP2 TNOP2}
#define TNOP8  {TNOP4 TNOP4}
#define TNOP16  {TNOP8 TNOP8}
#define TNOP32  {TNOP16 TNOP16}
#define TNOP64  {TNOP32 TNOP32}


//__volatile__ void dummy0(){printf("dummy0");}
__volatile__ void cpuDelayB()
{
	TNOP64;
	TNOP64;
	TNOP64;
	TNOP64;
//	__asm__ __volatile__("   nop");

//	tick = DWT->CYCCNT + tick;
//	while((DWT->CYCCNT - tick)&0x80000000u);
}
//__volatile__ void dummy1(){printf("dummy1");}

//typedef void *pfnc(uint32_t);


volatile void (*delay_pntA)() = &cpuDelayB;

#define cpuDelay(x) {(*delay_pntA)();}
#if 0
void cpuDelay(uint32_t tick)
{
	//uint8_t* pnt = (uint8_t*)&cpuDelay;
	//void (*delay_pnt)() = (&cpuDelayB)+((256-tick)*2);
	(*delay_pntA)();
}
#endif
void setDelay(uint32_t tick)
{
	delay_pntA = (&cpuDelayB)+((256-tick)*2);
}
#endif
#if  1
#if 1
//#define MALLOC_CAP_EXEC         (1 << 0) 
void (*delay_pntA)() =NULL;
#define cpuDelay(x) {(*delay_pntA)();}
void setDelay(uint8_t ticks)
{
// opcodes of void test_delay() {__asm__ (" nop"); __asm__ (" nop"); __asm__ (" nop"); ...}
//3d f0 0d f0 00 // one  nop
//3d f0 3d f0 0d f0 00 // one  nop
int    MAX_DELAY_CODE_SIZE = 0x280;
uint8_t*     pntS;
	// it can't execute but can read & write
	if(!delay_pntA)
	{
		pntS = malloc(MAX_DELAY_CODE_SIZE);
	}
	else
	{
		pntS = heap_caps_realloc(delay_pntA, MAX_DELAY_CODE_SIZE, MALLOC_CAP_8BIT);
	}
	uint8_t* pnt = (uint8_t*)pntS;
	//put head of delay procedure
	//~ *pnt++ = 0x36;
	//~ *pnt++ = 0x41;
	//~ *pnt++ = 0; 
	for(int k=0;k<ticks;k++)
	{
		//put NOPs
		*pnt++ = 0x3d;
		*pnt++ = 0xf0;
	}
	//put tail of delay procedure
	*pnt++ = 0x0d;
	*pnt++ = 0xf0;
	*pnt++ = 0x00;
	*pnt++ = 0x00;
	// move it to executable memory segment
	// it can't  write  but can read & execute
	delay_pntA = heap_caps_realloc(pntS,MAX_DELAY_CODE_SIZE,MALLOC_CAP_EXEC);
	//delay_pntA = heap_caps_realloc(pntS,MAX_DELAY_CODE_SIZE,MALLOC_CAP_32BIT);
}
#else
void setDelay(uint32_t tick)
{

}
//#pragma GCC optimize ("-O2")
void cpuDelayS(uint32_t ticks)
{
	for(int k=0;k<ticks;k++)
	{
		__asm__ __volatile__("   nop"); 
	}
}
//#pragma GCC optimize ("-O2")
void cpuDelay(uint32_t tick)
{
	uint32_t stop =_getCycleCount32() + tick;
	//while(_getCycleCount32()<stop);
	while((_getCycleCount32() - stop)>0x8000000u);
}
#endif
#endif
void del1()
{
	__asm__ __volatile__("   nop"); 
}
void del2()
{
	__asm__ __volatile__("   nop"); 
	__asm__ __volatile__("   nop"); 
}
void del3()
{
	__asm__ __volatile__("   nop"); 
	__asm__ __volatile__("   nop"); 
	__asm__ __volatile__("   nop"); 
}


typedef __packed struct
{
	uint8_t cmd;
	uint8_t addr;
	uint8_t eop;

	uint8_t  dataCmd;
	uint8_t  bmRequestType;
	uint8_t  bmRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLen;
}Req;
enum    DeviceState 	{ NOT_ATTACHED,ATTACHED,POWERED,DEFAULT,ADDRESS,
					PARSE_CONFIG,PARSE_CONFIG1,PARSE_CONFIG2,PARSE_CONFIG3,
					POST_ATTACHED,RESET_COMPLETE,POWERED_COMPLETE,DEFAULT_COMPL} ;

enum  CallbackCmd {CB_CHECK,CB_RESET,CB_WAIT0,CB_POWER,CB_TICK,CB_2,CB_2Ack,CB_3,CB_4,CB_5,CB_6,CB_7,CB_8,CB_9,CB_WAIT1} ;

//Req rq;
typedef struct
{
int 			isValid;	
int 			selfNum;
int                    epCount;	
int 			cnt;
uint32_t 		DP;
uint32_t 		DM;
 enum CallbackCmd   	cb_Cmd;
 enum DeviceState    	fsm_state;
 		uint16_t      	wires_last_state;
sDevDesc 	desc;
sCfgDesc  	cfg; 
Req 			rq;

int 			counterNAck;
int 			counterAck;

uint8_t     	descrBuffer[DEF_BUFF_SIZE];
uint8_t     	descrBufferLen;

 int    bComplete;
 int    in_data_flip_flop;
int     	      cmdTimeOut;
uint32_t  	      ufPrintDesc;
int        	      numb_reps_errors_allowed;

uint8_t    acc_decoded_resp[DEF_BUFF_SIZE];
uint8_t    acc_decoded_resp_counter;
	
int          asckedReceiveBytes;
int 	       transmitL1Bytes;
uint8_t    transmitL1[DEF_BUFF_SIZE];

uint8_t   Resp0[DEF_BUFF_SIZE];
uint8_t   R0Bytes;
uint8_t   Resp1[DEF_BUFF_SIZE];
uint8_t   R1Bytes;

int     bAcked;	

} sUsbContStruct;

sUsbContStruct * current;






// received data from ep0,ep1!!!!
//uint8_t   current->Resp0[DEF_BUFF_SIZE];
//uint8_t   current->R0Bytes;
//uint8_t   current->Resp1[DEF_BUFF_SIZE];
//uint8_t   current->R1Bytes;



uint32_t *   snd[4][2]  = {
					{&GPIO.out_w1tc,&GPIO.out_w1ts},	
					{&GPIO.out_w1ts,&GPIO.out_w1tc},	
					{&GPIO.out_w1tc,&GPIO.out_w1tc},
					{&GPIO.out_w1tc,&GPIO.out_w1tc}
				  } ;

				  
#ifdef WR_SIMULTA	
uint32_t sndA[4]  = {0,0,0,0};
#endif
				  



void restart()
{
	transmit_NRZI_buffer_cnt = 0;
}

void decoded_receive_buffer_clear()
{
	decoded_receive_buffer_tail = decoded_receive_buffer_head;
}

void decoded_receive_buffer_put(uint8_t val)
{
	decoded_receive_buffer[decoded_receive_buffer_head] = val;
	decoded_receive_buffer_head++;
}

uint8_t decoded_receive_buffer_get()
{
	return decoded_receive_buffer[decoded_receive_buffer_tail++];
}

uint8_t decoded_receive_buffer_size()
{
	return (uint8_t )(decoded_receive_buffer_head-decoded_receive_buffer_tail);
}

uint8_t cal5()
{
	uint8_t   crcb;
	uint8_t   rem;

	crcb = 0b00101;
	rem =  0b11111;

	for(int k=16;k<transmit_bits_buffer_store_cnt;k++)
	{
		int rb = (rem>>4)&1;
		rem   =  (rem<<1)&0b11111;

		if(rb^(transmit_bits_buffer_store[k]&1))
		{
			rem ^= crcb;
		}
	}
	return (~rem)&0b11111;
}
uint32_t cal16()
{
	uint32_t   crcb;
	uint32_t   rem;

	crcb = 0b1000000000000101;
	rem =  0b1111111111111111;

	for(int k=16;k<transmit_bits_buffer_store_cnt;k++)
	{
		int rb = (rem>>15)&1;
		rem   =  (rem<<1)&0b1111111111111111;

		if(rb^(transmit_bits_buffer_store[k]&1))
		{
			rem ^= crcb;
		}
	}
	return (~rem)&0b1111111111111111;
}
 void seB(int bit)
{
	transmit_bits_buffer_store[transmit_bits_buffer_store_cnt++] = bit;
}

 void pu_MSB(uint16_t msg,int N)
{
	for(int k=0;k<N;k++)
	{
		seB(msg&(1<<(N-1-k))?1:0);
	}
}
 void pu_LSB(uint16_t msg,int N)
{
	for(int k=0;k<N;k++)
	{
		seB(msg&(1<<(k))?1:0);
	}
}



void repack()
{
	int last = USB_LS_J;
	int cntOnes = 0;
	transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;
	for(int k=0;k<transmit_bits_buffer_store_cnt;k++)
	{
		if(transmit_bits_buffer_store[k]==0)
		{
			if(last==USB_LS_J||last==USB_LS_S)
			{
				last = USB_LS_K;
			}
			else
			{
				last = USB_LS_J;
			}
			cntOnes = 0;
		}
		else if(transmit_bits_buffer_store[k]==1)
		{
			cntOnes++;
			if(cntOnes==6)
			{
				transmit_NRZI_buffer[transmit_NRZI_buffer_cnt] = last;
				transmit_NRZI_buffer_cnt++;
				if(last==USB_LS_J)
				{
					last = USB_LS_K;
				}
				else
				{
					last = USB_LS_J;
				}
				cntOnes = 0;
			}
			if(last==USB_LS_S)
			{
				last = USB_LS_J;
			}
		}
		transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = last;
	}

	transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_S;
	transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_S;

	transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;
	transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;
	transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;
	//transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;

	transmit_bits_buffer_store_cnt = 0;

}

uint8_t rev8(uint8_t j)
{
	uint8_t res = 0;
	for(int i=0;i<8;i++)
	{
		res<<=1;
		res|=(j>>i)&1;
	}
	return res;
}
uint16_t rev16(uint16_t j)
{
	uint16_t res = 0;
	for(int i=0;i<16;i++)
	{
		res<<=1;
		res|=(j>>i)&1;
	}
	return res;
}
#ifdef DEBUG_ALL
uint16_t debug_buff[0x100];
#endif

int parse_received_NRZI_buffer()
{

	if(!received_NRZI_buffer_bytesCnt) return 0;
	
	uint32_t   crcb;
	uint32_t   rem;

	crcb = 0b1000000000000101;
	rem =  0b1111111111111111;

	int  res = 0;
	int  cntOnes = 0;
	
	int terr  = 0;
	uint8_t  current_res = 0xfe;
	uint16_t prev = received_NRZI_buffer[0];
	int      start = -1;
	uint8_t  prev_smb = M_ONE;
#ifdef DEBUG_ALL
	debug_buff[0] = received_NRZI_buffer_bytesCnt;
	uint8_t rcnt = 1;
	debug_buff[received_NRZI_buffer_bytesCnt] = 0xff;
#endif
	for(int i = 1;i<received_NRZI_buffer_bytesCnt;i++)
	{
		//define 2.5
		uint16_t curr = (prev&0xff00) + (((received_NRZI_buffer[i] - prev))&0xff);
		prev          = received_NRZI_buffer[i];

		uint8_t smb = curr>>8;
		int tm = (curr&0xff);
		//debug_buff[i] = tm | (smb<<8);
		if( tm<2  || (smb == 0) )
		{
			//terr+=tm<4?tm : 4;
			terr+=tm;
		}
		else
		{
			//terr = 0;
			int delta = ((((curr+terr)&0xff))*TIME_MULT+TIME_SCALE/2)/TIME_SCALE;
			
			for(int k=0;k<delta;k++)
			{
				int incc = 1;
				{
					if(prev_smb!=smb)
					{
						if(cntOnes!=6)
						{
							current_res = current_res*2+0;
						}
						else
						{
							incc = 0;
						}
						cntOnes = 0;
					}
					else
					{
						current_res = current_res*2+1;
						cntOnes++;
					}
					if(start>=0)
					{
						start+=incc;
					}
					if(current_res==0x1 && start<0 )
					{
						start = 0;
					}
					if( (start&0x7) == 0 && incc)
					{
						if(start==8)
						{
							res = current_res;
						}
#ifdef DEBUG_ALL
						debug_buff[rcnt++] = current_res;
#endif
						decoded_receive_buffer_put(current_res);
						if(start>8)
						{
							for(int bt =0;bt<8;bt++)
							{
								int rb = (rem>>15)&1;
								rem   =  (rem<<1)&0b1111111111111111;
								if(rb^((current_res>>(7-bt))&1))
								{
									rem ^= crcb;
								}
							}
						}
					}

				}

				prev_smb = smb;
			}
			terr = 0;
		}
	}
#ifdef DEBUG_ALL
	debug_buff[rcnt++] = 0xff;
#endif
	rem &=0b1111111111111111;
	if (rem==0b1111111111111111)
	{
		return res;
	}
	if(rem==0x800d)
	{
		return  T_NEED_ACK;
	}
	else
	{
		return  T_CHK_ERR;
	}
}



//#define WR_SIMULTA 
//#pragma GCC optimize ("-O2")
#pragma GCC optimize ("-O2")
void sendOnly()
{
	uint8_t k;
	SET_O;
#ifdef WR_SIMULTA	
	uint32_t out_base = GPIO.out;
	sndA[0] = (out_base | DP) &~DM;
	sndA[1] = (out_base | DM) &~DP;
	sndA[2] = (out_base )&~(DP | DM);
	sndA[3] = out_base | (DM | DP);
#endif	
	for(k=0;k<transmit_NRZI_buffer_cnt;k++)
	{
		//usb_transmit_delay(10);
		cpuDelay(TRANSMIT_TIME_DELAY);
#ifdef WR_SIMULTA	
		GPIO.out = sndA[transmit_NRZI_buffer[k]];
#else		
		//*snd[transmit_NRZI_buffer[k]][0] = (1 << DM_PIN);
		//*snd[transmit_NRZI_buffer[k]][1] = (1 << DP_PIN);
		*snd[transmit_NRZI_buffer[k]][0] = DM_PIN_M;
		*snd[transmit_NRZI_buffer[k]][1] = DP_PIN_M;
#endif		
		
	}
	transmit_NRZI_buffer_cnt = 0;
	SET_I;
}
#pragma GCC optimize ("-Os")
#if 0
#pragma GCC optimize ("-O2")
// safety  option, but slower and works with high cpu freq >=160MHz .
void sendRecieveNParse()
{
	uint8_t locRec = 0;
	uint32_t val   = 0xff;//DM_GPIO_Port->IDR&(0x3*DP_Pin);
	uint32_t nval  = 0xff;
	int32_t act = TOUT;
//	portDISABLE_INTERRUPTS();
	sendOnly();
	while(act>0 && (val||nval) )
	{
		val  = nval;
		nval = READ_BOTH_PINS;
		received_NRZI_buffer[locRec] = _getCycleCount8d8() |  nval;
		if(val!=nval)
		{
			locRec++;
			act = TOUT;
		}
		else act--;

		//~ int flag =  val!=nval;
		//~ locRec  +=  flag;
		//~ act 	 = (flag)?TOUT:(act-1);
	}
//	portENABLE_INTERRUPTS();
	received_NRZI_buffer_bytesCnt = locRec;
}
#pragma GCC optimize ("-Os")
#else
// dangerous option, but faster and works with low cpu freq ~80MHz . If we have noise on bus it can overflow received_NRZI_buffer[]
int flagR3 = 1;
//#pragma GCC optimize ("-O2")
//#define WR_SIMULTAA 1
#pragma GCC optimize ("-O2")
void sendRecieveNParse()
		{
	register uint32_t R3;
	register uint32_t R4;// = READ_BOTH_PINS;
	register uint16_t *STORE = received_NRZI_buffer;
	//portENTER_CRITICAL();		
#if 1			
	sendOnly();
#else			
	uint8_t k;
	SET_O;
#ifdef WR_SIMULTA	
	uint32_t out_base = GPIO.out;
	sndA[0] = (out_base | DP) &~DM;
	sndA[1] = (out_base | DM) &~DP;
	sndA[2] = (out_base )&~(DP | DM);
	sndA[3] = out_base | (DM | DP);
#endif	
	for(k=0;k<transmit_NRZI_buffer_cnt;k++)
	{
		//usb_transmit_delay(10);
		cpuDelay(TRANSMIT_TIME_DELAY);
#ifdef WR_SIMULTA	
		GPIO.out = sndA[transmit_NRZI_buffer[k]];
#else		
		//*snd[transmit_NRZI_buffer[k]][0] = (1 << DM_PIN);
		//*snd[transmit_NRZI_buffer[k]][1] = (1 << DP_PIN);
		*snd[transmit_NRZI_buffer[k]][0] = DM_PIN_M;
		*snd[transmit_NRZI_buffer[k]][1] = DP_PIN_M;
#endif		
		
	}
	SET_I;
	transmit_NRZI_buffer_cnt = 0;
#endif	
//
//			sendOnly();
START:
	R4 = READ_BOTH_PINS;
	*STORE = R4 | _getCycleCount8d4();
	STORE++;
	R3 = R4;
	//R4 = READ_BOTH_PINS;
	//if(R4!=R3)  goto START;
	if( R3 )
			{
		for(int k=0;k<TOUT;k++)
			{
			R4   = READ_BOTH_PINS;
			if(R4!=R3)  goto START;
			}
	}
	//__enable_irq();
	//portEXIT_CRITICAL();
	received_NRZI_buffer_bytesCnt = STORE-received_NRZI_buffer;
	transmit_NRZI_buffer_cnt = 0;
	flagR3 = R3;
}
#endif

#pragma GCC optimize ("-Os")

int sendRecieve()
{
	sendRecieveNParse();
	return parse_received_NRZI_buffer();
}

void SOF()
{
#if 1
	if(1)
	{
		//reB();
		repack();
	}
	//	else
	//	{
	//static int frameN = 0;
	//		frameN++;
	//		reB();
	//		pu_MSB(T_START,8);
	//		pu_MSB(T_SOF,8);// ack
	//		pu_LSB(frameN,11);
	//		pu_MSB(cal5(),5);
	//		repack();
	//	}
	sendOnly();
#endif
}


void pu_Addr(uint8_t cmd,uint8_t addr,uint8_t eop)
{
	//reB();
	pu_MSB(T_START,8);
	pu_MSB(cmd,8);//setup
	pu_LSB(addr,7);
	pu_LSB(eop,4);
	pu_MSB(cal5(),5);
	repack();
}

void pu_ShortCmd(uint8_t cmd)
{
	//reB();
	pu_MSB(T_START,8);
	pu_MSB(cmd,8);//setup
	//pu_MSB(cal16(),16);
	pu_MSB(0,16);
	repack();
}

void pu_Cmd(uint8_t cmd,uint8_t bmRequestType, uint8_t bmRequest,uint16_t wValue,uint16_t wIndex,uint16_t wLen)
{
	//reB();
	pu_MSB(T_START,8);
	pu_MSB(cmd,8);//setup
	pu_LSB(bmRequestType,8);
	pu_LSB(bmRequest,8);
	pu_LSB(wValue,16);
	pu_LSB(wIndex,16);
	pu_LSB(wLen,16);
	pu_MSB(cal16(),16);
	repack();
}

uint8_t ACK_BUFF[0x20];
int    ACK_BUFF_CNT = 0;
//#pragma GCC optimize ("-O2")
void ACK()
{
	if(ACK_BUFF_CNT==0)
	{
		//reB();
		//repack();
		//reB();
		transmit_NRZI_buffer_cnt =0;
		pu_MSB(T_START,8);
		pu_MSB(T_ACK,8);// ack
		repack();
		memcpy(ACK_BUFF,transmit_NRZI_buffer,transmit_NRZI_buffer_cnt);
		ACK_BUFF_CNT = transmit_NRZI_buffer_cnt;
		sendOnly();
		transmit_NRZI_buffer_cnt =0;
	}
	else
	{

		uint8_t k;
		SET_O;
	#ifdef WR_SIMULTA	
		uint32_t out_base = GPIO.out;
		sndA[0] = (out_base | DP) &~DM;
		sndA[1] = (out_base | DM) &~DP;
		sndA[2] = (out_base )&~(DP | DM);
		sndA[3] = out_base | (DM | DP);
	#endif	
		*snd[ACK_BUFF[0]][0] = DM_PIN_M;
		*snd[ACK_BUFF[0]][1] = DP_PIN_M;
		for(k=1;k<ACK_BUFF_CNT;k++)
		{
			//usb_transmit_delay(10);
			cpuDelay(TRANSMIT_TIME_DELAY);
	#ifdef WR_SIMULTA	
			GPIO.out = sndA[transmit_NRZI_buffer[k]];
	#else		
			//*snd[transmit_NRZI_buffer[k]][0] = (1 << DM_PIN);
			//*snd[transmit_NRZI_buffer[k]][1] = (1 << DP_PIN);
			*snd[ACK_BUFF[k]][0] = DM_PIN_M;
			*snd[ACK_BUFF[k]][1] = DP_PIN_M;
	#endif		
			
		}
		SET_I;
		transmit_NRZI_buffer_cnt =0;
	}
}

//enum  CallbackCmd {CB_CHECK,CB_RESET,CB_WAIT0,CB_POWER,CB_TICK,CB_2,CB_3,CB_4,CB_5,CB_6,CB_7,CB_8,CB_9,CB_WAIT1} ;
//char*  CallbackCmdCtr[] = {"CB_CHECK","CB_RESET","CB_WAIT0","CB_POWER","CB_TICK","CB_2","CB_3","CB_4","CB_5","CB_6","CB_7","CB_8","CB_9","CB_WAIT1"} ;
//#pragma GCC optimize ("-O2")
#pragma GCC optimize ("-Os")
void timerCallBack()
{
	lt = _getCycleCount32();
	decoded_receive_buffer_clear();
	
	if(current->cb_Cmd==CB_CHECK)
	{
		SET_O;
		SE_0;
		SET_I;
		SE_J;
		cpuDelay(TRANSMIT_TIME_DELAY);
		cpuDelay(TRANSMIT_TIME_DELAY);
		cpuDelay(TRANSMIT_TIME_DELAY);
		cpuDelay(TRANSMIT_TIME_DELAY);
		cpuDelay(TRANSMIT_TIME_DELAY);
		cpuDelay(TRANSMIT_TIME_DELAY);
		cpuDelay(TRANSMIT_TIME_DELAY);
		cpuDelay(TRANSMIT_TIME_DELAY);
		cpuDelay(TRANSMIT_TIME_DELAY);
		cpuDelay(TRANSMIT_TIME_DELAY);
		current->wires_last_state = READ_BOTH_PINS>>8;
		if(current->wires_last_state==M_ONE)
		{
			// low speed
		}
		else if(current->wires_last_state==P_ONE)
		{
			//high speed
		}
		else if(current->wires_last_state==0x00)
		{
			// not connected
		}
		else if(current->wires_last_state== (M_ONE + P_ONE) )
		{
			//????
		}
		current->bComplete = 1;
	}
	else if (current->cb_Cmd==CB_RESET)
	{
		SOF();
		//sendRecieveNParse();
		cpuDelay(TRANSMIT_TIME_DELAY);
		
		SET_O;
		SE_0;
		current->cmdTimeOut  = 	31;
		current->cb_Cmd  	     = 	CB_WAIT0;
	}
	else if (current->cb_Cmd==CB_WAIT0)
	{
		if(current->cmdTimeOut>0)
		{
			current->cmdTimeOut--;
		}
		else
		{
			//sendRecieveNParse();
			current->bComplete     = 1;
		}
	}
	else if (current->cb_Cmd==CB_WAIT1)
	{
		SOF();
		if(current->cmdTimeOut>0)
		{
			current->cmdTimeOut--;
		}
		else
		{
			SET_O;
			SE_0;
			SET_I;
			SE_J;
			SET_I;
			cpuDelay(TRANSMIT_TIME_DELAY);
			cpuDelay(TRANSMIT_TIME_DELAY);
			cpuDelay(TRANSMIT_TIME_DELAY);
			cpuDelay(TRANSMIT_TIME_DELAY);
			cpuDelay(TRANSMIT_TIME_DELAY);
			cpuDelay(TRANSMIT_TIME_DELAY);
			cpuDelay(TRANSMIT_TIME_DELAY);
			cpuDelay(TRANSMIT_TIME_DELAY);
			cpuDelay(TRANSMIT_TIME_DELAY);
			cpuDelay(TRANSMIT_TIME_DELAY);
			current->wires_last_state = READ_BOTH_PINS>>8;
			current->bComplete     = 1;
		}
	}
	else if (current->cb_Cmd==CB_POWER)
	{

		// for TEST
#ifdef TEST
		SOF();
		sendRecieve();
		SOF();
		SOF();
#else
		SET_O;
		SE_J;
		SET_I;
		current->cmdTimeOut  = 	 2;
		current->cb_Cmd  = CB_WAIT1;
#endif		
	}
	else if (current->cb_Cmd==CB_TICK)
	{
		SOF();
		current->bComplete     =         1;
	}
	else if(current->cb_Cmd==CB_3)
	{
			 SOF();
			 pu_Addr(current->rq.cmd,current->rq.addr,current->rq.eop);
			 pu_Cmd(current->rq.dataCmd, current->rq.bmRequestType, current->rq.bmRequest,current->rq.wValue, current->rq.wIndex, current->rq.wLen);
			 int res = sendRecieve();
			 if(res==T_ACK)
			 {
				 current->cb_Cmd=CB_4;
				 current->numb_reps_errors_allowed = 8;
				 return ;
			 }
			 else
			 {
				 current->numb_reps_errors_allowed--;
				 if(current->numb_reps_errors_allowed>0)
				 {
					 return ;
				 }
				 else
				 {
					current->cb_Cmd=CB_TICK;
					current->bComplete = 1;
				 }
			 }
	}
	else if(current->cb_Cmd==CB_4)
	{
		 SOF();
		 pu_Addr(T_OUT,current->rq.addr,current->rq.eop);
		 //reB();
		 pu_MSB(T_START,8);
		 pu_MSB(T_DATA1,8);//setup
		 for(int k=0;k<current->transmitL1Bytes;k++)
		 {
			 pu_LSB(current->transmitL1[k],8);
		 }
		 pu_MSB(cal16(),16);
		 repack();
		 sendRecieveNParse();
		 pu_Addr(T_IN,current->rq.addr,current->rq.eop);
				//setup
		 sendRecieveNParse();
		 ACK();
		current->cb_Cmd=CB_TICK;
		 current->bComplete = 1;
	}
	else if(current->cb_Cmd==CB_5)
	{
		 SOF();
		 pu_Addr(current->rq.cmd,current->rq.addr,current->rq.eop);
		 pu_Cmd(current->rq.dataCmd, current->rq.bmRequestType, current->rq.bmRequest,current->rq.wValue, current->rq.wIndex, current->rq.wLen);
		sendRecieveNParse();
		 //int res = sendRecieve(current->asckedReceiveBytes>8?8:current->asckedReceiveBytes);
		int res = parse_received_NRZI_buffer();
		if(res==T_ACK)
		{
			current->cb_Cmd = CB_6;
			current->in_data_flip_flop = 1;
			current->numb_reps_errors_allowed = 4;
			current->counterAck ++;
			return ;
		 }
		 else
		 {
			//SOF();
			 current->counterNAck ++;
			 current->numb_reps_errors_allowed--;
			 if(current->numb_reps_errors_allowed>0)
			 {
				// current->cb_Cmd = CB_TICK;
				 current->acc_decoded_resp_counter = 0;
				 return ;
			 }
			 else
			 {
				 current->cb_Cmd = CB_TICK;
				 current->bComplete       =  1;
			 }
		 }
	}
	else if(current->cb_Cmd==CB_6)
	{
		SOF();
		pu_Addr(T_IN,current->rq.addr,current->rq.eop);
		//setup
	        sendRecieveNParse();
		// if receive something ??
		if(current->asckedReceiveBytes==0 && current->acc_decoded_resp_counter==0 && received_NRZI_buffer_bytesCnt<SMALL_NO_DATA &&  received_NRZI_buffer_bytesCnt >SMALL_NO_DATA/4 )
	        {
	        		ACK();
	        		//printf("received_NRZI_buffer_bytesCnt=%d!!!\n",received_NRZI_buffer_bytesCnt);
				current->cb_Cmd = CB_TICK;
				current->bAcked = 1;
				current->bComplete       =  1;
	        		return ;
	        }
		int res = parse_received_NRZI_buffer();
		if(res == T_NEED_ACK)
		{
			//SOF();
			if(decoded_receive_buffer_size()>2)
			{
				
				decoded_receive_buffer_get();
				uint8_t sval = decoded_receive_buffer_get();
				if((current->in_data_flip_flop&1)==1)
				{
					if(sval==T_DATA1)
					{
						
					}
					else
					{
						current->cb_Cmd = CB_7;
						return ;
					}
				}
				else
				{
					if(sval==T_DATA0)
					{
						
					}
					else
					{
						current->cb_Cmd = CB_7;
						return ;
					}
				}
				current->in_data_flip_flop++;	
				int bytes =decoded_receive_buffer_size()-2;
				for(int kk=0;kk<bytes;kk++)
				{
					current->acc_decoded_resp[current->acc_decoded_resp_counter] = rev8(decoded_receive_buffer_get());
					current->acc_decoded_resp_counter++;
					current->asckedReceiveBytes--;
				}
				//while(decoded_receive_buffer_size()) {decoded_receive_buffer_get();}
				if(bytes<=0)
				{
					//printf("zero!!\n");
					current->acc_decoded_resp_counter = 0;
					current->asckedReceiveBytes = 0;
					current->cb_Cmd = CB_TICK;
					current->bComplete = 1;
				}
				else
				{
					current->cb_Cmd = CB_7;
					return ;
				}
			}
			else
			{
				current->acc_decoded_resp_counter = 0;
				current->asckedReceiveBytes = 0;
				current->cb_Cmd = CB_TICK;
				current->bComplete = 1;
				return ;
			}
		}
		else
		{
			current->numb_reps_errors_allowed--;
			if(current->numb_reps_errors_allowed>0)
			{
				return ; 
			}
			else
			{
				current->cb_Cmd = CB_TICK;
				current->bComplete = 1;
			}
		}
		
	}
	else if(current->cb_Cmd==CB_7)
	{
		SOF();
		pu_Addr(T_IN,current->rq.addr,current->rq.eop);
				//setup
	        sendRecieveNParse();
		ACK();
		if(current->asckedReceiveBytes>0)
		{
			current->cb_Cmd = CB_6;
			return ;
		}
		current->cb_Cmd = CB_8;
	}
	else if(current->cb_Cmd==CB_8)
	{
		SOF();
		pu_Addr(T_OUT,current->rq.addr,current->rq.eop);
		pu_ShortCmd(T_DATA1);
		sendOnly();
		current->cb_Cmd = CB_TICK;
		current->bComplete = 1;
	}
	else if(current->cb_Cmd==CB_2Ack)
	{
		SOF();
		pu_Addr(T_IN,current->rq.addr,current->rq.eop);
				//setup
	    sendRecieveNParse();
        if(received_NRZI_buffer_bytesCnt<SMALL_NO_DATA/2)
        {
                	// no data , seems NAK or something like this
			current->cb_Cmd = CB_TICK;
                	current->bComplete = 1;
					//printf("received_NRZI_buffer_bytesCnt = %d\n",prec);
            return ;
         }
                //ACK();
         ACK();
		current->cb_Cmd = CB_TICK;
		current->bComplete = 1;
	}
	else if(current->cb_Cmd==CB_2)
	{
		SOF();
		pu_Addr(T_IN,current->rq.addr,current->rq.eop);
				//setup
	        sendRecieveNParse();
                if(received_NRZI_buffer_bytesCnt<SMALL_NO_DATA/2)
                {
                	// no data , seems NAK or something like this
			current->cb_Cmd = CB_TICK;
                	current->bComplete = 1;
					//printf("received_NRZI_buffer_bytesCnt = %d\n",prec);
                	return ;
                }
                //ACK();
                //ACK();
		int res = parse_received_NRZI_buffer();
		if(res==T_NEED_ACK)
		{
			if(decoded_receive_buffer_size()>2)
			{
				decoded_receive_buffer_get();
				decoded_receive_buffer_get();
				int bytes =decoded_receive_buffer_size()-2;
				for(int kk=0;kk<bytes;kk++)
				{
					current->acc_decoded_resp[current->acc_decoded_resp_counter] = rev8(decoded_receive_buffer_get());
					current->acc_decoded_resp_counter++;
					current->asckedReceiveBytes--;
				}
			}
			current->asckedReceiveBytes = 0;
			current->cb_Cmd=CB_2Ack;
			return ;
		}
		else
		{
			current->numb_reps_errors_allowed--;
			if(current->numb_reps_errors_allowed>0)
			{
				return ;
		}
			else
			{
				current->cb_Cmd = CB_TICK;
				current->bComplete = 1;
			}
		}
		current->cb_Cmd = CB_TICK;
		current->bComplete = 1;
		current->asckedReceiveBytes = 0;
	}
	
	
}


void Request(uint8_t cmd,	 uint8_t addr,uint8_t eop,
			uint8_t  dataCmd,uint8_t bmRequestType, uint8_t bmRequest,uint16_t wValue,uint16_t wIndex,uint16_t wLen,uint16_t waitForBytes)
{
	 current->rq.cmd  = cmd;
	 current->rq.addr = addr;
	 current->rq.eop  = eop;
	 current->rq.dataCmd = dataCmd;
	 current->rq.bmRequestType = bmRequestType;
	 current->rq.bmRequest = bmRequest;
	 current->rq.wValue = wValue;
	 current->rq.wIndex = wIndex;
	 current->rq.wLen = wLen;

	 current->numb_reps_errors_allowed = 4;
	 current->asckedReceiveBytes = waitForBytes;
	 current->acc_decoded_resp_counter    = 0;
	current->bAcked = 0;
//	 if(cmd==T_SETUP)
//	 {
		 current->cb_Cmd = CB_5;
//	 }
	// HAL_Delay(1);
}

void RequestSend(uint8_t cmd,	 uint8_t addr,uint8_t eop,
			uint8_t  dataCmd,uint8_t bmRequestType, uint8_t bmRequest,uint16_t wValue,uint16_t wIndex,uint16_t wLen,uint16_t transmitL1Bytes,uint8_t* data)
{
	 current->rq.cmd  = cmd;
	 current->rq.addr = addr;
	 current->rq.eop  = eop;
	 current->rq.dataCmd = dataCmd;
	 current->rq.bmRequestType = bmRequestType;
	 current->rq.bmRequest = bmRequest;
	 current->rq.wValue = wValue;
	 current->rq.wIndex = wIndex;
	 current->rq.wLen = wLen;
	 current->transmitL1Bytes = transmitL1Bytes;
	 for(int k=0;k<current->transmitL1Bytes;k++)
	 {
		 current->transmitL1[k] = data[k];
	 }
	 current->numb_reps_errors_allowed = 4;
	 current->acc_decoded_resp_counter    = 0;
//	 if(cmd==T_SETUP)
//	 {
		 current->cb_Cmd = CB_3;
//	 }
}

void RequestIn(uint8_t cmd,	 uint8_t addr,uint8_t eop,uint16_t waitForBytes)
{
	 current->rq.cmd  = cmd;
	 current->rq.addr = addr;
	 current->rq.eop  = eop;
	 current->numb_reps_errors_allowed = 4;
	 current->asckedReceiveBytes = waitForBytes;
	 current->acc_decoded_resp_counter    = 0;
	 current->cb_Cmd = CB_2;
}



void fsm_Mashine()
{
	if(!current->bComplete) return;
	current->bComplete = 0;
	
	
	
	 if(current->fsm_state == 0)
	 {
		current->epCount = 0;
		current->cb_Cmd     = CB_CHECK;
		current->fsm_state   = 1;
	 }
	 if(current->fsm_state == 1)
	 {
		if(current->wires_last_state==M_ONE)
		// if(1)
		{
			current->cmdTimeOut = 100+current->selfNum*73;
			//current->cmdTimeOut = 100;
			current->cb_Cmd      = CB_WAIT0;
			current->fsm_state   = 2;
		}
		else
		{
			current->fsm_state   = 0;
			current->cb_Cmd      = CB_CHECK;
		}
	 }
	 else if(current->fsm_state==2)
	 {
		current->cb_Cmd       = CB_RESET;
		current->fsm_state    = 3;
	 }
	 else if(current->fsm_state==3)
	 {
		current->cb_Cmd       = CB_POWER;
#ifdef TEST
		current->fsm_state    =  3;
#else
		current->fsm_state    =  4;
#endif
	 }
	 else if(current->fsm_state==4)
	 {
		Request(T_SETUP,ZERO_USB_ADDRESS,0b0000,T_DATA0,0x80,0x6,0x0100,0x0000,0x0012,0x0012); 
		current->fsm_state    = 5; 
	 }
	 else if(current->fsm_state==5)
	 {
		if(current->acc_decoded_resp_counter==0x12)
		{
			memcpy(&current->desc,current->acc_decoded_resp,0x12);
			current->ufPrintDesc |= 1;
			Request(T_SETUP,ZERO_USB_ADDRESS,0b0000,T_DATA0,0x00,0x5,0x0000+ASSIGNED_USB_ADDRESS,0x0000,0x0000,0x0000);
			current->fsm_state    =  6; 
		}
		else
		{
			if(current->numb_reps_errors_allowed<=0)
			{
				current->fsm_state    =  0; 
				return;
			}
			current->fsm_state    =  4; 
		}
#if 1
#else
		current->fsm_state    =  0;
#endif
	 }
	 else if(current->fsm_state==6)
	 {
		current->cmdTimeOut = 5; 
		current->cb_Cmd       = CB_WAIT1;
		current->fsm_state    = 7;
	 }
	 
	 else if(current->fsm_state==7)
	 {
		Request(T_SETUP,ASSIGNED_USB_ADDRESS,0b0000,T_DATA0,0x80,0x6,0x0200,0x0000,0x0009,0x0009); 
		current->fsm_state    = 8; 
	 }
	 else if(current->fsm_state==8)
	 {
		if(current->acc_decoded_resp_counter==0x9)
		{
			memcpy(&current->cfg,current->acc_decoded_resp,0x9);
			current->ufPrintDesc |= 2;
			Request(T_SETUP,ASSIGNED_USB_ADDRESS,0b0000,T_DATA0,0x80,0x6,0x0200,0x0000,current->cfg.wLength,current->cfg.wLength);
			current->fsm_state    = 9;
		}
		else
		{
			current->fsm_state      = 0;
			return ;
		}
	 }
	 else if(current->fsm_state==9)
	 {
		if(current->acc_decoded_resp_counter==current->cfg.wLength)
		{
			current->ufPrintDesc |= 4;
			current->descrBufferLen = current->acc_decoded_resp_counter;
			memcpy(current->descrBuffer,current->acc_decoded_resp,current->descrBufferLen);
			current->cmdTimeOut = 25;
			current->cb_Cmd       = CB_WAIT1;
			current->fsm_state    = 94; 
		}
		else
		{
			current->cmdTimeOut = 5;
			current->cb_Cmd       = CB_WAIT1;
			current->fsm_state    = 7;
		}
	 } 
	 else if(current->fsm_state==94)
	 {
		// config interfaces??
		//printf("set configuration 1\n");
		Request(T_SETUP,ASSIGNED_USB_ADDRESS,0b0000,T_DATA0,0x00,0x9,0x0001,0x0000,0x0000,0x0000);
		current->fsm_state    = 96; 
	 }
	 else if(current->fsm_state==96)
	 {
		// config interfaces??
		 if(current->bAcked)
		 {
			Request(T_SETUP,ASSIGNED_USB_ADDRESS,0b0000,T_DATA0,0x21,0xa,0x0000,0x0000,0x0000,0x0000);
			 current->fsm_state    = 99; 
		 }
		 else
		 {
			current->fsm_state    = 94; 
		 }
	 }
	 else if(current->fsm_state==99)
	 {
		//uint8_t cmd0 = current->cnt&0x20?0x7:0x0;
		//current->cnt++;
		//uint8_t cmd1 = current->cnt&0x20?0x7:0x0;
		 //printf(" 3 LEDs enable/disable on keyboard \n");
		 if(current->bAcked)
		 {
			uint8_t cmd1 = 0;
			RequestSend(T_SETUP,ASSIGNED_USB_ADDRESS,0b0000,T_DATA0,0x21,0x9,0x0200,0x0000,0x0001,0x0001,&cmd1);
			current->fsm_state    = 100; 
		 }
		 else
		 {
			 current->fsm_state    = 96; 
		 }
	 }
	 else if(current->fsm_state==100)
	 {
		 led(0);
		RequestIn(T_IN,	ASSIGNED_USB_ADDRESS,1,8);
		current->fsm_state    = 101; 
	 }
	 else if(current->fsm_state==101)
	 {
		 if(current->acc_decoded_resp_counter>=1)
		 {
			current->ufPrintDesc |= 8;
			current->R0Bytes= current->acc_decoded_resp_counter;
			memcpy(current->Resp0,current->acc_decoded_resp,current->R0Bytes);	 
			
			led(1);
			//gpio_set_level(B23_GPIO, 1);
		 }
			//~ RequestIn(T_IN,	ASSIGNED_USB_ADDRESS,2,8);
			//~ current->fsm_state    = 102; 
			if(current->epCount>=2)
			  {
				RequestIn(T_IN,	ASSIGNED_USB_ADDRESS,2,8);
				current->fsm_state    = 102; 
			 }
			 else
			 {
				current->cmdTimeOut = 3; 
				current->cb_Cmd        = CB_WAIT1;
				current->fsm_state      = 104; 
			 }
	 }
	 else if(current->fsm_state==102)
	 {
		 if(current->acc_decoded_resp_counter>=1)
		 {
			current->ufPrintDesc |= 16;
			current->R1Bytes= current->acc_decoded_resp_counter;
			memcpy(current->Resp1,current->acc_decoded_resp,current->R0Bytes);	 
		 }
		current->cmdTimeOut = 2; 
		current->cb_Cmd        = CB_WAIT1;
		current->fsm_state      = 104; 
	 }
	 else if (current->fsm_state==104)
	 {
		current->cmdTimeOut = 4; 
		current->cb_Cmd        = CB_WAIT1;
#ifdef  DEBUG_REPEAT		 
 static int rcnt =0;
		rcnt++;  // 	
		if(  (rcnt&0xff)==0 ||  (current->wires_last_state!=M_ONE))
#else
		 if(current->wires_last_state!=M_ONE)
#endif			 
		{
			current->fsm_state      = 0; 
			return ;
		}
		current->fsm_state      = 100; 
	 }
	 else
	 {
		current->cmdTimeOut = 2; 
		current->cb_Cmd        = CB_WAIT1;
		current->fsm_state      = 0; 
	 }
}


void setPins(int DPPin,int DMPin)
{
	DP_PIN = DPPin;
	DM_PIN = DMPin;
	int diff = DPPin - DMPin;
	if(abs(diff)>7)
	{
		printf("PIN DIFFERENCE MUST BE LESS 8!\n");
		exit(1);
	}
	int MIN_PIN = (DPPin<DMPin)?DPPin:DMPin;
	
	DM_PIN_M   = (1 << DMPin);
	DP_PIN_M    = (1 << DPPin);
	RD_MASK    = (1<<DPPin)|(1<<DMPin);
	int DIFF = MIN_PIN-8;
	if(DIFF>=0)
	{
		RD_SHIFT = DIFF;
		M_ONE = 1<<(DM_PIN-MIN_PIN);//<<DIFF;
		P_ONE = 1<<(DP_PIN-MIN_PIN);//<<DIFF;
	}
	else
	{
		RD_SHIFT = -DIFF;
		M_ONE = 2;
		P_ONE = 1;
	}
}

sUsbContStruct  current_usb[NUM_USB];
int checkPins(int dp,int dm)
{
	int diff = abs(dp-dm);
	if(diff>7||diff==0) 
	{
		return 0;
	}
	if( dp<8 || dp>31) return 0;
	if( dm<8 || dm>31) return 0;
	//p
	return 1;
}
#include <math.h>

float testDelay6(float freq_MHz)
{
	// 6 bits must take 4.0 uSec
#define SEND_BITS  126
	float res = 1;
	transmit_NRZI_buffer_cnt = 0;
	{
		for(int k=0;k<SEND_BITS/2;k++)
		{
			transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_K;
			transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;
		}
	portENTER_CRITICAL();			
		uint32_t stim = _getCycleCount32();
		sendOnly();
		stim =  _getCycleCount32()- stim;
	portEXIT_CRITICAL	();
		res = stim*6.0/freq_MHz/SEND_BITS;
		printf("%d bits in %d nSec %d KHz  6 ticks in %d nS\n",SEND_BITS,(int)(1000*stim/(float)freq_MHz),(int)(1000*SEND_BITS*freq_MHz/stim),(int)(1000*stim*6.0/freq_MHz/SEND_BITS));
	}
	return res; 
}

uint8_t arr[0x200];

void initStates(int DP0,int DM0,int DP1,int DM1,int DP2,int DM2,int DP3,int DM3)
{
	decoded_receive_buffer_head = 0;
	decoded_receive_buffer_tail = 0;
	transmit_bits_buffer_store_cnt = 0;
	//printf("delayProc =%p\n",delayProc);
	printf("setDelay =%p\n",&setDelay);
#if 1
	printf("p0 = %p \n",&del1);
	printf("de = %p \n",&del2);
	printf("p1 = %p \n",&del3);
	int  len = (((uint32_t)&del3) - ((uint32_t)&del1))*2;
	for(int k=0;k<len;k++)
	{
		memcpy(arr,&del1,0x200);
	}
	printf("\n");
	for(int k=0;k<len;k++)
	{
		if(!(k&0xf)) printf("\n");
		printf("%02x " ,arr[k]);
	}
	printf("\n");
#endif	
	int calibrated = 0;
	for(int k=0;k<NUM_USB;k++)
	{
		current = &current_usb[k];
		if(k==0)
		{
			current->DP = DP0;
			current->DM = DM0;
		}
		else if(k==1)
		{
			current->DP = DP1;
			current->DM = DM1;
		}
		else if(k==2)
		{
			current->DP = DP2;
			current->DM = DM2;
		}
		else if(k==3)
		{
			current->DP = DP3;
			current->DM = DM3;
		}
		current->isValid = 0;
		if(checkPins(current->DP,current->DM))
		{
			printf("pins %d %d is OK!\n",current->DP,current->DM);
			current->selfNum = k;
			current->in_data_flip_flop       = 0;
			current->bComplete  = 1;
			current->cmdTimeOut = 0;
			current->ufPrintDesc =0;
			current->cb_Cmd     = CB_CHECK;
			current->fsm_state  = 0;
			current->wires_last_state = 0;
			current->counterNAck = 0;
			current->counterAck = 0;
			current->epCount = 0;
			
			gpio_config_t io_conf;	
			io_conf.intr_type      = GPIO_INTR_DISABLE;	
			io_conf.pin_bit_mask = (1<<current->DP) | (1<<current->DM);
			io_conf.pull_down_en = 0;	
			io_conf.pull_up_en = 0;	
			io_conf.mode = GPIO_MODE_INPUT;
			gpio_config(&io_conf);
			
			//~ gpio_pad_select_gpio(current->DP);
			//~ gpio_set_direction(current->DP, GPIO_MODE_INPUT);
			//~ gpio_pulldown_en(current->DP);
			
			//~ gpio_pad_select_gpio(current->DM);
			//~ gpio_set_direction(current->DM, GPIO_MODE_INPUT);
			//~ gpio_pulldown_en(current->DM);
			current->isValid = 1;
			
			// TEST
			setPins(current->DP,current->DM);
			
			if(!calibrated)
			{	
				//calibrate delay divide 2
				int  uTime = 254;
				int  dTime = 0;
				
				//rtc_cpu_freq_config_t  out_config;
				
				//rtc_clk_cpu_freq_get_config(&out_config);
				uint32_t freq_mhz = esp_clk_cpu_freq()/1000000;
				
				//uint32_t freq = rtc_clk_cpu_freq_value(rtc_clk_cpu_freq_get());
				printf("cpu freq = %d MHz\n",freq_mhz);
				
				TM_OUT = freq_mhz/2;
				
				// 8  - func divided clock to 8, 1.5 - MHz USB LS
				TIME_MULT = (int)(TIME_SCALE/(freq_mhz/4/1.5)+0.5);
				printf("TIME_MULT = %d \n",TIME_MULT);
				
				int     TRANSMIT_TIME_DELAY_OPT = 0;
				TRANSMIT_TIME_DELAY = TRANSMIT_TIME_DELAY_OPT;
				setDelay(TRANSMIT_TIME_DELAY);
				printf("TT=%d ",TRANSMIT_TIME_DELAY);
				float  cS_opt = testDelay6(freq_mhz);
//#define OPT_TIME (4.50f)
#define OPT_TIME (4.0f)
				for(int p=0;p<9;p++)
				{
					TRANSMIT_TIME_DELAY = (uTime+dTime)/2;
					setDelay(TRANSMIT_TIME_DELAY);
					printf("TT=%d ",TRANSMIT_TIME_DELAY);
					float cS = testDelay6(freq_mhz);
					if(fabsf(OPT_TIME-cS)<fabsf(OPT_TIME-cS_opt))
					{
						cS_opt = cS;
						TRANSMIT_TIME_DELAY_OPT = TRANSMIT_TIME_DELAY;
					}
					if(cS<OPT_TIME)
					{
						dTime = TRANSMIT_TIME_DELAY;
					}
					else
					{
						uTime = TRANSMIT_TIME_DELAY;
					}
				}
				TRANSMIT_TIME_DELAY = TRANSMIT_TIME_DELAY_OPT;
				setDelay(TRANSMIT_TIME_DELAY);
				printf("TT=%d ",TRANSMIT_TIME_DELAY);
				cS_opt = testDelay6(freq_mhz);
				printf("TRANSMIT_TIME_DELAY = %d time = %d error = %d%% \n",TRANSMIT_TIME_DELAY,(int)(1000*cS_opt),(int)((cS_opt-OPT_TIME)/OPT_TIME*100));
				//calibrated = 1;
			}
		}
		else
		{
			printf("pins %d %d is Errors !\n",current->DP,current->DM);
		}
		
	}
}

void usb_process()
{
	for(int k=0;k<NUM_USB;k++)
	{
		current = &current_usb[k];
		if(current->isValid)
		{
			setPins(current->DP,current->DM);
			timerCallBack();
			fsm_Mashine();
		}
	}
}
void printState()
{
	
static int cntl = 0;		
		cntl++;
	         int ref = cntl%NUM_USB;
		sUsbContStruct * pcurrent = &current_usb[ref];
		if(!pcurrent->isValid) return ;
		if((cntl%200)<NUM_USB)
		{
			printf("USB%d:flagR3 = %d Ack = %d Nack = %d %02x pcurrent->cb_Cmd = %d  state = %d epCount = %d",cntl%NUM_USB,flagR3,pcurrent->counterAck,pcurrent->counterNAck,pcurrent->wires_last_state,pcurrent->cb_Cmd,pcurrent->fsm_state,pcurrent->epCount);
#ifdef DEBUG_ALL
			for(int k=0;k<20;k++)
			{
				printf("%04x ", debug_buff[k]);
			}
#endif
			printf("\n");
		}
		//~ for(int k=0;k<0x14;k++)
		//~ {
			//~ if(cntl &1 )
			//~ {
				//~ printf("%04x ",prn(tr[k],0));
			//~ }
			//~ else
			//~ {
				//~ printf("%04x ",pr[k]);
			//~ }
		//~ }
		//~ printf("\n");
		if(pcurrent->ufPrintDesc&1)
		{
			pcurrent->ufPrintDesc &= ~(uint32_t)1;
			//~ printf("desc.bcdUSB          = %02x\n",pcurrent->desc.bcdUSB);
			//~ printf("desc.bDeviceClass    = %02x\n",pcurrent->desc.bDeviceClass);
			//~ printf("desc.bDeviceSubClass = %02x\n",pcurrent->desc.bDeviceSubClass);
			//~ printf("desc.bDeviceProtocol = %02x\n",pcurrent->desc.bDeviceProtocol);
			//~ printf("desc.bMaxPacketSize0 = %02x\n",pcurrent->desc.bMaxPacketSize0);
			//~ printf("desc.idVendor        = %02x\n",pcurrent->desc.idVendor);
			//~ printf("desc.idProduct       = %02x\n",pcurrent->desc.idProduct);
			printf("desc.bcdDevice       = %02x\n",pcurrent->desc.bcdDevice);
			printf("desc.iManufacturer   = %02x\n",pcurrent->desc.iManufacturer);
			printf("desc.iProduct        = %02x\n",pcurrent->desc.iProduct);
			//printf("desc.iSerialNumber   = %02x\n",pcurrent->desc.iSerialNumber);
			printf("desc.bNumConfigurations = %02x\n",pcurrent->desc.bNumConfigurations);
		}
		if(pcurrent->ufPrintDesc&2)
		{
			pcurrent->ufPrintDesc &= ~(uint32_t)2;
			printf("cfg.bLength         = %02x\n",pcurrent->cfg.bLength);
			//printf("cfg.bType           = %02x\n",pcurrent->cfg.bType);
			printf("cfg.wLength         = %02x\n",pcurrent->cfg.wLength);
			printf("cfg.bNumIntf        = %02x\n",pcurrent->cfg.bNumIntf);
			//~ printf("cfg.bCV             = %02x\n",pcurrent->cfg.bCV);
			//~ printf("cfg.bIndex          = %02x\n",pcurrent->cfg.bIndex);
			//~ printf("cfg.bAttr           = %02x\n",pcurrent->cfg.bAttr);
			//~ printf("cfg.bMaxPower       = %d\n",pcurrent->cfg.bMaxPower);
		}
		if(pcurrent->ufPrintDesc&8)
		{
			pcurrent->ufPrintDesc &= ~(uint32_t)8;
			printf("in0 :");
			for(int k=0;k<pcurrent->R0Bytes;k++)
			{
				printf("%02x ",pcurrent->Resp0[k]);
			}
			printf("\n");
		}
		if(pcurrent->ufPrintDesc&16)
		{
			pcurrent->ufPrintDesc &= ~(uint32_t)16;
			printf("in1 :");
			for(int k=0;k<pcurrent->R1Bytes;k++)
			{
				printf("%02x ",pcurrent->Resp1[k]);
			}
			printf("\n");
		}
		
		if(pcurrent->ufPrintDesc&4)
		{
			pcurrent->ufPrintDesc &= ~(uint32_t)4;
			sCfgDesc  	lcfg;
			sIntfDesc 		sIntf;
			HIDDescriptor 	hid[4];
			sEPDesc 		epd;
			int 			cfgCount   = 0;
			int 			sIntfCount   = 0;
			int 			hidCount   = 0;
			
			int 			pos = 0;
			#define STDCLASS        0x00
			#define HIDCLASS        0x03
			#define HUBCLASS	 	0x09      /* bDeviceClass, bInterfaceClass */
			printf("clear epCount %d self = %d %d\n",pcurrent->epCount,pcurrent->selfNum,pcurrent->descrBufferLen);
			pcurrent->epCount     = 0;
				while(pos<pcurrent->descrBufferLen-2)
				{
					uint8_t len  =  pcurrent->descrBuffer[pos];
					uint8_t type =  pcurrent->descrBuffer[pos+1];
					if(len==0)
					{
						//printf("pos = %02x type = %02x cfg.wLength = %02x pcurrent->acc_decoded_resp_counter = %02x\n ",pos,type,cfg.wLength,pcurrent->acc_decoded_resp_counter);
						pos = pcurrent->descrBufferLen;
					}
					if(pos+len<=pcurrent->descrBufferLen)
					{
						printf("\n");
							if(type == 0x2)
							{
								sCfgDesc cfg;
								memcpy(&cfg,&pcurrent->descrBuffer[pos],len);
								//printf("cfg.bLength         = %02x\n",cfg.bLength);
								//printf("cfg.bType           = %02x\n",cfg.bType);
								
								printf("cfg.wLength         = %02x\n",cfg.wLength);
								//~ printf("cfg.bNumIntf        = %02x\n",cfg.bNumIntf);
								//~ printf("cfg.bCV             = %02x\n",cfg.bCV);
								//printf("cfg.bIndex          = %02x\n",cfg.bIndex);
								//printf("cfg.bAttr           = %02x\n",cfg.bAttr);
								//~ printf("cfg.bMaxPower       = %d\n",cfg.bMaxPower);

							}
							else if (type == 0x4)
							{
								sIntfDesc sIntf;
								memcpy(&sIntf,&pcurrent->descrBuffer[pos],len);
								//printf("sIntf.bLength      = %02x\n",sIntf.bLength);
								//printf("sIntf.bType        = %02x\n",sIntf.bType);
								printf("sIntf.iNum         = %02x\n",sIntf.iNum);
								//~ printf("sIntf.iAltString   = %02x\n",sIntf.iAltString);
								//~ printf("sIntf.bEndPoints   = %02x\n",sIntf.bEndPoints);
								//~ printf("sIntf.iClass       = %02x\n",sIntf.iClass);
								//~ printf("sIntf.iSub         = %02x\n",sIntf.iSub);
								//~ printf("sIntf.iProto       = %d\n",sIntf.iProto);
								//~ printf("sIntf.iIndex       = %d\n",sIntf.iIndex);

							}
							else if (type == 0x21)
							{

								hidCount++;
								int i = hidCount-1;
								memcpy(&hid[i],&pcurrent->descrBuffer[pos],len);
								//printf("hid.bLength          = %02x\n",hid[i].bLength);
								//printf("hid.bDescriptorType  = %02x\n",hid[i].bDescriptorType);
								printf("hid.bcdHID           = %02x\n",hid[i].bcdHID);
								//~ printf("hid.bCountryCode     = %02x\n",hid[i].bCountryCode);
								//~ printf("hid.bNumDescriptors  = %02x\n",hid[i].bNumDescriptors);
								//~ printf("hid.bReportDescriptorType = %02x\n",hid[i].bReportDescriptorType);
								//~ printf("hid.wItemLengthH         = %02x\n",hid[i].wItemLengthH);
								//~ printf("hid.wItemLengthL         = %02x\n",hid[i].wItemLengthL);
							}
							else if (type == 0x5)
							{
								pcurrent->epCount++;
								printf("pcurrent->epCount = %d\n",pcurrent->epCount);
								sEPDesc epd;
								memcpy(&epd,&pcurrent->descrBuffer[pos],len);
								//printf("epd.bLength       = %02x\n",epd.bLength);
								//printf("epd.bType         = %02x\n",epd.bType);
								printf("epd.bEPAdd        = %02x\n",epd.bEPAdd);
								printf("epd.bAttr         = %02x\n",epd.bAttr);
								printf("epd.wPayLoad      = %02x\n",epd.wPayLoad);
								printf("epd.bInterval     = %02x\n",epd.bInterval);
							}
					}
					pos+=len;
				}
		}
	
}
