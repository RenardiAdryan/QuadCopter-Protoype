/*
 * receiver.c
 *
 *  Created on: Sep 14, 2019
 *      Author: Renardi Adryantoro P
 */

#include <GlobalVariable.h>

int throttle,safeThrottle,receiverStatus,yawReceive,rollReceive,pitchReceive;
uint8_t speedMode;

uint8_t data[9];
uint8_t DataReceivedReceiver;
uint8_t dataReceive[8];
void InitSerialReceiver_DMA(){
	HAL_UART_Receive_DMA(&huart1,data,sizeof(data));

}

void Parsing_Receiver(){
//static int count = 8,flag;

//	if(DataReceivedReceiver == '#' && count>=7){flag=1;count=0;}
//	else if(flag==1){
//		count++;
//		dataReceive[count] = DataReceivedReceiver;
//		if( DataReceivedReceiver == '!' && count>=7 ){
//			throttle 		= dataReceive[1];
//			serialReceive 	= dataReceive[2];
//			speedMode     	= dataReceive[3];
//			yawReceive  = ((dataReceive[4]<<8) | dataReceive[5])-400;
//			pitchReceive  	= dataReceive[6]-50;
//			rollReceive   	= dataReceive[7]-50;
//			flag=0;
//		}
//
//	}
	if(data[0] == '#' && data[8] == '!'){
		flagparsingReceiver =1;

		receiverStatus 	= data[2];
		if(receiverStatus){
			safeThrottle=throttle = data[1];
					speedMode     	= data[3];
					yawReceive  = ((data[4]<<8) | data[5])-400;
					pitchReceive  	= data[6]-50;
					rollReceive   	= data[7]-50;
		}
	}
	else {	flagparsingReceiver =0;}


}


void fail_safe(){


	if(!receiverStatus){

		if(safeThrottle > 100){throttle = 60;}
		else throttle = 0;

		speedMode =0;
		yawReceive =0;
		pitchReceive =0;
		rollReceive =0;
	}



}
