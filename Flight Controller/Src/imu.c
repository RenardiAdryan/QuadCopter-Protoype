/*
 *  Created on: Sep 14, 2019
 *      Author: Renardi Adryantoro P
 				Polteknik elektronika negeri suarabaya
 				Computer Engineering 2016
*/

#include "GlobalVariable.h"

uint8_t DataReceivedIMU[8]={0,0,0,0,0,0};//REceive ONly 8 bit
void InitSerialIMU_DMA(){

	HAL_UART_Receive_DMA(&huart3,DataReceivedIMU,sizeof(DataReceivedIMU));

}



int YPR[3];
unsigned char Re_buf[8],counter=0;
int dataYaw,dataPitch,dataRoll;
char flagReset=0;
float deltaTime,prevTime,timeNow;
void Parsing_IMU(){


	if(DataReceivedIMU[0] == 0xAA && DataReceivedIMU[7] == 0x55) {

		flagparsingIMU =1;

			YPR[0] = (DataReceivedIMU[1] << 8 | DataReceivedIMU[2])*0.01f;
			YPR[1] = (DataReceivedIMU[3] << 8 | DataReceivedIMU[4])*0.01f;
			YPR[2] = (DataReceivedIMU[5] << 8 | DataReceivedIMU[6])*0.01f;
			
			//Setelah nilai IMU 179 lalu melompat ke 475
			//range sudut 0-179 lalu 475-655 
			if (YPR[0] > 179)		{dataYaw = (655-YPR[0]);
																if(YPR[0]>475){dataYaw=-dataYaw;} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else									dataYaw = YPR[0];
			
			if (YPR[1] > 179)		{dataPitch = (655-YPR[1]);
															if(YPR[1]>475){dataPitch=-dataPitch;} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else									dataPitch = YPR[1];
			
			if (YPR[2] > 179)		{dataRoll = (655-YPR[2]);
																if(YPR[2]>475){dataRoll=-dataRoll;} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else									dataRoll = YPR[2];
	
	}
	else {	flagparsingIMU =0;}

	//dataRoll=dataRoll-180;
	//if(dataYaw<0) {dataYaw+=360;}
	//if(dataPitch<0) dataPitch += 360;
	//if(dataRoll<0) dataRoll += 360;
	
	//SEND to GCS
	HAL_UART_Transmit(&huart1,DataReceivedIMU, sizeof(DataReceivedIMU), 10);
}



void resetIMU(){
	uint8_t reset[2]={0xA5,0x55};
	uint8_t reset1[2]={0xA5,0x54};
	uint8_t reset2[2]={0xA5,0x52};

	HAL_UART_DMAPause(&huart3);
	HAL_Delay(50);
	HAL_UART_Transmit(&huart3,(uint8_t *)reset,sizeof(reset),100);
	HAL_Delay(170);
	HAL_UART_Transmit(&huart3,(uint8_t *)reset1,sizeof(reset1),100);
	HAL_Delay(170);
	HAL_UART_Transmit(&huart3,(uint8_t *)reset2,sizeof(reset2),100);
	HAL_UART_DMAResume(&huart3);
}
