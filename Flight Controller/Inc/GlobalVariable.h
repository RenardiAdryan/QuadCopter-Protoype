/*
 * GlobalVariable.h
 *
 *  Created on: Sep 14, 2019
 *      Author: Renardi Adryantoro P
 */

#ifndef GLOBALVARIABLE_H_
#define GLOBALVARIABLE_H_

#include "main.h"
#include "stm32f1xx_hal.h"
#include "math.h"
#include "stdio.h"

#define digitalRead(x,y)													HAL_GPIO_ReadPin(x,y)
#define HIGH																			1
#define LOW																				0
#define digitalWrite(x,y,val)									((val) ? (HAL_GPIO_WritePin(x,y,GPIO_PIN_SET))  : (HAL_GPIO_WritePin(x,y,GPIO_PIN_RESET)) )

#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
#define constrain(nilaix,bawah,atas) ( (nilaix)<(bawah) ? (bawah) : ( (nilaix)>(atas) ? (atas) : (nilaix) ) )


#define BuzzPORT GPIOB
#define BuzzPIN  GPIO_PIN_5

#define LED1PORT GPIOC
#define LED1PIN GPIO_PIN_14

#define BUTTON1PORT GPIOC
#define BUTTON1PIN  GPIO_PIN_15

#define BUTTON2PORT GPIOA
#define BUTTON2PIN  GPIO_PIN_12


typedef struct {
	float error;
	float preverror;
	float derivative;
	float sumIntegral;
	float setPoint;
	float output;
	float kp;
	float kd;
	float ki;
	float timesampling;
}varPID;
extern varPID GyroYAW,GyroPITCH,GyroROLL;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern TIM_HandleTypeDef htim1;

extern void PIDControlYAW();
extern void PIDControlROLL();
extern void PIDControlPITCH();
extern void trustControl();
extern void pidreset();

extern void InitSerialIMU_DMA(void);
extern void InitSerialReceiver_DMA(void);
extern void InitializeTimer(void);
extern void Parsing_IMU(void);
extern void Parsing_Receiver(void);
extern void resetIMU(void);
extern void fail_safe(void);
extern int dataYaw,dataPitch,dataRoll;
extern int throttle;
extern uint8_t speedMode;
extern int yawMAX,pitchMAX,rollMAX;
extern int yawReceive,pitchReceive,rollReceive;
extern char flagdroneARM;
extern char flagparsingIMU,flagparsingReceiver;

extern float motor1Thrust,motor2Thrust,motor3Thrust,motor4Thrust;
extern float motor1Torque,motor2Torque,motor3Torque,motor4Torque;
extern float thrust;
extern int pulseESC1,pulseESC2,pulseESC3,pulseESC4;
extern int RPMmotor1,RPMmotor2,RPMmotor3,RPMmotor4;
extern uint8_t data1[60];

#endif /* GLOBALVARIABLE_H_ */
