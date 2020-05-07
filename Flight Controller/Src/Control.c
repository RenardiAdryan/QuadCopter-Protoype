/*
 * mainControl.c
 *
 *  Created on: Sep 14, 2019
 *      Author: Renardi Adryantoro P
 				Polteknik elektronika negeri suarabaya
 				Computer Engineering 2016
 */


#include "GlobalVariable.h"



void InitializeTimer(){
HAL_TIM_Base_Start(&htim1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


}

void pidreset(){
	GyroYAW.sumIntegral		= 0;
	GyroROLL.sumIntegral	= 0;
	GyroPITCH.sumIntegral	= 0;

	GyroYAW.output	=0;
	GyroROLL.output =0;
	GyroPITCH.output=0;

	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1000);
}

void PIDControlYAW(){

	GyroYAW.setPoint = constrain(yawReceive, -yawMAX, yawMAX);
	GyroYAW.error = GyroYAW.setPoint - dataYaw;
	if(GyroYAW.error >=180){GyroYAW.error -=360;}
	else if(GyroYAW.error <-180){GyroYAW.error+=360;}

	GyroYAW.sumIntegral += GyroYAW.error * GyroYAW.timesampling;
	GyroYAW.sumIntegral = constrain(GyroYAW.sumIntegral,-200,200);

	GyroYAW.derivative = (GyroYAW.error - GyroYAW.preverror)/GyroYAW.timesampling;
	GyroYAW.preverror = GyroYAW.error;

	GyroYAW.output = GyroYAW.kp * GyroYAW.error + GyroYAW.kd*GyroYAW.derivative + GyroYAW.ki * GyroYAW.sumIntegral;

}

void PIDControlROLL(){


	GyroROLL.setPoint = constrain(rollReceive, -rollMAX, rollMAX);
	GyroROLL.error = GyroROLL.setPoint - dataRoll;
	if(GyroROLL.error >=180){GyroROLL.error -=360;}
	else if(GyroROLL.error <-180){GyroROLL.error+=360;}

	GyroROLL.sumIntegral += GyroROLL.error * GyroROLL.timesampling;
	GyroROLL.sumIntegral = constrain(GyroROLL.sumIntegral,-500,500);

	GyroROLL.derivative = (GyroROLL.error - GyroROLL.preverror)/GyroROLL.timesampling;
	GyroROLL.preverror = GyroROLL.error;

	GyroROLL.output = GyroROLL.kp * GyroROLL.error + GyroROLL.kd*GyroROLL.derivative + GyroROLL.ki * GyroROLL.sumIntegral;

}

void PIDControlPITCH(){

	GyroPITCH.setPoint = constrain(pitchReceive, -pitchMAX, pitchMAX);
	GyroPITCH.error =  GyroPITCH.setPoint - dataPitch;
	if(GyroPITCH.error >=180){GyroPITCH.error -=360;}
	else if(GyroPITCH.error <-180){GyroPITCH.error+=360;}

	GyroPITCH.sumIntegral += GyroPITCH.error * GyroPITCH.timesampling;
	GyroPITCH.sumIntegral = constrain(GyroPITCH.sumIntegral,-500,500);

	GyroPITCH.derivative = (GyroPITCH.error - GyroPITCH.preverror)/GyroPITCH.timesampling;
	GyroPITCH.preverror = GyroPITCH.error;

	GyroPITCH.output = GyroPITCH.kp * GyroPITCH.error + GyroPITCH.kd*GyroPITCH.derivative + GyroPITCH.ki * GyroPITCH.sumIntegral;

}

 float motor1Thrust,motor2Thrust,motor3Thrust,motor4Thrust;
 float motor1Torque,motor2Torque,motor3Torque,motor4Torque;
 float thrust;
 int pulseESC1,pulseESC2,pulseESC3,pulseESC4;
 int RPMmotor1,RPMmotor2,RPMmotor3,RPMmotor4;

float debug;
void trustControl(){

/*
  		CW 2\     /1 CCW
			 \   /
			  \ /
			  / \
           	 /   \
	   CCW 3/     \4 CW

	       QuadCopter
		   Motor 1000 KV
		   Propeller : 11 x 4.7
		   MaX thrust = 102.449448 N
		   Thrust for each motor = 25.6123619

	Formula :
	F = 1.225 * ((3.14(0.0254 * d)^2)/4) *(RPM*0.0254*pitch*1/60)^2 * (d/(3.29546*pitch))^1.5;
	F = 0.0750686 * 570.063376 * 0.598505539

*/


	const float RADS 			= 57.29577795;
	const float angleMotor1		= 45;
	const float angleMotor2		= 135;
	const float angleMotor3		= 225;
	const float angleMotor4		= 315;
	const float L				= 0.225;// arm length calculate from center of mass in 2D;


	thrust = map(throttle,0,200,0,102.449448);
	debug = cos(angleMotor1/RADS);
	motor1Torque = (thrust/4 + GyroPITCH.output*sin(angleMotor1/RADS) + GyroROLL.output*cos(angleMotor1/RADS) - GyroYAW.output)*L;
	motor2Torque = (thrust/4 + GyroPITCH.output*sin(angleMotor2/RADS) + GyroROLL.output*cos(angleMotor2/RADS) + GyroYAW.output)*L;
	motor3Torque = (thrust/4 + GyroPITCH.output*sin(angleMotor3/RADS) + GyroROLL.output*cos(angleMotor3/RADS) - GyroYAW.output)*L;
	motor4Torque = (thrust/4 + GyroPITCH.output*sin(angleMotor4/RADS) + GyroROLL.output*cos(angleMotor4/RADS) + GyroYAW.output)*L;

	motor1Thrust = motor1Torque/L;
	motor2Thrust = motor2Torque/L;
	motor3Thrust = motor3Torque/L;
	motor4Thrust = motor4Torque/L;

	RPMmotor1 = sqrt(motor1Thrust/ 0.0449289729)/0.0019896667;
	RPMmotor2 = sqrt(motor2Thrust/ 0.0449289729)/0.0019896667;
	RPMmotor3 = sqrt(motor3Thrust/ 0.0449289729)/0.0019896667;
	RPMmotor4 = sqrt(motor4Thrust/ 0.0449289729)/0.0019896667;

	RPMmotor1 = constrain(RPMmotor1,0,12000);
	RPMmotor2 = constrain(RPMmotor2,0,12000);
	RPMmotor3 = constrain(RPMmotor3,0,12000);
	RPMmotor4 = constrain(RPMmotor4,0,12000);

	pulseESC1 = map(RPMmotor1,0,12000,1000,2000);
	pulseESC2 = map(RPMmotor2,0,12000,1000,2000);
	pulseESC3 = map(RPMmotor3,0,12000,1000,2000);
	pulseESC4 = map(RPMmotor4,0,12000,1000,2000);


	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pulseESC1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,pulseESC2);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,pulseESC3);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,pulseESC4);


}



