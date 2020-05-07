#ifndef HEADER_H
#define HEADER_H

#include <avr/sleep.h>
#include <nRF24L01.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <RF24.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/



#define BAUDRATE 9600
#define PUSHBUTTON1 3
#define PUSHBUTTON2 4
#define IRQ 2

#define LED1 A0
#define BUZZER A1

#define CSN  8
#define CE   7


#endif
