#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <uart/e_uart_char.h>
#include <bluetooth/e_bluetooth.h>

#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>

#include <motor_led/advance_one_timer/e_led.h>
#include <motor_led/advance_one_timer/e_motors.h>

#include <motor_led/advance_one_timer/e_agenda.h>

#include <codec/e_sound.h>
#include <I2C/e_i2c_protocol.h>

#include <a_d/advance_ad_scan/e_ad_conv.h>
#include <a_d/advance_ad_scan/e_acc.h>
#include <a_d/advance_ad_scan/e_prox.h>
#include <a_d/advance_ad_scan/e_micro.h>


#define uart_send_static_text(msg) do { e_send_uart1_char(msg,sizeof(msg)-1); while(e_uart1_sending()); } while(0)
#define uart_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0)

#ifndef ZUSMORO_H
#define ZUSMORO_H
struct _guard {
	int op;
	int variable;
	int compValue;

};
typedef struct _guard guard;


struct _transition {
	int followState;
	int numberOfGuards;
	int* guards;
};
typedef struct _transition transition;

struct _state {
	int leds;
	/*
	* 1* LED0 2^0
	* 2* LED1 2^1
	* 4* LED2 2^2
	* 8* LED3 2^3
	* 16* LED4 2^4
	* 32* LED5 2^5
	* 64* LED6 2^6
	* 128* LED7 2^7
	* 256* FrontLED 2^8
	* 512* BodyLED 2^9
	* 1024* BEEP 2^10
	*/
	int motorLoffset;
	int motorLmin;
	int motorLmax;
	int motorLKpr;
	int motorLvar;
	int motorLvalue;
	
	int motorRoffset;
	int motorRmin;
	int motorRmax;
	int motorRKpr;
	int motorRvar;
	int motorRvalue;
	
	
	int numberOfTransitions;
	int* transitions;
};
typedef struct _state state;


void readInput();
void sendTransition(int y);
void sendState(int y);
void setSound(int s);
void setLeds(int n);
void enterState();
int controller(int off, int min, int max,
			int kpr, int var, int value);
#endif
