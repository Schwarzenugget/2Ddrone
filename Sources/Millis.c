/*
 * Millis.c
 *
 *  Created on: May 20, 2018
 *      Author: David
 */
#include "Millis.h"

/*Milliseconds counter*/

void Millis_Init(){
	millis = 0;
	
	SYST_CSR = 0x1; //Enable Timer
	SYST_CSR |= 1 << SysTick_CSR_CLKSOURCE_SHIFT; //System clock when 1, divided by 16 when 0
	SYST_CSR |= 0x1 << SysTick_CSR_TICKINT_SHIFT; //Enable Interruptions
	SYST_RVR = 41940; //Timer start value
}

void SysTick_Handler(void){
	millis++;
}
