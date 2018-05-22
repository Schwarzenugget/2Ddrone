/*
 * pwm_led.c
 *
 *  Created on: Apr 29, 2018
 *      Author: David
 */

#include <derivative.h>

void delay(int n){
	int counter;
	for(counter=0; counter<100000*n; counter++){};
}

void PWM_LED_Init()
{
    //MCG_BASE_PTR->C1 = MCG_C1_IREFS_MASK | MCG_C1_IRCLKEN_MASK; //  INTERNAL CLOCK|MCGIRCLK ACTIVE(SET)
    //MCG_BASE_PTR->C2 = MCG_C2_IRCS_MASK;   // SELECT FAST INTERNAL REFERENCE CLOCK (1)
   // SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_TPM2_MASK;  // ENABLE TPM2 CLOCK GATE
   // SIM_BASE_PTR->SOPT2 |= SIM_SOPT2_TPMSRC(3);  // MCGIRCLK IS SELECTED FOR TPM CLOCK

    //TPM2_BASE_PTR->SC = TPM_SC_CMOD(0);  // DISABLE EVERY CLOCK
    //TPM2_BASE_PTR->SC |= TPM_SC_PS(0);  // TODO especificar frequencia

    //TPM2_BASE_PTR->MOD = 40000;  // TODO especificar frequencia

    //SIM_BASE_PTR->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    //PORTB_BASE_PTR->PCR[18] = PORT_PCR_MUX(3);  // TODO especificar multiplexacio del TPM2_CH0

    //TPM2_BASE_PTR->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // SELECT CHANNEL MODE

    //TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD/10;  // TODO especificar duty cycle
    
    //TPM2_BASE_PTR->SC = TPM_SC_CMOD(1);  // COUNTER INC. ON EVERY CLOCK
    
    //LED CONFIG
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB_PCR18 |= PORT_PCR_MUX(3);
    PORTB_PCR19 |= PORT_PCR_MUX(3);
    
    //TPM CONFIG
	/*Enable clock con TPM0*/
	SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK;
	/*Select clock source*/
	SIM_SOPT2 |= 0x01 << SIM_SOPT2_TPMSRC_SHIFT; //MCGFLLCLK clock
	/*Disable TPM while configuring it*/
	TPM2_SC = TPM_SC_CMOD(0);
	/*TPM Modulo Register */
	TPM2_MOD = 8000; //1sec
	/*Divide Clock by 2^n*/
	TPM2_SC |= TPM_SC_PS(0);
	/*OC toogle mode*/
	TPM2_C0SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; //PWM and Make signal low on match
	TPM2_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; //PWM and Make signal low on match
	/*schedule next transition*/
	TPM2_C0V = 7600; //Duty Cycle 
	TPM2_C1V = 7600; //Duty Cycle
	/*Enable timer mode*/
	TPM2_SC |= TPM_SC_CMOD(1);
	
	/*PORTD_PCR1 &= ~PORT_PCR_MUX(4);
	PORTD_PCR1 |= PORT_PCR_MUX(1);
	GPIOD_PDDR |= 0x2; 	
	GPIOD_PDOR &= ~0x2;*/

}

void PWM_LED_Duty_Cycle(float axis){
	short negative_flag = 0;
	if(axis <0){
		axis = -axis;
		negative_flag = 1;
	}
	if(axis>1)axis=1;
	
	if(negative_flag){
		TPM2_C0V = TPM2_MOD;
		TPM2_C1V = TPM2_MOD*(1-axis);
	}else{
		TPM2_C0V = TPM2_MOD*(1-axis);
		TPM2_C1V = TPM2_MOD;
	}
}


