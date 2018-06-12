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
    PORTB_PCR18 |= PORT_PCR_MUX(1);
    PORTB_PCR19 |= PORT_PCR_MUX(1);
#if 1
    GPIOB_PDDR |= 1<<18;
#endif
#if 0
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
#endif

}

void PWM_Motors_Init(){
	/*
		* X:PWMA - PTB0
		* Y:PWMA - PTB1
		* X:PWMB - PTB2
		* X:PWMB - PTB3
		* AIN2(A&B) - PTC2
		* AIN1(A&B) - PTE29 (PTC1 is not properly soldered)
		* STBY(A&B) - PTE20
		* BIN1(A&B) - PTE21
		* BIN2(A&B) - PTE22
		*/
	
	/*Init Pins*/
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTB_PCR0 |= PORT_PCR_MUX(3); //PWM TPM1 CH0
	PORTB_PCR1 |= PORT_PCR_MUX(3); //PWM TPM1 CH1
	PORTB_PCR2 |= PORT_PCR_MUX(3); //PWM TPM2 CH0
	PORTB_PCR3 |= PORT_PCR_MUX(3); //PWM TPM2 CH1
	PORTC_PCR2 |= PORT_PCR_MUX(1);//GPIO
	PORTE_PCR29 |= PORT_PCR_MUX(1);//GPIO
	PORTE_PCR20 |= PORT_PCR_MUX(1);//GPIO
	PORTE_PCR21 |= PORT_PCR_MUX(1);//GPIO
	PORTE_PCR22 |= PORT_PCR_MUX(1);//GPIO
	
	/*Init PWM signals*/
	/*Enable clock con TPM0/2*/
	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK;
	/*Select clock source*/
	SIM_SOPT2 |= 0x01 << SIM_SOPT2_TPMSRC_SHIFT; //MCGFLLCLK clock
	/*Disable TPM while configuring it*/
	TPM1_SC = TPM_SC_CMOD(0);
	TPM2_SC = TPM_SC_CMOD(0);
	/*TPM Modulo Register */
	TPM1_MOD = 8000; 
	TPM2_MOD = 8000; 
	/*Divide Clock by 2^n*/
	TPM1_SC |= TPM_SC_PS(0);
	TPM2_SC |= TPM_SC_PS(0);
	/*OC toogle mode*/
	TPM1_C0SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; //PWM and Make signal low on match
	TPM1_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; //PWM and Make signal low on match
	TPM2_C0SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; //PWM and Make signal low on match
	TPM2_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; //PWM and Make signal low on match
	/*schedule next transition*/
	TPM1_C0V = 0; //Duty Cycle XA
	TPM1_C1V = 0; //Duty Cycle YA
	TPM2_C0V = 0; //Duty Cycle XB
	TPM2_C1V = 0; //Duty Cycle YB
	/*Enable timer mode*/
	TPM1_SC |= TPM_SC_CMOD(1);
	TPM2_SC |= TPM_SC_CMOD(1);
	
	/*Set GPIO signals as Outputs*/
	GPIOC_PDDR |= 1<<2;
	GPIOE_PDDR |= (1<< 29 | 1<<20 | 1<<21 | 1<<22);
	
	/*Set STBY AIN1 BIN1 as HIGH and AIN2 BIN2 as LOW*/
	GPIOE_PDOR |= (1<<29); //AIN1 HIGH
	GPIOE_PDOR |= (1<<21); //BIN1 HIGH
	GPIOE_PDOR |= (1<<20); //STBY HIGH
	
	
	
}

void PWM_Motor_Duty_Cycle(float xA, float xB, float yA, float yB){
	TPM1_C0V = 4000;//xA;
	TPM1_C1V = 4000;//yA;
	TPM2_C0V = 4000;//xB;
	TPM2_C1V = 4000;//yB;
	
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


