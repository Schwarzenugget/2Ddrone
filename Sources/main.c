/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "derivative.h" /* include peripheral declarations */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/******************************************************************************
* Pin assigment MKL25Z128
******************************************************************************/

/*
	* Y:PWMA - PTB0
	* Y:PWMB - PTB1
	* X:PWMA - PTB2
	* X:PWMB - PTB3
	* AIN1 - PTC2
	* AIN2 - PTE29 (PTC1 is not properly soldered)
	* STBY - PTE20
	* BIN1 - PTE21
	* BIN2 - PTE22
	* PTA14 - Interrupt Acc
	* PTE24 - I2C Acc
	* PTE25 - I2C Acc
	* PTB18 - Red LED
	* PTB19 - Green LED
	* PTA1 - UART 
	* PTA2 - UART
	*/

/******************************************************************************
* Main
******************************************************************************/  

int main (void)
{
	/**************
	* Accelerometer Init
	***************/
	AccReady = 0;
	MCU_Init();
  	Accelerometer_Init();
  	Calibrate();  
  	
  	/**************
  	* PWM Init
  	**************/
  	PWM_LED_Init(); //deprecated
  	PWM_Motors_Init();
  	
  	/**************
  	* Serial Communication Init
  	***************/
  	PrintReady = 0;
  	int count = 0;
  	ConsoleIO_Init();
	SysTick_Init();
	
	/**************
	PID init
	**************/
	Axis2D Input_g; 
	Axis2D Target_g = {0.0,0.0};
	Motors Output_pwm;
	PID XaxisA, XaxisB, YaxisA, YaxisB;
	/*Specify initial tuning parameters*/
	float Kp=50000, Ki=0, Kd=0; //0,2000,400
	/* Init Axis Y PID*/
	PID_Init(&XaxisA, &Input_g.X, &Output_pwm.XA, &Target_g.X, Kp, Ki, Kd, P_ON_E,DIRECT);
	PID_Init(&YaxisA, &Input_g.Y, &Output_pwm.YA, &Target_g.Y, Kp, Ki, Kd, P_ON_E,REVERSE);
	PID_Init(&XaxisB, &Input_g.X, &Output_pwm.XB, &Target_g.X, Kp, Ki, Kd, P_ON_E,REVERSE);
	PID_Init(&YaxisB, &Input_g.Y, &Output_pwm.YB, &Target_g.Y, Kp, Ki, Kd, P_ON_E,DIRECT);
	
	while(!AccReady){}; //Read Accelerometer to init PID
	AccReadValues(&Input_g.X, &Input_g.Y);
  	AccReady = 0;
  	
  	/*Turn On PID*/
  	PID_SetMode(&XaxisA, AUTOMATIC);
  	PID_SetMode(&XaxisB, AUTOMATIC);
	PID_SetMode(&YaxisA, AUTOMATIC);
  	PID_SetMode(&YaxisB, AUTOMATIC);
  	
	
  	while(1)
    {

		if (AccReady)		// Is a new set of data ready? 
		{  		
			AccReady = 0;
																	
			AccReadValues(&Input_g.X, &Input_g.Y);

			PID_Compute(&XaxisA); //PID computing
			PID_Compute(&YaxisA); //PID computing
			PID_Compute(&XaxisB);
			PID_Compute(&YaxisB);
			
			//PWM_LED_Duty_Cycle(Input_g.Y);
			PWM_Motor_Duty_Cycle(Output_pwm.XA, Output_pwm.XB, Output_pwm.YA, Output_pwm.YB);
						
		} 
		if (PrintReady){   //Time to print?
			PrintReady = 0;
			count++;
			printf("%d,%.6f,%.6f\r\n", count, Input_g.X, Input_g.Y);
			
		}
		
	}
}

/******************
 * PRINTF INTERRUPT
 *****************/

void SysTick_Init(){
	
	SYST_CSR = 0x1; //Enable Timer
	SYST_CSR |= 0 << SysTick_CSR_CLKSOURCE_SHIFT; //System clock when 1, divided by 16 when 0
	SYST_CSR |= 0x1 << SysTick_CSR_TICKINT_SHIFT; //Enable Interruptions
	SYST_RVR = 419400; //Timer start value
}

void SysTick_Handler(void){
	PrintReady = 1;
}



