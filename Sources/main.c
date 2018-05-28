/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "derivative.h" /* include peripheral declarations */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ConsoleIO.h"


/******************************************************************************
* Global variables
******************************************************************************/
unsigned char AccData[6];
short Xout_14_bit, Yout_14_bit, Zout_14_bit;
Axis2D Input_g; // Xout_g, Yout_g, 
float Zout_g;
char AccReady, PrintReady;
char Xoffset, Yoffset, Zoffset;
char DebugBuffer[64];


/******************************************************************************
* Main
******************************************************************************/  

int main (void)
{
	/**************
	* Accelerometer
	***************/
	AccReady = 0;
	MCU_Init();
  	Accelerometer_Init();
  	Calibrate();  
  	
  	/**************
  	* PWM
  	**************/
  	PWM_LED_Init();
  	PWM_Motors_Init();
  	
  	/**************
  	* UART Comm
  	***************/
  	int count = 0;
  	ConsoleIO_Init();
	PrintReady = 0;
	FTPM0_Init();
	/**************
	PID
	**************/
	//PID Xaxis; Not used in single axis demo
	PID Yaxis;
	/*Define Variables we'll be connecting to*/
	Axis2D Target_g = {0.0,0.0}, Output_pwm; //Input_g previously defined

	/*Specify initial tuning parameters*/
	float Kp=6000, Ki=100, Kd=0; //Random numbers
	/* Init Axis Y PID*/
	PID_Init(&Yaxis, &Input_g.Y, &Output_pwm.Y, &Target_g.Y, Kp, Ki, Kd, P_ON_E,REVERSE);
	
	while(!AccReady){}; //Read Accelerometer to init PID
	AccReadValues();
  	AccReady = 0;
  	
  	/*Turn On PID*/
  	PID_SetMode(&Yaxis, AUTOMATIC);
	
  	while(1)
    {
		if (AccReady)		// Is a new set of data ready? 
		{  		
			AccReady = 0;
																	
			AccReadValues();
			
			PID_Compute(&Yaxis); //PID computing
			
			PWM_LED_Duty_Cycle(Input_g.Y);
			PWM_Motor_Duty_Cycle(Output_pwm.Y, 0);
						
		} 
		if (PrintReady){
			PrintReady = 0;
			count++;
			printf("%d,%.6f,%.6f\r\n", count, Input_g.X, Input_g.Y);
			
		}
		
	}
}

/******************************************************************************
* MCU initialization function
******************************************************************************/ 

void MCU_Init(void)
{
	//I2C0 module initialization
	SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;		// Turn on clock to I2C0 module 
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;		// Turn on clock to Port E module 
	PORTE_PCR24 = PORT_PCR_MUX(5);			// especificar mux (PTE24 pin is I2C0 SCL line) 
	PORTE_PCR25 = PORT_PCR_MUX(5);			// especificar mux (PTE25 pin is I2C0 SDA line)
	I2C0_F  = 0x14; 						// SDA hold time = 2.125us, SCL start hold time = 4.25us, SCL stop hold time = 5.125us *
	I2C0_C1 = I2C_C1_IICEN_MASK;    		// Enable I2C0 module 
	
	//Configure the PTA14 pin (connected to the INT1 of the MMA8451Q) for falling edge interrupts
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;		// Turn on clock to Port A module 
	PORTA_PCR14 |= (0|PORT_PCR_ISF_MASK|	// Clear the interrupt flag 
					  PORT_PCR_MUX(0x1)|	// PTA14 is configured as GPIO 
					  PORT_PCR_IRQC(0xA));	// PTA14 is configured for falling edge interrupts 
	
	//Enable PORTA interrupt on NVIC
	NVIC_ICPR |= 1 << ((INT_PORTA - 16)%32); 
	NVIC_ISER |= 1 << ((INT_PORTA - 16)%32); 
}


/******************************************************************************
* Accelerometer initialization function
******************************************************************************/ 

void Accelerometer_Init (void)
{
	unsigned char reg_val = 0;  
	
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG2, 0x40);		// Reset all registers to POR values
	
	do		// Wait for the RST bit to clear 
	{
		reg_val = I2C_ReadRegister(MMA845x_I2C_ADDRESS, CTRL_REG2) & 0x40; 
	} 	while (reg_val);
	
	/*
	 * The measured acceleration data is stored in the OUT_X_MSB, OUT_X_LSB, OUT_Y_MSB, OUT_Y_LSB, OUT_Z_MSB, 
	 * and OUT_Z_LSB registers as 2’s complement 14-bit numbers. The most significant 8-bits of each axis are 
	 * stored in OUT_X (Y, Z)_MSB, so applications needing only 8-bit results can use these 3 registers and ignore 
	 * OUT_X,Y, Z_LSB. To do this, the F_READ bit in CTRL_REG1 must be set. When the F_READ bit is cleared, the fast 
	 * read mode is disabled. 
	 */
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, XYZ_DATA_CFG_REG, 0x00);		// +/-2g range -> 1g = 16384/4 = 4096 counts 
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG2, 0x02);		// High Resolution mode 
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, 0x1D);	// 0x5 ODR = 800Hz Fast mode , Reduced noise, Active mode	
}

/******************************************************************************
* Simple offset calibration
******************************************************************************/ 

void Calibrate (void)
{
	unsigned char reg_val = 0;
	
	while (!reg_val)		// Wait for a first set of data		 
	{
		reg_val = I2C_ReadRegister(MMA845x_I2C_ADDRESS, STATUS_REG) & 0x08; 
	} 	
	  	
	I2C_ReadMultiRegisters(MMA845x_I2C_ADDRESS, OUT_X_MSB_REG, 6, AccData);		// Read data output registers 0x01-0x06  
	  						
	Xout_14_bit = ((short) (AccData[0]<<8 | AccData[1])) >> 2;		// Compute 14-bit X-axis output value
	Yout_14_bit = ((short) (AccData[2]<<8 | AccData[3])) >> 2;		// Compute 14-bit Y-axis output value
	Zout_14_bit = ((short) (AccData[4]<<8 | AccData[5])) >> 2;		// Compute 14-bit Z-axis output value
	  					
	Xoffset = Xout_14_bit / 8 * (-1);		// Compute X-axis offset correction value
	Yoffset = Yout_14_bit / 8 * (-1);		// Compute Y-axis offset correction value
	Zoffset = (Zout_14_bit - SENSITIVITY_2G) / 8 * (-1);		// Compute Z-axis offset correction value
	  					
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, 0x00);		// Standby mode to allow writing to the offset registers	
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, OFF_X_REG, Xoffset);		
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, OFF_Y_REG, Yoffset);	
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, OFF_Z_REG, Zoffset);	
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG3, 0x00);		// Push-pull, active low interrupt 
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG4, 0x01);		// Enable DRDY interrupt 
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG5, 0x01);		// DRDY interrupt routed to INT1 - PTA14 
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, 0x1D);		// ODR = 800Hz Fast mode , Reduced noise, Active mode		
}

void AccReadValues(){
	I2C_ReadMultiRegisters(MMA845x_I2C_ADDRESS, OUT_X_MSB_REG, 6, AccData);		// Read data output registers 0x01-0x06 
	            
	Xout_14_bit = ((short) (AccData[0]<<8 | AccData[1])) >> 2;		// Compute 14-bit X-axis output value
	Yout_14_bit = ((short) (AccData[2]<<8 | AccData[3])) >> 2;		// Compute 14-bit Y-axis output value
	Zout_14_bit = ((short) (AccData[4]<<8 | AccData[5])) >> 2;		// Compute 14-bit Z-axis output value
	
	Input_g.X = ((float) Xout_14_bit) / SENSITIVITY_2G;		// Compute X-axis output value in g's
	Input_g.Y = ((float) Yout_14_bit) / SENSITIVITY_2G;		// Compute Y-axis output value in g's
	Zout_g = ((float) Zout_14_bit) / SENSITIVITY_2G;		// Compute Z-axis output value in g's
}

/***************************************************************** *************
* PORT A Interrupt handler
******************************************************************************/ 

void PORTA_IRQHandler()
{
	//flag de interrupcio
	PORTA_PCR14 |= PORT_PCR_ISF(1);
	//avisar al bucle principal
	AccReady = 1;
}



/******************
 * PRINTF INTERRUPT
 *****************/

void FTPM0_Init(){
	/*Enable clock con TPM0*/
	SIM_SCGC6 |= 1 << SIM_SCGC6_TPM0_SHIFT; 
	/*Select clock source*/
	SIM_SOPT2 |= 0x01 << SIM_SOPT2_TPMSRC_SHIFT; //MCGFLLCLK clock
	/*Disable TPM while configuring it*/
	TPM0_SC = 0x0;
	/*TPM Modulo Register*/
	TPM0_MOD = 0xD000;
	/*Clear Timer Overflow Flag*/
	TPM0_SC |= 0x1 << 7;
	/*Divide Clock by 2^6*/
	TPM0_SC |= 0x7;
	/*Enable timeout interrupt*/
	TPM0_SC |= 0x01 << TPM_SC_TOIE_SHIFT;
	/*Enable Timer Up-counter timer mode*/
	TPM0_SC |= 0x01 << 3; //Clock Mode Selection
	
	/*Enable IRQ17 interrupt*/
	NVIC_ISER |= NVIC_ISER_SETENA(0x20000);
	
}

void FTM0_IRQHandler(void){
	/*Clear Timer Overflow Flag*/
	TPM0_SC |= 0x1 << 7;
	PrintReady = 1;
	//AccReady = 1;
	
}

