#include "derivative.h" 
#include "I2C.h"

/*
 * TODO: substituir els caracters "??" per el registre d'E/S del modul I2C. Tambe al fitxer I2C.h
 * TODO: substituir "Especificar primer pas comunicacio" per la primera funcio del protocol I2C
 * TODO EXTRA: implementar funcio write multiple registers.
 */


void I2C_WriteRegister(unsigned char u8SlaveAddress, unsigned char u8RegisterAddress, /*unsigned*/ char u8Data)
{
	I2C_Start();
	
	// TODO afegir operacio. Macros definides a I2C.h	        
	I2C0_D = u8SlaveAddress << 1;									/* Send I2C device address with W/R bit = 0 */
	I2C_Wait();
	
	// TODO afegir operacio. Macros definides a I2C.h	

	I2C0_D = u8RegisterAddress;										/* Send register address */
	I2C_Wait();

	I2C0_D = u8Data;												/* Send the data */
	I2C_Wait();

	I2C_Stop();

    Pause(50);
}

void I2C_WriteMultipleRegister(unsigned char u8SlaveAddress, unsigned char u8RegisterAddress, unsigned char n, char *w)
{
	uint32_t i;
	
	I2C_Start();
	
	// TODO afegir operacio. Macros definides a I2C.h	          
	I2C0_D = u8SlaveAddress << 1;									/* Send I2C device address with W/R bit = 0 */
	I2C_Wait();
	
	// TODO afegir operacio. Macros definides a I2C.h	

	I2C0_D = u8RegisterAddress;										/* Send register address */
	I2C_Wait();
	
	for(i=0; i<n; i++) {
		I2C0_D = *w;
		w++;															/* Send the data */
		I2C_Wait();
	}
	
	I2C_Stop();

    Pause(50);
}

unsigned char I2C_ReadRegister(unsigned char u8SlaveAddress, unsigned char u8RegisterAddress)
{
	unsigned char result;
	  
	I2C_Start();
	
	// TODO afegir operacio. Macros definides a I2C.h		          
	I2C0_D = u8SlaveAddress << 1;									/* Send I2C device address with W/R bit = 0 */
	I2C_Wait();
	
	// TODO afegir operacio. Macros definides a I2C.h										

	I2C0_D = u8RegisterAddress;										/* Send register address */
	I2C_Wait();

	I2C_RepeatedStart();
	
	I2C0_D = (u8SlaveAddress << 1) | 0x01;							/* Send I2C device address this time with W/R bit = 1 */ 
	I2C_Wait();	

	I2C_EnterRxMode();
	I2C_DisableAck();

	result = I2C0_D;												
	I2C_Wait();
	I2C_Stop();  
	result = I2C0_D;
	Pause(50);
	return result;
}

void I2C_ReadMultiRegisters(unsigned char u8SlaveAddress, unsigned char u8RegisterAddress, unsigned char n, unsigned char *r)
{
	char i;
	
	I2C_Start();

	// TODO afegir operacio. Macros definides a I2C.h	          
	I2C0_D = u8SlaveAddress << 1;									/* Send I2C device address with W/R bit = 0 */
	I2C_Wait();
	
	// TODO afegir operacio. Macros definides a I2C.h									

	I2C0_D = u8RegisterAddress;										/* Send register address */
	I2C_Wait();

	I2C_RepeatedStart();
		
	I2C0_D = (u8SlaveAddress << 1) | 0x01;							/* Send I2C device address this time with W/R bit = 1 */ 
	I2C_Wait();	

	I2C_EnterRxMode();
	I2C_EnableAck();
	
	i = I2C0_D;
	I2C_Wait();
	
	for(i=0; i<n-2; i++) 
	{
	    *r = I2C0_D;
	    r++;
	    I2C_Wait();
	}
	
	I2C_DisableAck();
	*r = I2C0_D;
	r++;
	I2C_Wait();
	I2C_Stop();
	*r = I2C0_D; 
	Pause(50);		
}

void Pause(int number)
{
	int cnt;
	for(cnt=0; cnt<number; cnt++)
	{
		asm("nop");
	};
}
