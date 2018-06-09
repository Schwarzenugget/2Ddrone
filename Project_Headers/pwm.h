/*
 * pwm_led.h
 *
 *  Created on: Apr 29, 2018
 *      Author: David
 */

#ifndef PWM_LED_H_
#define PWM_LED_H_

void delay(int);
void PWM_LED_Init();
void PWM_LED_Duty_Cycle(float);
void PWM_Motors_Init();
void PWM_Motor_Duty_Cycle(float, float, float, float);

#endif /* PWM_LED_H_ */



