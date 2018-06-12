/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/
#if 0
	#if ARDUINO >= 100
	  #include "Arduino.h"
	#else
	  #include "WProgram.h"
	#endif
#endif

#include <PID.h>


/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void PID_Init(PID* PID, REAL* Input, REAL* Output, REAL* Setpoint,
        REAL Kp, REAL Ki, REAL Kd, int POn, int ControllerDirection)
{
	PID->myOutput = Output;
	PID-> myInput = Input;
	PID->mySetpoint = Setpoint;
	PID->inAuto = 0;

    PID_SetOutputLimits(PID, 0, 8000);				//default output limit corresponds to
												//the TPM module counter limit

    PID->SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    PID_SetControllerDirection(PID, ControllerDirection);
    PID_SetTunings(PID, Kp, Ki, Kd, POn);

    //PID->lastTime = millis-PID->SampleTime; //TODO Check if it works
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
char PID_Compute(PID* PID)
{
   if(!PID->inAuto) return 0;
   //unsigned long now = millis;
   //unsigned long timeChange = (now - PID->lastTime);
   //if(timeChange>=PID->SampleTime)
   //{
      /*Compute all the working error variables*/
      REAL input = *(PID->myInput);
      REAL error = *(PID->mySetpoint) - input;
      REAL dInput = (input - PID->lastInput);
      PID->outputSum+= (PID->ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!PID->pOnE) PID->outputSum-= PID->kp * dInput;

      if(PID->outputSum > PID->outMax) PID->outputSum= PID->outMax;
      else if(PID->outputSum < PID->outMin) PID->outputSum= PID->outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
	   REAL output;
      if(PID->pOnE) output = PID->kp * error;
      else output = 0;

      /*Compute Rest of PID Output*/
      output += PID->outputSum - PID->kd * dInput;

	    if(output > PID->outMax) output = PID->outMax;
      else if(output < PID->outMin) output = PID->outMin;
	    *(PID->myOutput) = output;

      /*Remember some variables for next time*/
	  //PID->lastInput = input;
	  //PID->lastTime = now;
	    return 1;
   //}
   //else return 0;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID_SetTunings(PID* PID, REAL Kp, REAL Ki, REAL Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   PID->pOn = POn;
   PID->pOnE = POn == P_ON_E;

   PID->dispKp = Kp; PID->dispKi = Ki; PID->dispKd = Kd;

   REAL SampleTimeInSec = ((REAL)PID->SampleTime)/1000;
   PID->kp = Kp;
   PID->ki = Ki * SampleTimeInSec;
   PID->kd = Kd / SampleTimeInSec;

  if(PID->controllerDirection ==REVERSE)
   {
	  PID->kp = (0 - PID->kp);
	  PID->ki = (0 - PID->ki);
	  PID->kd = (0 - PID->kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID_SetTunings_lr(PID* PID, REAL Kp, REAL Ki, REAL Kd){
    PID_SetTunings(PID, Kp, Ki, Kd, PID->pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID_SetSampleTime(PID* PID, int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      REAL ratio  = (REAL)NewSampleTime
                      / (REAL)PID->SampleTime;
      PID->ki *= ratio;
      PID->kd /= ratio;
      PID->SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SetOutputLimits(PID* PID, REAL Min, REAL Max)
{
   if(Min >= Max) return;
   PID->outMin = Min;
   PID->outMax = Max;

   if(PID->inAuto)
   {
	   if(*(PID->myOutput) > PID->outMax) *(PID->myOutput) = PID->outMax;
	   else if(*(PID->myOutput) < PID->outMin) *(PID->myOutput) = PID->outMin;

	   if(PID->outputSum > PID->outMax) PID->outputSum= PID->outMax;
	   else if(PID->outputSum < PID->outMin) PID->outputSum= PID->outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID_SetMode(PID* PID, int Mode)
{
    char newAuto = (Mode == AUTOMATIC);
    if(newAuto && !PID->inAuto)
    {  /*we just went from manual to auto*/
    	PID_ManualToAuto_Init(PID);
    }
    PID->inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID_ManualToAuto_Init(PID* PID)
{
	PID->outputSum = *(PID->myOutput);
	PID->lastInput = *(PID->myInput);
   if(PID->outputSum > PID->outMax) PID->outputSum = PID->outMax;
   else if(PID->outputSum < PID->outMin) PID->outputSum = PID->outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_SetControllerDirection(PID* PID, int Direction)
{
   if(PID->inAuto && Direction !=PID->controllerDirection)
   {
	   PID->kp = (0 - PID->kp);
	   PID->ki = (0 - PID->ki);
	   PID->kd = (0 - PID->kd);
   }
   PID->controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
REAL PID_GetKp(PID* PID){ return  PID->dispKp; }
REAL PID_GetKi(PID* PID){ return  PID->dispKi;}
REAL PID_GetKd(PID* PID){ return  PID->dispKd;}
int PID_GetMode(PID* PID){ return  PID->inAuto ? AUTOMATIC : MANUAL;}
int PID_GetDirection(PID* PID){ return PID->controllerDirection;}


