#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1


//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1

#define REAL float

struct PID
{
	
	REAL dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	REAL dispKi;				//   format for display purposes
	REAL dispKd;				//
    
	REAL kp;                  // * (P)roportional Tuning Parameter
    REAL ki;                  // * (I)ntegral Tuning Parameter
    REAL kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

    REAL *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    REAL *myOutput;             //   This creates a hard link between the variables and the 
    REAL *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	REAL outputSum, lastInput;

	unsigned long SampleTime;
	REAL outMin, outMax;
	char inAuto, pOnE;
}typedef PID;

//commonly used functions **************************************************************************
void PID_Init(PID*, REAL*, REAL*, REAL*,        // * constructor.  links the PID to the Input, Output, and 
	REAL, REAL, REAL, int, int);//   Setpoint.  Initial tuning parameters are also set here.
									  //   (overload for specifying proportional mode)


void PID_SetMode(PID*, int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

char PID_Compute(PID*);                       // * performs the PID calculation.  it should be
									  //   called every time loop() cycles. ON/OFF and
									  //   calculation frequency can be set using SetMode
									  //   SetSampleTime respectively

void PID_SetOutputLimits(PID*, REAL, REAL); // * clamps the output to a specific range. 0-255 by default, but
														  //   it's likely the user will want to change this depending on
														  //   the application
	


//available but not commonly used functions ********************************************************
void PID_SetTunings_lr(PID*, REAL, REAL,       // * While most users will set the tunings once in the 
				REAL);         	    //   constructor, this function gives the user the option
									  //   of changing tunings during runtime for Adaptive control
void PID_SetTunings(PID*, REAL, REAL,       // * overload for specifying proportional mode
				REAL, int);         	  

void PID_SetControllerDirection(PID*, int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
									  //   means the output will increase when error is positive. REVERSE
									  //   means the opposite.  it's very unlikely that this will be needed
									  //   once it is set in the constructor.
void PID_SetSampleTime(PID*, int);              // * sets the frequency, in Milliseconds, with which 
									  //   the PID calculation is performed.  default is 100
									  
										  
										  
//Display functions ****************************************************************
REAL PID_GetKp(PID*);						  // These functions query the pid for interal values.
REAL PID_GetKi(PID*);						  //  they were created mainly for the pid front-end,
REAL PID_GetKd(PID*);						  // where it's important to know what is actually 
int PID_GetMode(PID*);						  //  inside the PID.
int PID_GetDirection(PID*);					  //

//Init Struct;
void PID_ManualToAuto_Init(PID*);


#endif

