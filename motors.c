#include <p33fxxxx.h>
#include <outcompare.h>
#include <math.h>

#include "motors.h"

void motorSetup(void){
  
  //Setup output compare
  CloseOC1();
  CloseOC2();

  //Configure output compare using Timer 3
  ConfigIntOC1(OC_INT_OFF & OC_INT_PRIOR_5);
  ConfigIntOC2(OC_INT_OFF & OC_INT_PRIOR_5);

  PR3 = MOTOR_PWM_PERIOD;
  T3CONbits.TON = 1;
  T3CONbits.TSIDL = 1;
  T3CONbits.TGATE = 0;
  T3CONbits.TCKPS = 0b10; //prescaling 1:64
  T3CONbits.TCS = 0;
 

  OpenOC1(OC_IDLE_CON & OC_TIMER3_SRC & OC_PWM_FAULT_PIN_DISABLE , 0x00,0x00 );
  OpenOC2(OC_IDLE_CON & OC_TIMER3_SRC & OC_PWM_FAULT_PIN_DISABLE , 0x00,0x00 );

  //Write decent values to the control values
   motorSet(0.0,0.0, MOTOR_MODE_COAST);

}

//Set the motor output, values from -1 to 1
void motorSet(double left, double right, char mode){
    int val;

	if( mode == MOTOR_MODE_BRAKE )
    {
		MOTOR_BRAKE_LEFT = 1;
		MOTOR_BRAKE_RIGHT = 1;

		MOTOR_DIR_LEFT = 1;
		MOTOR_DIR_RIGHT = 1;
    }
    else{
		if( left < 0.0 )
		{
			MOTOR_DIR_LEFT = 1;
			MOTOR_BRAKE_LEFT = 0;
		}
		else
		{
			MOTOR_DIR_LEFT = 0;
			MOTOR_BRAKE_LEFT = 1;
		}

		if( right < 0.0 )
		{
			MOTOR_DIR_RIGHT = 1;
			MOTOR_BRAKE_RIGHT = 0;
		}
		else
		{
			MOTOR_DIR_RIGHT = 0;
			MOTOR_BRAKE_RIGHT = 1;
		}	

		val = fabsf(left) * MOTOR_PWM_PERIOD;

		SetDCOC1PWM(val);


		val = fabsf(right) * MOTOR_PWM_PERIOD;

		SetDCOC2PWM(val);	
	}


}
