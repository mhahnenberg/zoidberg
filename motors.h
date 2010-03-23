#ifndef MOTORS_H
#define MOTORS_H

#define MOTOR_PWM_PERIOD 0x80

//The brake/INA pin
#define MOTOR_BRAKE_LEFT LATAbits.LATA4
#define MOTOR_BRAKE_RIGHT LATBbits.LATB7

//The brake/INB pin
#define MOTOR_DIR_LEFT LATBbits.LATB5
#define MOTOR_DIR_RIGHT LATBbits.LATB6

//Mode info for motorSet command
#define MOTOR_MODE_COAST 0
#define MOTOR_MODE_BRAKE 1

//Function prototypes

//Initializes what is needed for motor output
void motorSetup(void);

//Set the motor output left/right, also the braking mode
// -1 to 1
void motorSet(double left, double right, char mode);


#endif
