#ifndef CONSTANTS_H
#define CONSTANTS_H

#define OUTLED LATBbits.LATB14
#define USER_BUTTON PORTBbits.RB4

#define EMITTER_EVEN LATBbits.LATB9
#define EMITTER_ODD LATBbits.LATB8

#define TRUE 1
#define FALSE 0

#define SAMPLES 200
#define NUM_STDDEVS 4
#define TEST_POINTS 9
#define AVG_CALIBRATE_POINT 5
#define ERROR_MARGIN 50
#define DECISION_TEST_POINTS 4
#define DECISION_THRESHOLD (0.5)
#define FINAL_STAGE_NUM 1
#define SENSORS 7

#define NUM_CALIS 2

#define CENTER_SETPOINT 3.0

//Go straight if sensor values beyond this range
#define UNKNOWN_LOW_CUTOFF -2
#define UNKNOWN_HIGH_CUTOFF 17

//Percent per millisecond
#define SPEED_STEP 0.001
#define START_SPEED_PERCENT 0.0
#define MAX_SPEED_PERCENT 0.95

//(x)/dt is number of milliseconds to average over
#define MOTOR_ALPHA ((2.0)/(1.0 + (18.0/dt)))

//Locomotion gains 
#define KP 0.365
#define KI 0.0
#define KD 0.0

#endif
