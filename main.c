#include <p33fxxxx.h>
#include "config.h"  //Must be included after p33fxxxx.h but before libpic30.h
#include <libpic30.h>
#include <stdio.h>
#include <libq.h>
#include <math.h>
#include <outcompare.h>
#include <string.h>

#include "adcgather.h"
#include "motors.h"
#include "calibrate.h"
#include "matrix.h"
#include "constants.h"

//Initializes all the periphreals, and makes it ready to start the main processing loop
void mainSetup(void);
void block_for_user(void);
void loadWeights(double *);
void saveWeights(double *);

static const _prog_addressT WEIGHTS_PAGE = 0x15780; //The last page on 128k devices

void loadWeights(double *weights){
 int readData[64];
 _memcpy_p2d16( readData, WEIGHTS_PAGE, 64*sizeof(int));
 memcpy(weights, readData, sizeof(double)*(SENSORS+1));
}

void saveWeights(double *weights){
 int writtenData[64]; 
 memset(writtenData, 0, 64*sizeof(int));
 memcpy(writtenData, weights, sizeof(double)*(SENSORS+1));
	 
 _erase_flash(WEIGHTS_PAGE);
 _write_flash16(WEIGHTS_PAGE, writtenData);
}

#define L 0
#define S 1
#define R 2

int turns[22] = {R, L, S, R, S, R ,S, L, L, S,L,S,R,S,R,S,L,S,L,S,R,S};
void _write_flash_word16(_prog_addressT dst, int dat);

int main() {
	int i, j;
	int samples[SENSORS];
	unsigned char button, seen_line, seen_space, decision;
	unsigned char in_decision;
	int dec_num = 0;
	double decision_remain;

	double center, error, derivError, integralError, lastError;
	double dec_filter, dec_alpha;
	double leftspeed, rightspeed, ema_lspeed, ema_rspeed, maxspeed;

	double dt;

	double max_zscore;
	double left_zscore, right_zscore;
	int max_index;
	double zsum;

	//Linear regression weights and center position averages for main and alternate surfaces   
	//Night concrete
	//double main_weights[SENSORS+1] = {18.956198, -0.005023, 0.000011, -0.000792, 0.001970, 0.001492, 0.001390, 0.014168};
	//double main_avg[SENSORS] = {-525.846497, -299.165833, -908.395020, -1290.236328, -822.292480, -1008.658752, -713.698303};
	
double first_avg[SENSORS] = {-445.001709, -513.806335, -466.066620, -672.973450, -509.750092, -806.531677, -436.698395};
double first_stddevs[SENSORS] = {17.055450, 40.831696, 21.177078, 15.581568, 29.431732, 25.235197, 22.167875};

double second_avg[SENSORS] = {-620.986755, -723.098389, -605.508179, -941.415161, -731.218140, -1106.643188, -625.981873};
double second_stddevs[SENSORS] = {31.663101, 37.803436, 51.807541, 19.172064, 35.450817, 168.128632, 15.314978};

//in room
//double first_avg[SENSORS] = {-1676.604126, -2473.728027, -2085.450439, -3131.261719, -2376.939697, -3574.072510, -1812.640747};
//double first_stddevs[SENSORS] = {37.744209, 61.490601, 112.425003, 128.595932, 149.119324, 178.550217, 38.950939};

//double second_avg[SENSORS] = {-1676.604126, -2473.728027, -2085.450439, -3131.261719, -2376.939697, -3574.072510, -1812.640747};
//double second_stddevs[SENSORS] = {37.744209, 61.490601, 112.425003, 128.595932, 149.119324, 178.550217, 38.950939};

	double *all_avgs[NUM_CALIS] = {first_avg, second_avg};
	double *all_stddevs[NUM_CALIS] = {first_stddevs, second_stddevs};

	double *main_avg;
	double *main_stddevs;

	double all_mses[NUM_CALIS];
	double best_mse;
	int best_mse_index;

	//Decision point calibration
	double sensor_stddevs[SENSORS] = {3.869839, 3.123523, 3.201246, 9.717693, 7.065145, 11.029683, 6.414215}; // radio room 4/14/2009
	double sensor_means[SENSORS] = {-122.903305, -95.223358, -93.184944, -142.496689, -103.891716, -130.915009, -91.071587}; // radio room 4/14/2009

	mainSetup();
	printf( "Process initialized...\r\n");


    button = USER_BUTTON;

    for( i = 0; i < 20; i++ ){
	  __delay_ms(100);
      OUTLED = !OUTLED;
    }

 	
	if( USER_BUTTON != button ){
		surface_calibrate(first_avg, first_stddevs);

		decision_calibrate(sensor_means, sensor_stddevs);
    }
    else{
//        loadWeights(weights);
	}

//Testing setting registers from codes
/*
asm("; Set up a pointer to the location to be written.");
asm("MOV    #tblpage(#0xF80004),W0");
asm("MOV    W0,TBLPAG");
asm("MOV    #tbloffset(#0xF80004),W0");
asm("; Get the new data to write to the configuration register");
asm("MOV    #0x0000,W1");
asm("; Perform the table write to load the write latch");
asm("TBLWTL W1,[W0]");
asm("; Configure NVMCON for a configuration register write");
asm("MOV    #0x4000,W0");
asm("MOV    W0,NVMCON");
asm("; Disable interrupts, if enabled");
asm("PUSH   SR");
asm("MOV    #0x00E0,W0");
asm("IOR    SR");
asm("; Write the KEY sequence");
asm("MOV    #0x55,W0");
asm("MOV    W0,NVMKEY");
asm("MOV    #0xAA,W0");
asm("MOV    W0,NVMKEY");
asm("; Start the programming sequence");
asm("BSET   NVMCON,#0x0F");
asm("; Insert two NOPs after programming");
asm("NOP");
asm("NOP");
asm("; Re-enable interrupts, if required");
asm("POP    SR");
_erase_flash(0x400);*/

	/* initialize variables */
	integralError = 0.0;
	lastError = 0.0;

	maxspeed = START_SPEED_PERCENT;
	ema_lspeed = 0.0;
	ema_rspeed = 0.0;
	decision_remain = 0.0;
	in_decision = FALSE;
	dec_filter = 0.0;
	dec_alpha = (2.0)/(100.0 + 1.0);
	center = 0.0;
	OUTLED = 0;

	while (1) {
		//Calculate number of milliseconds since last update
 		dt = ((((unsigned long)TMR5HLD) << 16) + (unsigned long)TMR4) * (1000./(FCY/8.));
		//printf("%X|%X  %f\r\n", TMR5HLD, TMR4, dt);

 		TMR5HLD = 0x00;
		TMR4 = 0x00;	

	    //Copy the gathered ADC samples so we can work with them without interference
	    for( i = 0; i < SENSORS; i++ ){
	      samples[i] = ADCSamples[i];
	 	}

	    //Start the ADC gathering in the background
	    startADCGather();
	
	//	printf("%d %d %d %d %d %d %d\r\n", samples[0], samples[1], samples[2], samples[3], samples[4], samples[5], samples[6]);

		/* speed ramp up */
		if (maxspeed < MAX_SPEED_PERCENT)
			maxspeed += SPEED_STEP*dt;

		//Pick the best model to use
		for( i = 0; i < NUM_CALIS; i++)
			all_mses[i] = 0.0;

		for( i = 0; i < NUM_CALIS; i++){
			for( j = 0; j < SENSORS; j++ )
				all_mses[i] += (samples[j] - all_avgs[i][j]) * (samples[j] - all_avgs[i][j]);

		//all_mses[i] += ((samples[j]-all_avgs[i][j])/all_stddevs[i][j])*((samples[j]-all_avgs[i][j])/all_stddevs[i][j]);
		}	
		
		best_mse = all_mses[0];
		best_mse_index = 0;

		for( i = 1; i < NUM_CALIS; i++)
		{
			if( all_mses[i] < best_mse ){
				best_mse = all_mses[i];
				best_mse_index = i;
			}
		}	

		main_avg = all_avgs[best_mse_index];
		main_stddevs = all_stddevs[best_mse_index];

		//Calculate the center 
		max_index = 0;
		max_zscore = fabs((samples[0] - main_avg[0])/main_stddevs[0]);

		for (i = 1; i < SENSORS; i++) {
			if (fabs((samples[i]-main_avg[i])/main_stddevs[i]) > max_zscore){
				max_index = i;
				max_zscore = fabs((samples[i]-main_avg[i])/main_stddevs[i]);
			}
		}
/*
		for (i = 0; i < SENSORS; i++) 
			printf("%f ", (double)fabs((samples[i]-main_avg[i])/main_stddevs[i]));

		printf("\r\n");
*/

		center = max_index;
		
		if (max_index > 0)
		  left_zscore = fabs((samples[max_index-1]-main_avg[max_index-1])/main_stddevs[max_index-1]);
		else
		  left_zscore = 0;

		if (max_index < SENSORS - 1)
		  right_zscore = fabs((samples[max_index+1]-main_avg[max_index+1])/main_stddevs[max_index+1]);
		else
		  right_zscore = 0;
		
		if( left_zscore > right_zscore )
			center -= left_zscore / (left_zscore + right_zscore + max_zscore);
		else
			center += right_zscore / (left_zscore + right_zscore + max_zscore);

			
		/* determine if we're at a decision point */
//		decision = FALSE;
//		seen_line = FALSE;
//		seen_space = FALSE;
//		zsum = 0.0;
//		
//		for (i = 0; i < SENSORS; i++) {
//			double z =  fabs((samples[i] - main_avg[i])/main_stddevs[i]);
//			// we see a line
//			if (z > max_zscore/3){
//			  seen_line++;
//			  zsum += z;
//			}
//		}
//
//		decision = zsum > 150;
//		//printf("zsum: %f\r\n", zsum);
//		OUTLED = in_decision;
//
//		if( !in_decision && decision )
//		{
//			in_decision = TRUE;
//			decision_remain = 850;			
//			dec_num++;
//		}
//		
//		decision_remain -= dt;
//
//		if( in_decision && decision_remain <= 0)
//			in_decision = FALSE;		
//
//		if( in_decision ){
//             if( turns[dec_num - 1 % 22] == R )
//				center -= 1.65;
//			 else if( turns[dec_num - 1 % 22] == L)
//				center += 1.65;
//		}
//
//		//Calculate terms for PID 
		error = CENTER_SETPOINT - center;
		integralError = integralError + error*dt;
		derivError = (error - lastError)/dt;
		lastError = error;
		


		//printf("\r\n");
		dec_filter += dec_alpha*(decision - dec_filter);
		
		if (dec_filter > DECISION_THRESHOLD)
			decision = 1;
		else
			decision = 0;


//		printf("Center: %f\r\n", center);

		
		/* set the correct motor speeds to turn and correct */
		if (center < UNKNOWN_LOW_CUTOFF || center > UNKNOWN_HIGH_CUTOFF)
		{
			leftspeed = maxspeed;
			rightspeed = maxspeed;
		}
		else if (error > 0)
		{
			leftspeed = maxspeed;

			rightspeed = maxspeed - KP*fabs(error) - KD*derivError - KI*integralError;

			if (rightspeed < -maxspeed)
              rightspeed = -maxspeed;
		}
		else
		{
			leftspeed = maxspeed - KP*fabs(error) - KD*derivError - KI*integralError;
			if (leftspeed < -maxspeed)
              leftspeed = -maxspeed;

			rightspeed = maxspeed;
		} 

		/* low pass filter for motor speeds */
		ema_lspeed += MOTOR_ALPHA*(leftspeed - ema_lspeed);
		ema_rspeed += MOTOR_ALPHA*(rightspeed - ema_rspeed);		

		motorSet(ema_lspeed, ema_rspeed, MOTOR_MODE_COAST);
	//	motorSet(.95,.95,MOTOR_MODE_COAST);
	//	printf("left: %f\tright: %f\n", (double)leftspeed, (double)rightspeed);

	    //Keep this the last thing in the loop, wait for the ADC to be ready for the next loop
	    joinADCGather();
	}
	return 0;
}

void mainSetup(void){
 //Setup the PLL to get 40 mips operation
 //Assumes a 10mhz crystal, based on Microchip's oscillator document

 // Configure PLL prescaler, PLL postscaler, PLL divisor
 PLLFBD=30; // M = 32
 CLKDIVbits.PLLPOST=0; // N1 = 2
 CLKDIVbits.PLLPRE=0; // N2 = 2

 // Initiate Clock Switch to Primary Oscillator with PLL (NOSC = 0b011)
 __builtin_write_OSCCONH(0x03);
 __builtin_write_OSCCONL(0x01);

 // Wait for Clock switch to occur
 while (OSCCONbits.COSC != 0b011);

 // Wait for PLL to lock
 while(OSCCONbits.LOCK!=1) {};

 //Setup 32 bit timer with 1/8 prescaler to measure time
 T4CONbits.TON = 1;
 T4CONbits.TSIDL = 1;
 T4CONbits.TGATE = 0;
 T4CONbits.T32 = 1;
 T4CONbits.TCKPS = 0b01; //prescaling 1:64
 T4CONbits.TCS = 0;

 //Setup Ports for IO, 1=input, 0=output
 TRISA = 0x0000; 
 TRISB = 0x0000;

 //Setup all the analog inputs as inputs on the TRIS bits
 TRISAbits.TRISA0 = 1; 
 TRISAbits.TRISA1 = 1;
 TRISBbits.TRISB0 = 1;
 TRISBbits.TRISB1 = 1;
 TRISBbits.TRISB2 = 1;
 TRISBbits.TRISB3 = 1;
 TRISBbits.TRISB15 = 1;

 //Setup the user button
 TRISBbits.TRISB4 = 1;

 //Map the UART and OC1/2 periphs
 __C30_UART=1;
 
 U1BRG = BRGVAL;
 U1MODEbits.UARTEN = 1;

 //Unlock the periph mapping
 __builtin_write_OSCCONL(OSCCON & 0xbf);

 RPINR18bits.U1RXR = 11; //PGC2/RP11
 RPOR5bits.RP10R = 0b00011; //PGD2/RP10 = U1TX

 RPOR6bits.RP12R = 0b10010; //OC1
 RPOR6bits.RP13R = 0b10011; //OC2

 //Relock the periph mapping
 __builtin_write_OSCCONL(OSCCON | 0x40);

 //Initiliaze the motors
 motorSetup();

 //Setup ADC and take one sample to initialize the values
 setupADCGather();
 startADCGather();
 joinADCGather(); 
}




