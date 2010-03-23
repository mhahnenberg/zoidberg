#include "calibrate.h"
#include <stdio.h>
#include <p33fxxxx.h>
#include <libpic30.h>
#include <dsp.h>
#include "matrix.h"
#include "constants.h"
#include "adcgather.h"
#include "math.h"

void surface_calibrate(double *mean, double *stddev_samples){
	double delta;
	double m2[SENSORS];
	int samples[SENSORS];
	int i, j, l, n;

	OUTLED = 1;

	printf("Surface Calibrate\r\n");

	/* wait for button press */
	block_for_user();

	/* start read sensors */
	startADCGather();
	joinADCGather();

	n = 0;

	for (i = 0; i < SENSORS; i++){
		mean[i] = 0.0;
		m2[i] = 0.0;
	}

	for (i = 0; i < 3; i++){
		printf("Position %d/%d:\r\n", i+1,3);

		/* wait for button press */
		block_for_user();	

		/* start read sensors */
		startADCGather();
		joinADCGather();

		for (j = 0; j < SAMPLES; j++) {
				//Copy the gathered ADC samples so we can work with them without interference
			    for (l = 0; l < SENSORS; l++) {
			      samples[l] = ADCSamples[l];
			 	}
			
			    startADCGather();
		
				/* Online variance calculation -- don't ask me how this works, ask Knuth */
				n++;
				for (l = 0; l < SENSORS; l++){
					delta = samples[l] - mean[l];
					mean[l] += delta/(double)n;
					m2[l] += delta*(samples[l] - mean[l]);
				}
			    joinADCGather();
			}
	}

	for( i = 0; i < SENSORS; i++)
		stddev_samples[i] = sqrt(m2[i]/(double)(n-1));


	/* notify user that there is nothing left to do */
	OUTLED = 0;

	printf("double _avg[SENSORS] = {%f, %f, %f, %f, %f, %f, %f};\r\n",
		mean[0],
		mean[1],
		mean[2],
		mean[3],
		mean[4],
		mean[5],
		mean[6]);	

	printf("double _stddevs[SENSORS] = {%f, %f, %f, %f, %f, %f, %f};\r\n", 
		stddev_samples[0],
		stddev_samples[1],
		stddev_samples[2],
		stddev_samples[3],
		stddev_samples[4],
		stddev_samples[5],
		stddev_samples[6]);

}

/* decision point calibration version 2.0 -- sensor window method */
void decision_calibrate(double *mean, double *stddev_samples) {
	double delta, m2, samples[SENSORS];
	int i, j, k, l, n;

	OUTLED = 1;

	/* fill the all_samps array and calculate the mean for each turned on sensor */
	for (i = 0; i < SENSORS; i++) {
		printf("Decision Calibrate--Sensor: %d \r\n", i);

		/* wait for button press */
		block_for_user();
	
		/* start read sensors */
		startADCGather();
		joinADCGather();

		/* initialize the stuff for the variance calculation */
		n = 0;
		mean[i] = 0.0;
		m2 = 0.0;
		
		// for each point on the line
		for (k = 0; k < 3; k++) {
			printf("Position %d\r\n", k);
			/* wait for button press */
			block_for_user();

			for (j = 0; j < SAMPLES; j++) {
				//Copy the gathered ADC samples so we can work with them without interference
			    for (l = 0; l < SENSORS; l++) {
			      samples[l] = ADCSamples[l];
			 	}
			
			    startADCGather();
		
				/* Online variance calculation -- don't ask me how this works, ask Knuth */
				n++;
				delta = samples[i] - mean[i];
				mean[i] += delta/(double)n;
				m2 += delta*(samples[i] - mean[i]);
	
			    joinADCGather();
			}
		}
		
		stddev_samples[i] = sqrt(m2/(double)(n-1));
	}

	/* notify user that there is nothing left to do */
	OUTLED = 0;

	printf("double sensor_stddevs[SENSORS] = {%f, %f, %f, %f, %f, %f, %f};\r\n", 
		stddev_samples[0],
		stddev_samples[1],
		stddev_samples[2],
		stddev_samples[3],
		stddev_samples[4],
		stddev_samples[5],
		stddev_samples[6]);

	printf("double sensor_means[SENSORS] = {%f, %f, %f, %f, %f, %f, %f};\r\n",
		mean[0],
		mean[1],
		mean[2],
		mean[3],
		mean[4],
		mean[5],
		mean[6]);
}

void block_for_user() {
    unsigned char cur = USER_BUTTON;
    int i;

	OUTLED = 0;
    for( i = 0; i < 5; i++ )
	  __delay_ms(100);
	OUTLED = 1; 
 	
	while(USER_BUTTON==cur);
}
