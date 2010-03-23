#include <adc.h>
#include <libpic30.h>
#include "adcgather.h"
#include "constants.h"

volatile int ADCSamples[SENSORS];
volatile int ADCCurrentSample = -1;


void setupADCGather(void){

 OpenADC1( ADC_MODULE_ON &
		   ADC_IDLE_STOP &
           ADC_ADDMABM_ORDER &
   		   ADC_AD12B_12BIT &
		   ADC_FORMAT_INTG &
		   ADC_CLK_AUTO &
		   ADC_MULTIPLE &
		   ADC_AUTO_SAMPLING_OFF &
		   ADC_SAMP_OFF,

		   ADC_VREF_AVDD_AVSS &
		   ADC_SCAN_OFF &
		   ADC_SELECT_CHAN_0  & //This was poorly documentted
           ADC_DMA_ADD_INC_1 &
		   ADC_ALT_BUF_OFF &
		   ADC_ALT_INPUT_OFF,

		   ADC_SAMPLE_TIME_30 &
		   ADC_CONV_CLK_SYSTEM &
		   ADC_CONV_CLK_32Tcy,

		   ADC_DMA_BUF_LOC_1,

		   ENABLE_AN0_ANA & 
   		   ENABLE_AN1_ANA & 
		   ENABLE_AN2_ANA & 
		   ENABLE_AN3_ANA & 
		   ENABLE_AN4_ANA & 
		   ENABLE_AN5_ANA & 
	   	   ENABLE_AN9_ANA,
		   ENABLE_ALL_DIG_16_31,


		   SCAN_NONE_0_15,
		   SCAN_NONE_16_31);

 //Setup the interrupts
 ConfigIntADC1(ADC_INT_ENABLE & ADC_INT_PRI_6);

 ADCCurrentSample = -1;

}

void startADCGather(void){
  ADCCurrentSample = 0;
  EMITTER_ODD = 1;
  __delay_us(50);

  SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN &
 			    ADC_CH0_POS_SAMPLEA_AN0);

  AD1CON1bits.SAMP = 1;
}

void joinADCGather(void){
 while( ADCCurrentSample != -1 );
}

void __attribute__((interrupt, auto_psv)) _ADC1Interrupt(void){
 //Clear the IF
 IFS0bits.AD1IF = 0;


 /*This is a state machine that works as follows
Odd On
Read Even
Even Off
Read Odd
Odd Off
Read Even
Even On
Read Odd	

Tricky things:
Always want to do On value - Off value to get positive number
Channels are AN0-AN5 but sample 6 (starting at zero) is on channel AN9
First two steps are in startADCGather
Odd means sensors 1,3,5
Even means sensos 0,2,4,6
 */
 if( ADCCurrentSample == 0 ){
    ADCSamples[0] = ReadADC1(0); 
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN2));
 }
 else if( ADCCurrentSample == 1 ){
    ADCSamples[2] = ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN4));
 }
 else if( ADCCurrentSample == 2 ){
    ADCSamples[4] = ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN9));
 }
 else if( ADCCurrentSample == 3 ){
    ADCSamples[6] = ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN1));
    EMITTER_EVEN = 0;
	__delay_us(50);
 } 
 else if( ADCCurrentSample == 4 ){
    ADCSamples[1] = ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN3));
 }
 else if( ADCCurrentSample == 5 ){
    ADCSamples[3] = ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN5));
 }
 else if( ADCCurrentSample == 6 ){
    ADCSamples[5] = ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN0));
    EMITTER_ODD = 0;
	__delay_us(50);
 }
 else if( ADCCurrentSample == 7 ){
    ADCSamples[0] -= ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN2));
 }
 else if( ADCCurrentSample == 8 ){
    ADCSamples[2] -= ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN4));
 }
 else if( ADCCurrentSample == 9 ){
    ADCSamples[4] -= ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN9));
 }
 else if( ADCCurrentSample == 10 ){
    ADCSamples[6] -= ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN1));
    EMITTER_EVEN = 1;
	__delay_us(50);
 }
 else if( ADCCurrentSample == 11 ){
    ADCSamples[1] -= ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN3));
 }
 else if( ADCCurrentSample == 12 ){
    ADCSamples[3] -= ReadADC1(0);
 	SetChanADC1(0, (ADC_CH0_NEG_SAMPLEA_VREFN) &
 				    (ADC_CH0_POS_SAMPLEA_AN5));
 }
 else if( ADCCurrentSample == 13 ){
    ADCSamples[5] -= ReadADC1(0);	
    ADCCurrentSample = -1;
 } 

 if (ADCCurrentSample != -1){
   AD1CON1bits.SAMP = 1;
   ADCCurrentSample++;
 }
}
