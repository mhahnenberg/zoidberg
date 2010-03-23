#ifndef ADCGATHER_H
#define ADCGATHER_H

//ADC Interrupt Handlers and management
extern volatile int ADCSamples[7];
extern volatile int ADCCurrentSample;

//Initializes the ADC unit and all needed info
void setupADCGather(void);
//Start the first ADC sampling
void startADCGather(void);
//Wait until this round of sampling is done
void joinADCGather(void);
//As one sample finishes, trigger the next one
void __attribute__((interrupt, auto_psv)) _ADC1Interrupt(void);   

#endif
