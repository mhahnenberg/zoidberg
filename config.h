#ifndef CONFIG_H
#define CONFIG_H

//IMPORTANT: FCY MUST be defined as a compiler macro
//Current value is typically 40Mhz

#define BAUDRATE 19200
#define BRGVAL ((FCY/BAUDRATE)/16)-1

//Configuration Bits, uses AND masks
_FOSCSEL(FNOSC_FRC)  //Start with the internal oscilattor with no PLL
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_HS) //Enable Clock Switching
_FWDT(FWDTEN_OFF & WINDIS_OFF) //No Watchdog plx
_FGS(GSS_OFF & GCP_OFF & GWRP_OFF) //No code protection

#endif
