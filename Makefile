# MPLAB IDE generated this makefile for use with GNU make.
# Project: Mobot.mcp
# Date: Tue Oct 21 17:57:16 2008

AS = pic30-as.exe
CC = pic30-gcc.exe
LD = pic30-ld.exe
AR = pic30-ar.exe
HX = pic30-bin2hex.exe
RM = rm

Mobot.hex : Mobot.cof
	$(HX) "Mobot.cof"

Mobot.cof : main.o
	$(CC) -mcpu=33FJ64MC802 "main.o" "C:\Program Files (x86)\Microchip\MPLAB C30\lib\dsPIC33F\libp33FJ64MC802-coff.a" -o"Mobot.cof" -Wl,-L"C:\Program Files\Microchip\MPLAB C30\lib",--script="default.gld",--defsym=__MPLAB_BUILD=1,--heap=512,-Map="Mobot.map",--report-mem

main.o : ../../../../program\ files\ (x86)/microchip/mplab\ c30/bin/bin/../../support/dsPIC33F/h/p33FJ64MC802.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/bin/bin/../../support/dsPIC33F/h/p33Fxxxx.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/bin/bin/../../support/peripheral_30F_24H_33F/adc.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/bin/bin/../../include/stdio.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/bin/bin/../../include/math.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/bin/bin/../../include/yvals.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/bin/bin/../../include/stdlib.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/bin/bin/../../support/generic/h/dsp.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/bin/bin/../../support/dsPIC33F/h/p33FJ64MC802.h ../../../../program\ files\ (x86)/microchip/mplab\ c30/bin/bin/../../support/dsPIC33F/h/p33fxxxx.h main.c
	$(CC) -mcpu=33FJ64MC802 -x c -c "main.c" -o"main.o" -g -Wall

clean : 
	$(RM) "main.o" "Mobot.cof" "Mobot.hex"

