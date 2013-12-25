/* ********************************************************************************
 * 
 * File: main.c
 * 
 * This file is a combination of RTTY code by Rob Harisson from the Icarus project
 * (http://www.robertharrison.org/svn/), 
 * code from Texas Instruments and code written by Maria Christopoulou (SV1PLE).
 * 
 * TODO : 1) Add more Baud rates. Currenlty only 45.45 is supported
 * 		  2) RTTY decoder	
 *  
 * 
 * *******************************************************************************/

/*****************************************************************************/
/*
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include "stdio.h"
#include "string.h"
#include "usbstk5505.h"
#include "aic3204.h"
#include "PLL.h"
#include "sinewaves.h"


Int16 left_input;
Int16 right_input;
Int16 left_output;
Int16 right_output;


#define SAMPLES_PER_SECOND 48000
#define GAIN_IN_dB         0
#define ASCII 7 // 8 for 8 bit ascii

unsigned long int i = 0;
char message[80];

/*--------------------------------------------------------------------
 * rtty_txbit
 * 
 * Assign frequencies on bits 0,1
 * 
 * -------------------------------------------------------------------*/
  
void rtty_txbit (int bit)
{
  if (bit)
  {
    // Mark
     	for ( i = 0  ; i < SAMPLES_PER_SECOND * 0.022L  ;i++  )
 			{

        		left_output = generate_sinewave_1(2295, 1000);
				right_output = generate_sinewave_2(2295,1000); 
    
     	 		aic3204_codec_write(left_output, right_output);
 			}
  }
  else
  {
    // Space
 	 	for ( i = 0  ; i < SAMPLES_PER_SECOND * 0.022L  ;i++  )
 			{
     		
         	left_output = generate_sinewave_1(2125, 1000); 
     	 	right_output = generate_sinewave_2(2125,1000); 
    
     	 	aic3204_codec_write(left_output, right_output);
 			}
  }
 
 
}


/*--------------------------------------------------------------------
 * rtty_txbyte
 * 
 *  
 * -------------------------------------------------------------------*/

void rtty_txbyte (char c)
{
  /* Simple function to sent each bit of a char to 
   	** rtty_txbit function. 
   	** NB The bits are sent Least Significant Bit first
   	**
   	** All chars should be preceded with a 0 and 
   	** proceded with a 1. 0 = Start bit; 1 = Stop bit
   	**
   	*/
 
  int i;
 
  rtty_txbit (0); // Start bit
 
  // Send bits for for char LSB first	
 
  for (i=0;i<ASCII;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1); 
 
    else rtty_txbit(0);	
 
    c = c >> 1;
 
  }
 
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}


/*--------------------------------------------------------------------
 * rtty_txstring
 * 
 * Assign frequencies on bits 0,1
 * 
 * -------------------------------------------------------------------*/

void rtty_txstring (char * string)
{
 
  /* Simple function to sent a char at a time to 
   	** rtty_txbyte function. 
   	** NB Each char is one byte (8 Bits)
   	*/
 
  char c;
 
  c = *string++;
 
  while ( c != '\0')
  {
    rtty_txbyte (c);
    c = *string++;
  }
}

/* ------------------------------------------------------------------------ *
 *                                                                          *
 *  main( )                                                                 *
 *                                                                          *
 * ------------------------------------------------------------------------ */
void main( void ) 
{
    /* Initialize BSL */
    USBSTK5505_init( );
    
    /* Initialize the Phase Locked Loop in EEPROM */
    pll_frequency_setup(100);

    /* Initialise hardware interface and I2C for code */
    aic3204_hardware_init();
    
    /* Initialise the AIC3204 codec */
	aic3204_init(); 

 
	/* Set sampling frequency in Hz and ADC gain in dB */
    set_sampling_frequency_and_gain(SAMPLES_PER_SECOND, GAIN_IN_dB);

    printf("\nRTTY Transmission\n\n");
    printf( "Mark frequency is 2295Hz and Space frequency is 2125Hz. Carrier Shift 170Hz.\n\n" );
	
    asm(" bclr XF");
   
 		sprintf(message,"CQ CQ CQ DE SV1PLE SV1PLE SV1PLE"); 
		rtty_txstring (message);


   /* Disable I2S and put codec into reset */ 
    aic3204_disable();

    printf( "\n***Program has Terminated***\n" );
    SW_BREAKPOINT;
}

/* ------------------------------------------------------------------------ *
 *                                                                          *
 *  End of main.c                                                           *
 *                                                                          *
 * ------------------------------------------------------------------------ */












