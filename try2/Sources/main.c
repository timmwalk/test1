/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* Firefly 2015 Halloween hurray! */
#include "MK22F51212.h"
#include <string.h>


static long unsigned int i = 1;

static long unsigned int j = 1;
static long unsigned int outer = 1;
static int val;
static int *ptr;



#define BLUE_LED_PIN_NUM 	5
#define PWM_PIN_NUM 		0
#define MOD_COUNT			105 //209 for 100kHz
#define CNV_MIN				1		/* for LED off */
#define CNV_MAX				( MOD_COUNT * 65 / 100 )/* LED max bright */  //was 60
#define CNV_INC				1

static void wait ( unsigned int count );

int main(void)
{
	typedef enum {
		UP,
		HOLD_ON,
		HOLD_OFF,
		DOWN
	} state_type;

	state_type state;
	int cnv;
	int stateCount;


	outer = 1;
	state = UP;
	stateCount = 0;

    /* This for loop should be replaced. By default this loop allows a single stepping. */
    i = 1;
    cnv = CNV_MIN;
    val = 0;
    ptr = 0;

    /* turn all ports' clocks on */
    SIM->SCGC5 |= SIM_SCGC5_PORTA(1) | SIM_SCGC5_PORTB(1) | SIM_SCGC5_PORTC(1) | SIM_SCGC5_PORTD(1);
    /* turn on FTM1 */
    SIM->SCGC6 |= SIM_SCGC6_FTM1(1);

    /* set MUX for PortD pin 5 */
    PORTD->PCR[BLUE_LED_PIN_NUM] = PORT_PCR_MUX( 1 );
    /* set MUX for FTM1 CHNL 0 on PortB pin */
    PORTB->PCR[PWM_PIN_NUM] = PORT_PCR_MUX( 3 );

    /* set GPIO pin DDR to output */
    //PTD->PDDR = 1 << BLUE_LED_PIN_NUM;

    /* PWM */
    FTM1->CONF = 0xC0;  //sets timer to function in BDM mode
    FTM1->FMS = 0x00;	//clear fault register
    FTM1->MODE |= 0X04; //no sync needed for pwm  make 0x05 for FTM rather than

    FTM1->MOD = MOD_COUNT;
    FTM1->CONTROLS[0].CnSC = 0x28; 	//edge alignment init H then becomes L
    FTM1->CONTROLS[0].CnV = cnv;

    FTM1->CNTIN = 0x00;
    FTM1->SC = 0X08;					//SYSTEM clock divide by 1

	//PTD->PCOR = 1 << BLUE_LED_PIN_NUM; //toggle LED

    while ( 1 )
    {
    	i = 0;

    	//PTD->PTOR = 1 << BLUE_LED_PIN_NUM; //toggle LED
    	PTD->PCOR = 1 << BLUE_LED_PIN_NUM; //keep blue LED off

    	if ( state == UP )
			{
				cnv = cnv + CNV_INC + CNV_INC * stateCount;
				if (cnv > CNV_MAX )
				{
					cnv = CNV_MAX;
					state = HOLD_ON;
					stateCount = 0;
				}
			FTM1->CONTROLS[0].CnV = cnv;
			}
		else if ( state == DOWN )
			{
			cnv = cnv - CNV_INC -CNV_INC * stateCount;
			if (cnv < CNV_MIN )
				{
					cnv = CNV_MIN;
					state = HOLD_OFF;
					stateCount = 0;
				}
			FTM1->CONTROLS[0].CnV = cnv;
			}
		else if ( state == HOLD_ON )
			{
			FTM1->CONTROLS[0].CnV = cnv = CNV_MAX;
			wait( 6000 );
			state = DOWN;
			}
		else if ( state == HOLD_OFF)
		{
			FTM1->CONTROLS[0].CnV = cnv = CNV_MIN;
			wait( 12000 );
			state = UP;
		}

		//FTM1->CNTIN = 0x00;
	    //FTM1->SC = 0X08;					//SYSTEM clock divide by 1

    	wait( 600 );
    	stateCount++;
    }
    /* Never leave main */
    return 0;
}


static void wait ( unsigned int count )
{

unsigned int ii, jj;

for ( ii = 0; ii < count; ii++ )
{
	for ( jj = 0; jj < 100; jj++ );
}

}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
