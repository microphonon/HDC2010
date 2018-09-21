#include <msp430.h> 
#include <stdio.h>
#include <stdint.h>

/*
Demo code to continuously read and display temperature and humidity data from TI HDC2010 using MSP430FR5969 Launchpad.
Device is periodically polled by the MCU in on-demand mode. No temp or humidity threshold interrupts.
Low-level I2C communication using registers, interrupts, and MCU LPM0 mode. Display data on terminal program.
Temperature resolution: 14-bit and displayed with 4 significant figures.
Humidity resolution: 11-bit displayed with 3 significant figures. Ground the ADR pin to set slave address at 0X40.
Sensor is briefly heated at startup/reset; heater draws 90 mA at 3.3V.  There is not enough current available
on the FR5969 Launchpad to supply the heater; use an external 3,3V power supply if heater function is implemented.
Main loop runs with timed interrupt from LPM3 and VLO clock. I2C clock 100 kHz; UART 9600 baud.
IDE with CCS 6.1.3 and nofloat printf support.

P1.6  UCB0SDA with 10k pullup
P1.7  UCB0SCL with 10k pullup
P2.6  UCA1TXD
P2.5  UCA1RXD

September 2018
 */
# define PERIOD 10000 //Samping period. 10000 count is approximately 1 second; maximum is 65535
# define HEATER 20000 //Pulse heater at reset. 20000 count is approximately 2 seconds

void SetTimer(void);
void SetVLO(void);
void SetPins(void);
void SetUART(void);
void SetI2C(void);
void Measure(void);
void GetData(void);
void Heater(void);
void GetID(void);

//Variables for I2C communication
volatile uint8_t *PTxData;   // Pointer to TX data
volatile uint8_t TXByteCtr;
volatile uint8_t *PRxData;  // Pointer to RX data
volatile uint8_t RXByteCtr;
volatile uint8_t RxBuffer[4];   // Allocate 4 bytes of RAM for data

//Variables for UART terminal display
char str[80];
volatile uint32_t T,TC,H,HC;

volatile uint8_t i,count;

void main(void) {

    WDTCTL = WDTPW | WDTHOLD;	//Stop watchdog timer

    SetPins();
    SetVLO();
    SetTimer();
    SetUART();
    SetI2C();

    _BIS_SR(GIE); //Enable global interrupts

    GetID();
    //Trap CPU if sensor ID incorrect
    if(*(PRxData+3)!=0x49)
    {
    	P1OUT |= BIT0;
    	while(1){}
    }
    /*	Comment the following line to prevent pulsed heating after reset.
   	CAUTION: Heater cannot be powered by VCC of the FR5969 Launchpad. */
    Heater(); 

    while(1)
    {
    	TA0CCR0 = PERIOD; //Looping period with VLO
    	LPM3;		//Wait in low power mode
    	P1OUT |= BIT0; //Timeout: Processing started; turn on green LED on Launchpad

    	UCB0IE |= UCTXIE0 + UCRXIE0; //Enable TX and RX I2C interrupts
    	Measure(); //Initiate measurement.
    	//Wait in LPM3 until data becomes available.  Polling DRDY wastes power
    	TA0CCR0 = 12; //May have to adjust this delay depending on sensor
    	LPM3;
    	//Timeout; fetch the data
    	GetData();	//Get T-H data
    	UCB0IE &= ~(UCRXIE0 + UCTXIE0); //Disable I2C interrupts

  	//Process 16-bit raw temperature data
   	T = ((uint16_t)(*(PRxData+2)) << 8)|(uint16_t)*(PRxData+3);
    	//Convert temperature data
   	TC = (((T<<14) + (T<<8) - (T<<7) - (T<<3) - (T<<2)) >> 16) - 0xFA0; //No decimal point

    	//Process 16-bit raw humidity data
        H = (uint16_t)(*PRxData << 8)|(uint16_t)*(PRxData+1);
        //Convert temperature data
        HC = ((H<<10) - (H<<4) - (H<<3)) >> 16; //No decimal point

        sprintf(str,"%s %lu.%.2lu%s %lu.%.1lu%s", "Temp:", (int32_t)(TC/100),(int32_t)(TC%100),"C Rel Humidity:",
        		(int32_t)(HC/10),(int32_t)(HC%10),"%\r\n\n");
    	count = sizeof str;
    	for (i=0; i < count; i++)
    		{
    	     	 while (!(UCA1IFG & UCTXIFG)); //Poll serial: Is TX buffer empty?
    	     	 UCA1TXBUF = str[i]; //Send data 1 byte at a time
    	     }
    	P1OUT &= ~BIT0; //Turn off green LED
    	}
}

#pragma vector=TIMER0_A0_VECTOR
 __interrupt void timerfoo (void)
{
	LPM3_EXIT;
}

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
	switch(__even_in_range(UCB0IV,30))
	{
	    case 0: break;         // Vector 0: No interrupts
	    case 2: break;         // Vector 2: ALIFG
	    case 4: break;          // Vector 4: NACKIFG
	    case 6: break;         // Vector 6: STTIFG
	    case 8: break;         // Vector 8: STPIFG
	    case 10: break;         // Vector 10: RXIFG3
	    case 12: break;         // Vector 12: TXIFG3
	    case 14: break;         // Vector 14: RXIFG2
	    case 16: break;         // Vector 16: TXIFG2
	    case 18: break;         // Vector 18: RXIFG1
	    case 20: break;         // Vector 20: TXIFG1
	    case 22:                 // Vector 22: RXIFG0
	    	 RXByteCtr--;        // Decrement RX byte counter
	    	 if (RXByteCtr) //Execute the following if counter not zero
	    	 	 {
	    		 	 *(PRxData + RXByteCtr) = UCB0RXBUF; // Move RX data to address PRxData
	    			 if (RXByteCtr == 1)     // Only one byte left?
	    			 UCB0CTL1 |= UCTXSTP;    // Generate I2C stop condition BEFORE last read
	    	 	 }
	    	 else
	    	 	 {
	    		 	 *PRxData = UCB0RXBUF;   // Move final RX data to PRxData(0)
	    		 	 LPM0_EXIT; 		// Exit active CPU
	    	 	 }
	    	 break;
	    case 24:       		// Vector 24: TXIFG0
	    	if (TXByteCtr)      // Check if TX byte counter not empty
	   	    	{
	    			UCB0TXBUF = *PTxData++; // Load TX buffer
	    			TXByteCtr--;            // Decrement TX byte counter
	   	    	}
	   	    else
	   	    	{
	   	    		UCB0CTL1 |= UCTXSTP;        // I2C stop condition
	   	    		UCB0IFG &= ~UCTXIFG0;        // Clear USCI_B0 TX int flag
	   	    		LPM0_EXIT; 		// Exit LPM0
	   	    	}
	   	    break; 
	    case 26:  break;        // Vector 26: BCNTIFG
	    case 28: break;         // Vector 28: clock low timeout
	    case 30: break;         // Vector 30: 9th bit
	    default: break;
	}
}

 void SetPins(void)
  {
	PM5CTL0 &= ~LOCKLPM5; //Unlocks GPIO pins at power-up

 	/* Port 1
 	P1.0 Green LED
 	P1.1 Launchpad switch
 	P1.6 SDA I2C
 	P1.7 SCL I2C
 	 */
 	P1DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5;
 	P1SEL1 |= BIT6 + BIT7; //Setup I2C on UCB0
 	P1OUT &= ~BIT0; //LED off

 	/* Port 2
 	P2.1  Button on Launchpad
 	P2.5 TXD UART
 	P2.6 RXD UART
 	*/
 	P2DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT7;
 	P2SEL1 |= BIT5 + BIT6; //Setup UART on UCA1

	/* Port 3 */
 	P3DIR |=  BIT0 + BIT1 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;

 	/* Port 4
 	P4.6 Red LED
 	*/
 	P4DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
 	P4OUT &= ~BIT6; //LED off
  }

 void SetVLO(void)
 {
	 CSCTL0 = CSKEY; //Password to unlock the clock registers
	 //Default frequency ~ 10 kHz
	 CSCTL2 |= SELA__VLOCLK;  //Set ACLK to VLO
	 CSCTL0_H = 0xFF; //Re-lock the clock registers
    }

 void SetTimer(void)
 {
	 //Enable the timer interrupt, MC_1 to count up to TA0CCR0, Timer A set to ACLK (VLO)
	 TA0CCTL0 = CCIE;
	 TA0CTL |= MC_1 + TASSEL_1;
 }

 void SetUART(void) //UCA1 module; do polling instead of interrupts
  {
 	 UCA1CTLW0 |= UCSWRST;
 	//Next line selects SMCLK which is DCO (default frequency: 1 MHz)
 	 UCA1CTLW0 |=  UCSSEL1; //This writes 0x80 which sets BIT7
 	 //Next two lines divide 1 MHz to get 9600 baud
 	 UCA1BRW = 0x06;
 	 UCA1MCTLW |= UCOS16 + UCBRF3 + UCBRS5;
 	 UCA1CTLW0 &= ~UCSWRST;
  }

 void SetI2C(void)
   {
  	 // Configure the eUSCI_B0 module for I2C at 100 kHz
	 UCB0CTLW0 |= UCSWRST;
	 //Select SMCLK, Master, synchronous, I2C
	 UCB0CTLW0 |=  UCSSEL__SMCLK + UCMST + UCSYNC + UCMODE_3;
	 UCB0BRW = 10; 	//Divide SMCLK by 10 to get ~100 kHz
	 UCB0I2CSA = 0x40; // HDC2010 address; ADR grounded
	 UCB0CTLW0 &= ~UCSWRST; // Clear reset
   }

 void Heater(void)
   {
	 const uint8_t HeaterOn[] = {0x0E,0x08}; //Heater activation command
	 UCB0IE |= UCTXIE0; //Enable TX interrupt
	 UCB0CTL1 |= UCTR;  //Set as transmitter
  	 PTxData = (uint8_t *)HeaterOn;
  	 TXByteCtr = 2;
  	 UCB0CTL1 |= UCTXSTT;
  	 LPM0;       // MCU Remain in LPM0 until all data transmitted
  	 while (UCB0CTL1 & UCTXSTP);  // Ensure stop condition got sent

  	 P4OUT |= BIT6; //Heater is on. Turn on red LED on Launchpad
  	 TA0CCR0 = HEATER; //Heater on time.
  	 LPM3;		//MCU waits in low power mode

  	 const uint8_t HeaterOff[] = {0x0E,0x00}; //Heater deactivation
  	 PTxData = (uint8_t *)HeaterOff;
  	 TXByteCtr = 2;
  	 UCB0CTL1 |= UCTXSTT;
  	 LPM0;
  	 while (UCB0CTL1 & UCTXSTP);
  	 UCB0IE &= ~UCTXIE0; //Disable TX interrupt
  	 P4OUT &= ~BIT6; //Turn off red LED
   }

 void Measure(void)
  {
	 //Initiates a temp-humid measurement with 14- and 11-bit resolution, respectively
 	 const uint8_t MS[] = {0x0F,0x11};
 	 UCB0CTL1 |= UCTR;  //Set as transmitter
 	 PTxData = (uint8_t *)MS;      // TX array start address
 	 TXByteCtr = 2;              // Load TX byte counter
 	 UCB0CTL1 |= UCTXSTT;   // Start condition
 	 LPM0;                   // Remain in LPM0 until all data transmitted
  }


 void GetData(void)
 {
	 const uint8_t ReadData[] = {0x00}; //First byte address; read will auto-increment
	 UCB0CTL1 |= UCTR;  //Set as transmitter
	 PTxData = (uint8_t *)ReadData;      // TX the array start address
	 TXByteCtr = 1;              // Load TX byte counter
	 UCB0CTL1 |= UCTXSTT;   // I2C start condition
	 LPM0;                   // Remain in LPM0 until all data transmitted
	 while (UCB0CTL1 & UCTXSTP);  // Ensure stop condition got sent
	 //Receive 4 data bytes
	 UCB0CTL1 &= ~UCTR; //Set as receiver
	 PRxData = (uint8_t *)RxBuffer;    // Start of RX buffer
	 RXByteCtr = 4;
	 UCB0CTL1 |= UCTXSTT;
	 LPM0;
 }

 void GetID(void)
 {
	 const uint8_t ReadID[] = {0xFC};
	 UCB0IE |= UCTXIE0 + UCRXIE0; //TX and RX interrupts
	 UCB0CTL1 |= UCTR;  //Set as transmitter
	 PTxData = (uint8_t *)ReadID;      // TX the array start address
	 TXByteCtr = 1;              // TX byte counter
	 UCB0CTL1 |= UCTXSTT;   // Start condition
	 LPM0;                   // Remain in LPM0 until all data transmitted
	 while (UCB0CTL1 & UCTXSTP);  // Ensure stop condition occurred
	 //Read the 4 data bytes
	 UCB0CTL1 &= ~UCTR; //Set as receiver
	 PRxData = (uint8_t *)RxBuffer;    // Start of RX buffer
	 RXByteCtr = 4; // RX byte counter
	 UCB0CTL1 |= UCTXSTT; // I2C start condition
	 LPM0;
	 while (UCB0CTL1 & UCTXSTP);
	 UCB0IE |= ~(UCTXIE0 + UCRXIE0);
 }
