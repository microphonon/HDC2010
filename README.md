# HDC2010
 
Demo code to continuously read and display temperature and humidity data from Texas Instruments HDC2010 using the MSP430F5529 or MSP40FR5969 Launchpad.
Device is periodically polled by the MCU in on-demand mode. No temp or humidity threshold interrupts.
Low-level I2C communication using UCB0 registers, interrupts, and MCU LPM0 mode. Display data on terminal program.
Temperature resolution: 14-bit displayed with 4 significant figures.
Humidity resolution: 11-bit displayed with 3 significant figures. Ground the ADR pin to set slave address at 0X40.
Sensor is briefly heated at startup/reset; heater draws 90 mA at 3.3V.
Main loop runs with timed interrupt from LPM3 and VLO clock. I2C clock: 100 kHz using 10k pullup resistors; UART: 9600 baud.
IDE with CCS 6.1.3 and nofloat printf support.

<p>Configuration registers for the Fx5xx and FR59xx families are not compatible; code is in the corresponding folders.

<p>Mouser product video: https://www.youtube.com/watch?v=p8QBYTFYhUg
