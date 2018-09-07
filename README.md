# HDC2010
HDC2010.c  Demo code to continuously read and display temperature and humidity data from TI HDC2010 using MSP430F5529 Launchpad.
Device is periodically polled by the MCU in on-demand mode. No temp or humidity threshold interrupts.
Low-level I2C communication using registers, interrupts, and MCU LPM0 mode. Display data on terminal program.
Temperature resolution: 14-bit and displayed with 4 significant figures.
Humidity resolution: 11-bit displayed with 3 significant figures. Ground the ADR pin to set slave address at 0X40.
Sensor is briefly heated at startup/reset; heater draws 90 mA at 3.3V
Main loop runs with timed interrupt from LPM3 and VLO clock. I2C clock 100 kHz; UART 9600 baud.
IDE with CCS 6.1.3 and nofloat printf support.

P3.0  SDA with 10k pullup
P3.1  SCL with 10k pullup
P3.3  TXD
P3.4  RXD
