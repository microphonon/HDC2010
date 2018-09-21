
The eUSCI_B module supports hardware byte counting and automatic stop bit generation for I2C.  The module must be reconfigured for different payload sizes, so it was simpler to perform these functions in software. The MSP430FR5969 Launchpad that this code was developed on was unable to supply enough current for the sensor heater (90mA at 3.3V).   The jumpers were changed for an external 3.3V power supply and the heater worked OK. Simply remove one line of code to prevent pulsed heating on reset.  Launchpad pins configured as follows:

<p>P1.6  UCB0SDA with 10k pullup
<br>P1.7  UCB0SCL with 10k pullup
<br>P2.6  UCA1TXD
<br>P2.5  UCA1RXD
