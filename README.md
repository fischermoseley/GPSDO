# GPS Disciplined Oscillator (GPSDO)
## Motivation
High precision, stable frequency references are particularly hard to make. GPS-based references suffer from high noise due to multipath effects, and crystal-based sources are inexpensive but exhibit signficant frequency drift. Atomic clocks and routine calibration can mitigate these issues, but come with a large price tag and constant maintenance. 

## Process
A GPS Discilipined Oscillator (GPSDO) combines the time-averaged stability of the GPS network with the low-noise characteristics of a Oven-Controlled Crystal Oscillator (OCXO). This is done with a frequency locked loop running on an Arduino Nano, which measures the frequency of an internal OCXO by with reference to an onboard GPS. The GPS module outputs a pulse to mark every new UTC second, which is used to gate a timer on the ATMega328. This is used to calculate the frequency of the crystal, and if it deviates from a perfect 10MHz, then the crystal's VCO input is adjusted with a 16-bit DAC until the signal has been corrected.

## Results
The result of this is a frequency source accurate to within 1 part per billion, or about three seconds out of every hundred years. By using a frequency locked loop instead of a phase locked loop, the difference measurement circuitry is simplified and the total cost of the unit is reduced by 30%.

If you're curious and would like to see more details, there's a more complete writeup on my website [here.](https://www.fischermoseley.com/single-post/2017/05/04/Homebuilt-GPS-Disciplined-Oscillator-Project-Overview)

## Pictures and Schematic

Front of the Oscillator:
![alt text](https://github.com/FischerMoseley/GPSDO/Images/Off.JPG "Front of the Oscillator")

Status Indicator:
![alt text](https://github.com/FischerMoseley/GPSDO/Images/On.JPG "Status Indicator")

Internal Construction:
![alt text](https://github.com/FischerMoseley/GPSDO/Images/Side.JPG "Internal Construction")

System Schematic:
![alt text](https://github.com/FischerMoseley/GPSDO/Images/Schematic.JPG "System Schematic")
