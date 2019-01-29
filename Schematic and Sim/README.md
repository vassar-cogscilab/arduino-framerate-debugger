The schematic was created using the Falstad Analog Circuit Simulator Applet for easy of sharing and communication. 

To view the schematic and circuit simulation:

Download the text file in this folder. 
Go to: http://falstad.com/circuit/circuitjs.html
Click: File > Open > (location of saved text file)


Simulation notes:
Allow simulation several minutes to and stabilize threshold and upper limit voltage trigger levels. 
Lowpass filters used in trigger circuits utilize large capactors which need to be charged. 
The simulation speed rate is significatly lower than live testing speeds. Live adjustments will stablize in about 3 seconds. 

The simulated waveform is a very rough approximation of a photocell output measuring a single ON frame with two OFF frames from a 60Hz display.
The signal is created with a 20Hz 30% duty cycle pulse at 100mV. 
Noise from brightness settings is created with a 300Hz triagle wave at 500mV and can be toggled on/off.
True measured photocell outputs have a DC offset based on ambient light and more rounded waves. 
The simulated output accurately represents the circuit's affect on a similar wave structure, but is only intended as a conceptual aid. 


Usage notes:
Amplification should be adjusted to maximum gain without clipping. 
A peak between 90%-95% of supply voltage is ideal for resolution and increased signal to noise ratio (4.5V-4.75V on a 5V supply).
Setting the upper limit warning LED trigger level to 96% visually aids this adjustment (4.8V on a 5V supply). 

Brightness settings below 100% are acheived with a 300Hz PWM of the frame output on the monitor used for development and testing. 
This noise can be removed with the adjustable filter at the cost of some pulse width accuracy. 

The filter range can be changed by adding/removing a 2.2k ohm resistor in parallel with the 10k ohm potentiometer. 
Adding the parallel resistor decreases the total impedence of the RC lowpass filter, thereby raising the max filter frequency from 72Hz to 402Hz. 
This should be used with any input signals >100Hz

Output threshold levels should be set to aproximately 1/4-1/2 the signal's floor. 
In testing, this has been aproximately 1.45V (or 75/255 on PWM). 

