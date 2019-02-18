# FrequencyAndDutyMeter
Code for the frequency and duty meter of square wave signals.

This code has been written the frequency and duty cycle meter of 3.3 V square waves based on timing of their pulse width and period. 
The timing of both is measured by counting the number of 48 MHz CPU clock cycles of the Atmel SAMD21 microcontroller installed in 
Adafruit's ItsyBitsy M0 Express board. (https://www.adafruit.com/product/3727). It may or may not work with other MCUs featuring 
the ARM Cortex M0 processor.

This project's hardware and further details are described in the following Instructable: 
https://www.instructables.com/id/How-to-Measure-High-Frequency-and-Duty-Cycle-Simul/ 

For further instructions look inside the .ino file. You are welcome to use this code in your projects free of any monetary charge, 
but please do mention its source when posting results of your work.
