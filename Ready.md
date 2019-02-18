//*********************************************************************
// This code has been written by Robert Fraczkiewicz in December 2018 for
// the frequency and duty cycle meter of 3.3 V square waves based on timing 
// of their pulse width and period. The timing of both is measured by counting 
// the numberof internal 48 MHz clock cycles of the Atmel SAMD21 microcontroller 
// installedin Adafruit's ItsyBitsy M0 Express board. (https://www.adafruit.com/product/3727).
// It may or may not work with other MCUs featuring the ARM Cortex M0 processor.
// 
// This project's hardware and further details are described in the following Instructable:
// https://www.instructables.com/id/How-to-Measure-High-Frequency-and-Duty-Cycle-Simul/
// 
// To implement it on your project, you have to pick the output stream first. If you have 
// Adafruit's monochrome 128x32 OLED SPI display (https://www.adafruit.com/product/661) 
// connected to your ItsyBitsy M0 Express, as described in the Instructable, then leave 
// the OLED_DISPLAY_OUT uncommented. If you want to output measured data into the
// serial stream for debugging purposes, then uncomment SERIAL_OUT, keeping in
// mind that this extra communication will put additional strain on the CPU.
//
// And finally, you are welcome to use this code in your projects free of any monetary
// charge, but please do mention its source when posting results of your work.
// 
