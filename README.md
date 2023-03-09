# pluto-transmit-demo
Demonstration of transmitting samples via IIO with the Pluto SDR, including code to
enable the 8x transmit interpolator for lower sample rates.

This is based on the standard IIO demo code from Analog Devices. I commented out the
receive functions, and added code to use the weird IIO device interface to enable
the 8x transmit interpolator that the Pluto implements outside the AD9361 radio,
in the FPGA fabric. This makes it possible to use lower sample rates than the AD9361
can natively accept, which we need for Opulent Voice mode.

In order to make this demo generate some output that could be evaluated visually
on a spectrum analyzer, I replaced the default transmit samples (all zeroes, of
all things!) with a tone that's swept sinusoidally from -20 kHz to +20 kHz.

Here's what that looks like:

https://user-images.githubusercontent.com/5356541/223925492-e374c4c6-6d37-42ef-9d8d-9d4153d3f8e3.mov

