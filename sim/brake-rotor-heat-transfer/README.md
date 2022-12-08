# Brake Rotor Heat Transfer - 2012
This simulates heat transfer in brake rotors using Matlab.

Basically you set the geometry of the rotor and say what cell size you want. In this case it's at 1.0mm.

There were many ways I initialized the temperature, but in this case it is just initializing each cell at 1000K.

Then it loops through until temperatures change by less than a threshold.

Basically what it is doing is very similar to convolution. It cycles through each cell, and sees how much heat would transfer in/out with each neighbor.

In this case these are larger equations, and the heat transfer changes on boundary conditions such as on the surface or corners of the rotors.

So more complicated than general Convolution. But similar.

The input is initial values the user sets.

The output is the chart, which I attached as an image.

<img src="https://github.com/Kinvert/resume/blob/main/sim/brake-rotor-heat-transfer/SimpleRotorResult.jpg">
