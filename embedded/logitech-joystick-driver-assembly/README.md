# Logitech Extreme 3D Pro Joystick Driver

I did this over Christmas Break in 2010-2011. It uses assembly code on an embedded microcontroller. I did some edits later to release it for download on Parallax's OBEX, which they later moved to GitHub.

https://github.com/parallaxinc/propeller/tree/master/libraries/community/p1/All/Logitech%20Extreme%203D%20Pro%20Driver

I wrote the Logitech Demo. I did not write FullDuplexSerial or the ADC Driver.

## The Project
Basically I wanted a tool to control robots etc. I was using a microcontroller called the Parallax Propeller which can't take in USB.

So I took apart a Logitech Extreme 3D Pro and sort of hacked it to work.

There are other versions where I had used the driver to make bigger things. For example I used this when I made custom control boards for Kerbal Space Program.

## How it Worked
What was interesting, and I don't recall exactly what was going on, but there were fewer wires than buttons.

If I remember correctly it didn't just leave a wire high and wait to get a signal at the other end when the button was pressed.

Rather there were 3 phases it would pulse out. It took a while to reverse engineer how they did it. I had to use my USB Oscilloscope to figure out how it worked.

Then I wrote assembly code to duplicate it.

Hopefully before too long I can open one of these up and document more clearly how it worked on the hardware side.

The ADC for the 4 axes was much more straight forward.

## Used In Several Projects
I used this many times. Wirelessly driving robots was a common theme. I could hook an XBEE up.

Another cool project I briefly mentioned was Kerbal Space Program. This was part of the custom control board to fly simulated rockets. The buttons could be programmed, and the ADC channels were also used for Roll, Pitch, Yaw, and Throttle.

A few other people used it in projects which is pretty cool. So the instructions were clear enough they were able to get it to work.

## Video
https://www.youtube.com/watch?v=XJqs5wlxxEY