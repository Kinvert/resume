# Construction Robots - 2017
I managed this one rather than coding. The students did most of this.

Basically there was a big table with plastic beads and these robots would move them around.

- Front End Loader
- Dump Truck
- Bulldozer

While doing the event Dave from MatterHackers dropped by which was cool.

https://youtu.be/EbC444fI4sk?t=244

## How it Worked
There was a central board with a Parallax Propeller. Several joysticks were hooked up to ADCs. There were some buttons too, for example at least one was to control the dump truck bucket.

This would be interpreted by Construction_3.spin, and the signal was sent out via serial using XBEE.

The robots would all get the same commands. So they'd have to know what to ignore, and what to obey.

I don't remember exactly what happened, but for a while there was an unexpected byte coming in sometimes. We were in a high noise environment at Maker Faire and we did have several XBEE transmittors. So the students made a junk byte version to handle that unexpected issue.

I actually still don't know exactly what was going on with that. I wish I wasn't so busy at the event it would have been a fun one to troubleshoot.