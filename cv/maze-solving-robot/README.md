# Maze Solving Robot
I made a Maze Solving Robot for Maker Faire Detroit at The Henry Ford in 2015. Sorry for the very messy code. It was an early project of mine and I never intended people to actually see the code.

Along with this I also made a program to take a picture of a drawing a student made, let them threshold it how they want, then it would find contours and color those contours randomly.

The goal of the exhibit was to get kids interested in Computer Vision and STEM in general.

This is actually what eventually led to me founding Kinvert.

<img src="https://www.kinvert.com/wp-content/uploads/2017/06/maker-faire-detroit-michigan-computer-vision-lesson-summer-for-kids-coding-1024x683.jpg">

## YouTube Video

[![Discussion Video 1](https://img.youtube.com/vi/stuCPNpmuDA/0.jpg)](https://www.youtube.com/watch?v=stuCPNpmuDA "Discussion Video 1")

## How it Worked
A Raspberry Pi would run this program. It used a RaspiCam.

A red blanket was on the ground, and someone could take blue ribbons and make a maze out of them.

This is from memory, I don't really have time to review the code right now:

- Raspberry Pi takes picture with RaspiCam
- Change the picture to a top down view
- Fill the frame with the known size red blanket
- Have conversions to and from world and pixel coordinates
- Set red as traversable and blue as non traversable
- Run an A* Algorithm to find the best path
- Approximate that A* Algo path with turn angles and drive distances
- Turn that list of moves in to commands
- Send those commands over Serial to basically program the robot
- The robot would be waiting for those commands
- Robot would confirm it got all the commands and turn on a light
- Set down the robot on the start location, pointed in some direction though I forget where it had to point
- Hit a button on the robot
- The robot would then dead reckon the drive based on turn angles and drive distances

For simplicity I think the robot only cared about the center point. If I didn't want the wheels touching I could have done that math, or I could have just dilated the non traversable path by half the robot width.

Again, it was ugly temp code to just show off a robot at Maker Faire. I'm embarassed at the code but proud of the project I pulled off back then. It was a fun one.
