# MNIST From Scratch
I have used different methods here to properly classify hand written letters using MNIST data.

## MNIST.cpp 94.5% 2022
After making this work in a Jupyter Notebook, and getting it to work with Cozmo, I ported the code to C++.

One difficulty was the lack of a Normal Distribution in Eigen. I found the Box-Muller Transform and used that.

## mnist.ipynb 98% 2022
This is the first one I got working. It uses a Jupyter Notebook so it is more interactive and GitHub can display it well.
 
I first got it working with Keras as a benchmark, then built a system from scratch to handle MNIST.

## cozmo-mnist.py ~50% 2022
Using code from my Jupyter Notebook, I started making changes so I could use a robot with a camera to take pictures of handwritten numbers.

I had to do more Data Augmentation to get this to work.

With more effort in consisten lighting, better Data Augmentation to better emulate camera noise etc, and some more epochs, I should be able to get this over 75% fairly quickly.

The robot is the Anki Cozmo, which is one of the robots my company Kinvert used to teach until March 2020.