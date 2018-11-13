# Self-positioning Remote Control Car
HKU ELEC3442 Embedded System - Group Project 

## Objectives

This project aims to produce a self-positioning remote control car with the following designed features:

1. Remote controlling
2. Positioning and navigating
3. Obstacle detection and collision avoidance
4. Image transmission and display

The deliverables would be a car with camera, a remote controller that can control the moving of the car, a positioning system based on visual identification, and a interactive image transmission system based on web technology. 

The product can be used as a simple RC car, a moving photographic platform, or a transporting robot for industry application.

## Scope

### Hardware
* A car was designed and assembled by our team (circuit design and implementation and physical construction of the car.)
* Raspberry Pi as the controlling unit. 
* Remote controller is based on the Sense Hat together with a Raspberry Pi
* Positioning system is based on camera and Raspberry Pi ([AprilTags](https://april.eecs.umich.edu/media/pdfs/olson2011tags.pdf))

### Software
* Mainly based on [Robot Operating System (ROS)](http://www.ros.org/)
* Focused on the controlling logic design and software implementation
* Positioning system aims to implement the more precise positioning technology for indoor/small-range use (industry application rather than the approximate public positioning service)
