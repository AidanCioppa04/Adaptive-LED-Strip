# smart-LED-project
simple LED project for my desk using arduino with some ambient light sensors

This project, which we are calling the Adaptive LED Strip is a simple to setup and use LED strip capable of adapting to light levels within the room.
The project uses a motion sensor to toggle between on and off, and a light sensor to detect the current light level of the room to adjust the brightness of the strip as necessary

Wiring is as follows

For the Light Sensor\
  -ground to ground\
  -5V to 5V\
  -SLA to SLA\
  -SCR to SCR
  
For the Motion Sensor\
  -ground to ground\
  -5V to 5V\
  -data line to pin 2
  
For LED strip\
  -ground to ground\
  -5V to 5V\
  -data line to pin 9

It is reccomended to run the ground and 5V from the arduino onto a breadboard and attach the sensors via the breadboard as there are not enough 5V/GND pins on the arduino to accomodate for the three peripherals

After wiring everything together, simply upload the code Base.ino onto the arduino board and the project will run

## Modifications

If you are looking for an implementation where the motion sensor instead toggles between colors on the board, the code in ColorModelProtoypes will provide a simple implementation. With minimal code adjustments and an additional motion sensor, both of these features could be added to the same implementation as well.

change NUM_LEDS in at the top of the code to adjust how many LEDs are used on the strip

LED color is easily changeable by referencing the FastLED wiki and changing the color in the functions
