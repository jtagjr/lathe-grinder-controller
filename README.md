This project is for use on my K.O. Lee surface grinder. It receives input from the user via a stop button, start button and 2 rotary encoder switches. It displays output on a four character LED display. These buttons can start, stop, and change the speed of a servo motor by sending messages over the Modbus to a servo motor controller. The user can set the speed (RPMs) of the motor via the rotary encoder switches. The speed is displayed on the LED display.

Includes a branch from ModbusMaster branch that ports to Mbed OS. Updated to use UnbufferedSerial for the ModbusMaster class and several changes to fix receiving data from the servo controller.

Includes a branch from the TM1637 library for controlling the LED device. Added an additional write function and lookup table.
>>>>>>> b33b3667e80a7615cbf2eaba0f20433d38780041
