ps
==

Physics-Sensor Simulation Code

This routine communicates with the HC (Host Controller) and MC (Master Controller) programs, using CAN bus compatible ascii/format messages, and simulates the operation of a glider winch, including the physics of a launch, such as the aerodynamics of the glider from ground roll through recovery via parachute.  

The implementation of glider winch focuses on sensors, e.g. cable tension sensing, and other units, e.g. the CAN bus controlled motor controller, using the CAN bus for communication.  The physics simulation receives CAN messages and sends CAN messages which simulates the operation of the winch.

The routine uses MATLAB for the main computations and Java for the communication.



