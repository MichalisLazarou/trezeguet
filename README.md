# trezeguet
Coursework 2 - Brushless motor

MISTAKE IN THE SUBMITTED CODE: ki term of the PID controller that is set as a global variable should be 0.01 instead of -0.01, it was changed as we were testing in the last few minutes before submission and could not be submitted correctly in time. We were meant to submit ki=0.01, terribly sorry for the inconvenience. This variable is responsible for stopping the motor, being negative makes the motor slow down and if it doesn't stop it will accelerate again and keep rotating infinitely.

How to test the code:

In the main function, in the while(1) loop there are some commented instructions. If you want to test the speed of the motor, uncomment the instruction pc.printf("timer: %f\n\r", speed);, Timer essentially printes the speed as rotations per second and they are displayed in the coolTerm win execution window.

If you want to check the rotations per second uncomment the instruction pc.printf("counter: %i\n\r", counterA);
This instruction prints on the screen the number of rotations completed.
