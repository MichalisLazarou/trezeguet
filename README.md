# trezeguet
Coursework 2 - Brushless motor

MISTAKE IN THE SUBMITTED CODE: ki term of the PID controller that is set as a global variable should be 0.01 instead of -0.01, it was changed as we were testing in the last few minutes before submission and could not be submitted correctly in time we were meant to submit ki=0.01, terribly sorry for the inconvenience. This variable is responsible for stopping the motor, being negative makes the motor slow down and if it doesn't stop it will accelerate again and keep rotating infinitely.

How code works:

Main function:
