# gleepbot
Programs for Gleepbot. 

# PID 
Test programs to learn how to use a PID loop.

## encoder_read
Reads from the encoder via the interrupt. Use the serial plotter to see the position realtime.

## encoder_read_servo
Reads from the encoder via the interrupt and puts the motor to a specified position. 

See `int target = 200;` in `loop()` to change the position the motor will rotate to.

Modify the PID constants to change how the motor responds to wheel position changes and how fast it tries to get to the target position.