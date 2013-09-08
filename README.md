N64ToUSB
========

This is an Arduino Uno Implementation of converting N64 Controller serial data to USB, using Darran Hunt's Big Joystick firmware.

Michele Perla's assembly code is used to communicate to the controller over the serial line, which is
then translated into a data packet for Hunt's BigJoystick arduino firmware.  Only a single controller
was implemented, but multiple controllers could be added with ease.  If you do add another controller,
take care to fine-tune the loop() delay, as the 8U2 chip on the Uno can get overwhelmed.
