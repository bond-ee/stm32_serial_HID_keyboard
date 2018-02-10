# Serial to USB HID Keyboard Passthrough using the STM32

The beginnings of the project can be found on my [website](https://www.bond-ee.com/serial-to-hid-keyboard-passthrough).   

## What is it used for?

For when you dont have a usb keyboard around, but you do have a laptop and a STM32 discovery board. It all started when I was working on a project involving a Raspberry Pi, the login password had been forgotten so SSH was out and I didnt have a Keyboard laying around!  
## How do I used it?

If you have a STM32F469 Disovery board the hex file can be used directly, if not you can set your board up using CubeMX (details on the setup can be found on my website link above), then adding in the necessary .h and .c files. 
 
Then open up your favourite terminal program, connect to the serial port and type away! (Baudrate 115200)

On OSX, search for the serial port

`` ls /dev/tty.* ``

Then connect using screen

`` screen <enter serial port here> 115200 ``
	
There are a few quirks, Upper and lower case letters work exactly the same as well as the number keys, the arrows keys have been moved to the 'wasd' keys, while holding down the option key;
- w->∑->up
- a->å->left
- s->ß->down
- d->∂->right

Currently non-letters are not supported, but they may be in the future.



