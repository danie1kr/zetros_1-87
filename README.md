# remote controlled h0 zetros
## goal
This repository holds the files for a remote controlled Mercedes Benz Zetros model car (1:87 scale) and a custom PCB for a teensy based remote control. 
The model car itself is based on the herpa model car (#091213) outfitted with a fast build chassis from [here](http://www.mikromodellbau.de), a servo and some LEDs.
![enter image description here](https://github.com/danie1kr/zetros_1-87/raw/master/image.jpg)
## repository structure
- **arduino**: the code files for the teensy
- **schematics**: the schematics and pcb PDF files
- **remote laser cut**: the design files necessary for laser cutting the body.
## parts list
Order the PCB and solder the following parts according to the documentation:
- 1 Teensy 3.2 (see [here](https://www.pjrc.com))
- 1 Deltang Tx2V (see [here](http://www.deltang.co.uk))
- 3 Digital Potentiometers (MCP42010-IP)
- 1 5V source (TSR_1-2450)
- 2 Joysticks
- 5 LEDs
- 4 Buttons
- 1 9V battery holder
- 1 On-Off switch
- 1 mini or micro USB breakout + cable to Teensy
- Various pins/sockets, cables, resistors (at least 6x 10k for the voltage divider)
## licence
[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)
