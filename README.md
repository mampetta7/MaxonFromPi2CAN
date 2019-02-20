# MaxonFromPi2CAN
Controls maxon motor controller from Pi using Pi2CAN adaptor board from SKPang. The board runs on socket CAN. 

The Pi config file should be updated to turn on SPI and set CAN parameters. Follow instructions here - https://www.skptechnology.co.uk/pican2-software-installation/

Then bring up socket CAN by typing the following command in the terminal

// $ sudo /sbin/ip link set can0 up type can bitrate 1000000

// the bit rate shuold match the bit rate set on the maxon controller

// to bring down CAN: $ sudo /sbin/ip link set can0 down


REFERENCES:

Pi2CAN (PiCAN2) board: http://skpang.co.uk/catalog/pican2-canbus-board-for-raspberry-pi-23-p-1475.html
Installation and bringup: https://www.skptechnology.co.uk/pican2-software-installation/
