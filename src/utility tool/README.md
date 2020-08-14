
There are some utility tools for debug: [Baidu Pan](https://pan.baidu.com/s/1EHCHOiDqKoP81X50nBFBZw)  Secret Key: o2qk


### Keil uvision5:
The Integrated Development Environmen for Windows. Install it in default way.<br>
When you are registering it, please try more times.<br>


### USBTrace:
USBTrace is an easy to use and powerful USB analyzer (USB traffic sniffer) software for Windows.<br>
It is a fee-paying software.<br>
New users can use it for 15 days for free.<br>


### DAPLink_lpc11u35_archble:
It is the source code of DAPLink for lpc11u35_archble.<br>

DAPLink.zip is the original program includes MSC, CDC, WEBUSB, HID.<br>
DAPLink_for_Debug.zip is the program for debug and it only include HID and UART.<br>

You can find the project from \DAPLink\projectfiles\uvision\lpc11u35_archble_if<br>
Build it using Keil.<br>
Then you will find two .bin file in \DAPLink\projectfiles\uvision\lpc11u35_archble_if\build:<br>
 * lpc11u35_archble_if.bin<br>
     This file has something wrong. Don't use it.<br>
 * lpc11u35_archble_if_crc.bin<br>
     This file works fine. Drag it to lpc11u35_archble.<br>