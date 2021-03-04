/*
 * custom_pins.ino
 * Custom pin definition example.
 * Author: Kenta IDA
 * This sketch is based on simple_daplink.ino sample sketch in the Seeed_Arduino_DAPLink library.
 * The original license is below.
 */
/*
 * Seeed_Arduino_DAPLink.ino
 *
 * Copyright (c) 2020 seeed technology co., ltd.  
 * Author      : weihong.cai (weihong.cai@seeed.cc)  
 * Create Time : Aug 2020
 * Change Log  : 
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software istm
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS INcommInterface
 * THE SOFTWARE.
 *
 * Get start:
 * 1. For a device to be correctly detected as a CMSIS-DAP adapter, it must contain the string "CMSIS-DAP" in 
 *    its USB product name. 
 * 
 * 2. Download the Adafruit TinyUSB Library from: https://github.com/adafruit/Adafruit_TinyUSB_Arduino.
 *    And add it to your local arduino library.
 * 3. Arduino IDE tool-->Select the board: Seeeduino Wio Terminal. 
 * 4. Arduino IDE tool-->Select the USB Stack: TinyUSB. 
 * 5. Arduino IDE tool-->Select the COM port. 
 *
 * WARNING:
 *   This demo is just a test-demo. At present, it only enumerate a HID Device to communicate with host(PC).
 *   But it can't work as a DAPLink adapter. I think that it need to deal with the problem of data interaction 
 *   between host and device. If there is any progress, please tell to me.
 *
 */

#include "Adafruit_TinyUSB.h"
#include "DAP_config.h"
#include "DAP.h"
#include <stdint.h>
#include <cassert>

uint32_t baud;
uint32_t old_baud;

#define SerialTTL    Serial1

#define EPOUT 0x00
#define EPIN  0x80
#define EPSIZE 64
class CMSISDAPV2 : public Adafruit_USBD_Interface 
{
public:
    CMSISDAPV2() {
        this->setStringDescriptor("CMSIS-DAP interface");
    }
    virtual uint16_t getDescriptor(uint8_t itfnum, uint8_t* buf, uint16_t bufsize) override {
        assert(buf != nullptr);
        if( bufsize == 0 ) return 0;

        uint8_t const intf_desc[] = {
            TUD_VENDOR_DESCRIPTOR(itfnum, 0, EPOUT, EPIN, EPSIZE)
        };
        const uint16_t intf_desc_len = sizeof(intf_desc);

        if( bufsize < intf_desc_len ) return 0;

        memcpy(buf, intf_desc, intf_desc_len);
        return intf_desc_len;
    }
};

static CMSISDAPV2 cmsisdap_v2;

extern "C" void tud_vendor_rx_cb(uint8_t itf)
{
    auto data_available = tud_vendor_available();
    if( data_available > 0 ) {
        uint8_t buffer[EPSIZE];
        uint8_t response[EPSIZE];
        auto bytes_received = tud_vendor_read(buffer, sizeof(buffer));
        auto bytes_to_write = DAP_ExecuteCommand(buffer, response);
        if( bytes_to_write > 0 ) {
            tud_vendor_write(response, bytes_to_write);
        }
    }
}

void setup() {
    USBDevice.setProductDescriptor("CMSIS-DAP");
    USBDevice.addInterface(cmsisdap_v2);
    //USBDevice.setID(0x0D28,0x0204);
    // Custom pin definitions.
    // You must change pin definitions BEFORE calling DAP_Setup function.
    SWDIOPin.setPin(A9);
    SWCLKPin.setPin(A8);
    TDIPin.setPin(A5);
    TDOPin.setPin(A4);
    nRESETPin.setPin(A0);

    pinMode(LED_BUILTIN, OUTPUT);
    
    baud = old_baud = 115200;
    Serial.begin(baud);
    SerialTTL.begin(baud);

    // wait until device mounted
    while( !USBDevice.mounted() ) delay(1);
    
    DAP_Setup();
}


void loop() {
  // put your main code here, to run repeatedly:
  baud = Serial.baud();
  if (baud != old_baud) {
    SerialTTL.begin(baud);
    while (!SerialTTL);
    old_baud = baud;
  }

  if (Serial.available() > 0)
  {
    char c = Serial.read();
    SerialTTL.write(c);
  }

  if (SerialTTL.available() > 0) {
    char c = SerialTTL.read();
    Serial.write(c);
  }
}
