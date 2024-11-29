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


#include "DAP_config.h"
#include "DAP.h"

#include <stdint.h>
static uint32_t free_count;
static uint32_t send_count;

static uint32_t recv_idx;
static uint32_t send_idx;
static volatile uint8_t  USB_ResponseIdle;

#define USBDevice TinyUSBDevice

// define usb_hid
Adafruit_USBD_HID usb_hid;

static uint8_t USB_Request [DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // Request  Buffer
uint8_t rawhidResponse[DAP_PACKET_SIZE];

uint32_t baud;
uint32_t old_baud;

#define FREE_COUNT_INIT          (DAP_PACKET_COUNT)
#define SEND_COUNT_INIT          0

#define SerialTTL    Serial1

uint8_t const desc_hid_report[] =
{
    0x06, 0x00, 0xFF,     /*  Usage Page (vendor defined) ($FF00) global */
    0x09, 0x01,           /*  Usage (vendor defined) ($01) local */
    0xA1, 0x01,           /*  Collection (Application) */
    0x15, 0,              /*  LOGICAL_MINIMUM (0) */
    0x26, (0xFF&0xFF), ((0xFF>>8)&0xFF), /* logical maximum = 255 */
    0x75, 8,              /*  REPORT_SIZE (8bit) */
    // Input Report
    0x95, 64,             /*  Report Length (64 REPORT_SIZE) */
    0x09, 0x01,           /*  USAGE (Vendor Usage 1) */
    0x81, (0<<0 | 1<<1 | 0<<2),  /*  Input(data,var,absolute) */
    // Output Report
    0x95, 64,                    /*  Report Length (64 REPORT_SIZE) */
    0x09, 0x01,                  /*  USAGE (Vendor Usage 1) */
    0x91, (0<<0 | 1<<1 | 0<<2),  /*  Output(data,var,absolute) */
    // Feature Report
    0x95, 1,                     /*  Report Length (1 REPORT_SIZE) */
    0x09, 0x01,                  /*  USAGE (Vendor Usage 1) */
    0xB1, (0<<0 | 1<<1 | 0<<2),  /*  Feature(data,var,absolute) */
    0xC0                         /*  END_COLLECTION	 */

};

void setup() {
    USBDevice.setProductDescriptor("CMSIS-DAP");
    //USBDevice.setID(0x0D28,0x0204);
    
    usb_hid.enableOutEndpoint(true);
    usb_hid.setPollInterval(2);
    usb_hid.setBootProtocol(0);
    usb_hid.setStringDescriptor("CMSIS-DAP");
    usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    usb_hid.setReportCallback(get_report_callback, set_report_callback);
    
    usb_hid.begin();
    
    pinMode(LED_BUILTIN, OUTPUT);
    
    baud = old_baud = 115200;
    Serial.begin(baud);
    SerialTTL.begin(baud);

    // wait until device mounted
    while( !USBDevice.mounted() ) delay(1);
    
    DAP_Setup();

    recv_idx = 0;
    send_idx = 0;
    USB_ResponseIdle = 1;
    free_count = FREE_COUNT_INIT;
    send_count = SEND_COUNT_INIT;
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

void hid_send_packet()
{
    if (send_count) {
        send_count--;
        usb_hid.sendReport(0, USB_Request[0], DAP_PACKET_SIZE);
        send_idx = (send_idx + 1) % DAP_PACKET_COUNT;
        free_count++;
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t get_report_callback (uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    return (0);
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    int i;
    // main_led_state_t led_next_state = MAIN_LED_FLASH;
    switch (report_type) {
        case 0:
            if (bufsize == 0) {
                break;
            }

            if (buffer[0] == ID_DAP_TransferAbort) {
                DAP_TransferAbort = 1;
                break;
            }

            // Store data into request packet buffer
            // If there are no free buffers discard the data
            if (free_count > 0) {
                free_count--;
                memcpy(USB_Request[recv_idx], buffer, bufsize);
                DAP_ExecuteCommand(buffer, USB_Request[recv_idx]);
                recv_idx = (recv_idx + 1) % DAP_PACKET_COUNT;
                send_count++;
                if (USB_ResponseIdle) {
                    hid_send_packet();            
                }
            } 
            break;

        case HID_REPORT_FEATURE:
            break;
    }
}
