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
 *    its USB product name. Unfortunately we can't override this from within the sketch, you'll have to edit
 *    files in your Arduino installation to change it.
 *    edit ~/Arduino15/packages/Seeeduino/hardware/samd/1.7.7/boards.txt
 *    add CMSIS-DAP to seeed_wio_terminal.build.usb_product:
 *    
 *    seeed_wio_terminal.build.usb_product="Seeed CMSIS-DAP"
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

static uint32_t free_count;
static uint32_t send_count;

static uint32_t recv_idx;
static uint32_t send_idx;
static volatile uint8_t  USB_ResponseIdle;

// define usb_hid
Adafruit_USBD_HID usb_hid;

// uint8_t rawhidRequest[DAP_PACKET_SIZE];
static uint8_t USB_Request [DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // Request  Buffer
uint8_t rawhidResponse[DAP_PACKET_SIZE];

#define FREE_COUNT_INIT          (DAP_PACKET_COUNT)
#define SEND_COUNT_INIT          0

uint8_t const desc_hid_report[] =
{
    // TUD_HID_REPORT_DESC_GENERIC_INOUT(64)
    /* HID */
//     0x06, 0xC0, 0xFF,      /* 30 */
//     0x0A, 0x00, 0x0C,
    
//     0xA1, 0x01,                  /* Collection 0x01 */
//     // RawHID is not multireport compatible.
//     // On Linux it might work with some modifications,
//     // however you are not happy to use it like that.
//     // 0x85, 0,			 /* REPORT_ID */
//     0x75, 0x08,                  /* report size = 8 bits */
//     0x15, 0x00,                  /* logical minimum = 0 */
//     0x26, 0xFF, 0x00,            /* logical maximum = 255 */
    
//     0x95, 64,        /* report count TX */
//     0x09, 0x01,                  /* usage */
//     0x81, 0x02,                  /* Input (array) */
    
//     0x95, 64,        /* report count RX */
//     0x09, 0x02,                  /* usage */
//     0x91, 0x02,                  /* Output (array) */
//     0xC0                         /* end collection */ 


    /* HID */
//     0x06, lowByte(RAWHID_USAGE_PAGE), highByte(RAWHID_USAGE_PAGE),      /* 30 */
//     0x0A, lowByte(RAWHID_USAGE), highByte(RAWHID_USAGE),
    
//     0xA1, 0x01,                  /* Collection 0x01 */
//     // RawHID is not multireport compatible.
//     // On Linux it might work with some modifications,
//     // however you are not happy to use it like that.
//     //0x85, HID_REPORTID_RAWHID,			 /* REPORT_ID */
//     0x75, 0x08,                  /* report size = 8 bits */
//     0x15, 0x00,                  /* logical minimum = 0 */
//     0x26, 0xFF, 0x00,            /* logical maximum = 255 */
    
//     0x95, RAWHID_TX_SIZE,        /* report count TX */
//     0x09, 0x01,                  /* usage */
//     0x81, 0x02,                  /* Input (array) */
    
//     0x95, RAWHID_RX_SIZE,        /* report count RX */
//     0x09, 0x02,                  /* usage */
//     0x91, 0x02,                  /* Output (array) */
//     0xC0                         /* end collection */ 


    /* USER CODE BEGIN 0 */ /* A minimal Report Desc with INPUT/OUTPUT/FEATURE report. Zach Lee */
//     0x06,0x00,0xFF,         /*  Usage Page (vendor defined) ($FF00) global */
//     0x09,0x01,              /*  Usage (vendor defined) ($01) local */
//     0xA1,0x01,              /*  Collection (Application) */
//     0x15,0x00,              /*   LOGICAL_MINIMUM (0) */
//     0x25,0xFF,              /*   LOGICAL_MAXIMUM (255) */
//     0x75,0x08,              /*   REPORT_SIZE (8bit) */
    
//     // Input Report
//     0x95,64,                /*   Report Length (64 REPORT_SIZE) */
//     0x09,0x01,              /*   USAGE (Vendor Usage 1) */
//     0x81,0x02,              /*   Input(data,var,absolute) */
    
//     // Output Report
//     0x95,64,                /*   Report Length (64 REPORT_SIZE) */
//     0x09,0x01,              /*   USAGE (Vendor Usage 1) */
//     0x91,0x02,              /*   Output(data,var,absolute) */
    
//     // Feature Report
//     0x95,64,                /*   Report Length (64 REPORT_SIZE) */
//     0x09,0x01,              /*   USAGE (Vendor Usage 1) */
//     0xB1,0x02,              /*   Feature(data,var,absolute) */
//     /* USER CODE END 0 */
//     0xC0                    /*  END_COLLECTION	             */


//     0x06,0xA0,0xFF,    //Usage Page(FFA0h, vendor defined)
//     0x09, 0x01,        //Usage(vendor defined)
//     0xA1, 0x01,        //Collection(Application)
//     0x09, 0x02 ,       //Usage(vendor defined)
//     0xA1, 0x00,        //Collection(Physical)
//     0x06,0xA1,0xFF,    //Usage Page(vendor defined)
    
//    //Input Report
//     0x09, 0x03 ,       //Usage(vendor defined)
//     0x09, 0x04,        //Usage(vendor defined)
//     0x15, 0x80,        //LOGICAL_MINIMUM(0x80 or -128)
//     0x25, 0x7F,        //LOGICAL_MAXIMUM(0x7F or 127)
//     0x35, 0x00,        //Physical minimum(0)
//     0x45, 0xFF,        //Physical maximum(255)
//     0x75, 0x08,        //Report size (8bit)
//     0x95, 0x40,        //Report Length(64 fields)
//     0x81, 0x02,        //Input(data, variable, absolute)
    
//    //Output Report
//     0x09, 0x05,        //Usage(vendor defined)
//     0x09, 0x06,        //Usage(vendor defined)
//     0x15, 0x80,        //LOGICAL_MINIMUM(0x80 or -128)
//     0x25, 0x7F,        //LOGICAL_MAXIMUM(0x7F or 127)
//     0x35, 0x00,        //Physical minimum(0)
//     0x45, 0xFF,        //Physical maximum(255)
//     0x75, 0x08,        //Report size(8bit)
//     0x95, 0x40,        //Report Length(64 fields)
//     0x91, 0x02,        //Output(data, variable, absolute)
//     0xC0,              //Collection(Physical)
//     0xC0               //Collection(Application)


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
    usb_hid.enableOutEndpoint(true);
    usb_hid.setPollInterval(2);
    usb_hid.setBootProtocol(0);
    usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    usb_hid.setReportCallback(get_report_callback, set_report_callback);
    
    usb_hid.begin();
    
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.begin(115200);
    // Serial1.begin(115200);
    
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

    // usb_hid.sendReport(0, "abc", sizeof("abc"));
    // Serial.println("Seeed Arduino DAPLink.");
    // delay(2);

  /* Teensy Demo Logic */
//   // Check if there is new data from the RawHID device
//   auto bytesAvailable =
// #ifdef HIDPROJECT_RAWHID
//     RawHID.available();
// #else
//     RawHID.recv(rawhidRequest, 0);
// #endif
//   if (bytesAvailable > 0) {
// #if DAP_SERIAL_LOG
//     Serial.print("cmd ");
//     Serial.print(rawhidRequest[0], HEX);
//     Serial.print(" ");
//     Serial.print(rawhidRequest[1], HEX);
//     Serial.print(" ");
// #endif /* DAP_SERIAL_LOG */
//     auto sz = DAP_ProcessCommand(rawhidRequest, rawhidResponse);
// #if DAP_SERIAL_LOG
//     Serial.print("rsp ");
//     Serial.print(sz);
//     Serial.println(" B");
// #endif /* DAP_SERIAL_LOG */
// #ifdef HIDPROJECT_RAWHID
//     RawHID.enable(); // signal that we're ready to receive another buffer
// #endif
//     if (sz > 0) {
// #ifdef HIDPROJECT_RAWHID
//       RawHID.write(rawhidResponse, DAP_PACKET_SIZE);
// #else
//       RawHID.send(rawhidResponse, DAP_PACKET_SIZE);
// #endif
//     }
//   }
}

// USB HID override function return 1 if the activity is trivial or response is null 
// __attribute__((weak))
// uint8_t usbd_hid_no_activity(U8 *buf)
// {
//     return 0;
// }

void hid_send_packet()
{
    if (send_count) {
        send_count--;
        usb_hid.sendReport(0, USB_Request[send_idx], DAP_PACKET_SIZE);
        send_idx = (send_idx + 1) % DAP_PACKET_COUNT;
        free_count++;
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t get_report_callback (uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    // int i;
    // digitalWrite(LED_BUILTIN, 1);
  
    Serial.println("Seeed Arduino DAPLink 1.");
    // auto sz = DAP_ProcessCommand(buffer, rawhidResponse);

    // for(i=0; i<64; i++){
	// 	Serial.println(rawhidResponse[i]);
	// }

    // if(sz > 0){
    //   usb_hid.sendReport(0, rawhidResponse, sizeof(rawhidResponse));
    // }
    
    // digitalWrite(LED_BUILTIN, 1);
    // return reqlen;

    switch (report_type) {
        case HID_REPORT_INPUT:
            switch (reqlen) {
                case USBD_HID_REQ_PERIOD_UPDATE:
                    break;

                case USBD_HID_REQ_EP_CTRL:
                case USBD_HID_REQ_EP_INT:
                    if (send_count > 0) {
                        send_count--;
                        memcpy(buffer, USB_Request[send_idx], DAP_PACKET_SIZE);
                        send_idx = (send_idx + 1) % DAP_PACKET_COUNT;
                        free_count++;
                        return (DAP_PACKET_SIZE);
                    } else if (reqlen == USBD_HID_REQ_EP_INT) {
                        USB_ResponseIdle = 1;
                    }
                    break;
            }

            break;

        case HID_REPORT_FEATURE:
            break;
    }

    return (0);
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
//   char* data = "123";
//   char data[64];
//   memset(data,0,64);
//   data[1] = 3;
    int i;
    // This example doesn't use multiple report and report ID
    (void) report_id;
    (void) report_type;
    
    digitalWrite(LED_BUILTIN, 1);
    
    Serial.println("Seeed Arduino DAPLink 2.");
    
    Serial.print("Size of buffer = ");
    Serial.print(sizeof(buffer));
    Serial.print("\n");
    
    Serial.print("(unsigned int)buffer = ");
    Serial.print((unsigned int)(buffer));
    Serial.print("\n");
    
    Serial.print("(*)buffer = ");
    Serial.print(*buffer);
    Serial.print("\n");
    
    Serial.print("report_id = ");
    Serial.print(report_id);
    Serial.print("\n");
    
    Serial.print("report_type = ");
    Serial.print(report_type);
    Serial.print("\n");

//    Serial.print("data = ");
//    Serial.print(data);
//    Serial.print("\n");

  // echo back anything we received from host
//    usb_hid.sendReport(0, buffer, bufsize);

//    usb_hid.sendReport(0, buffer, sizeof(buffer));

    // main_led_state_t led_next_state = MAIN_LED_FLASH;
    switch (report_type) {
        // case HID_REPORT_OUTPUT:
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
                // if(usbd_hid_no_activity(USB_Request[recv_idx]) == 1){
                //     //revert HID LED to default if the response is null
                //     // led_next_state = MAIN_LED_DEF;
                // }

                recv_idx = (recv_idx + 1) % DAP_PACKET_COUNT;
                send_count++;
                if (USB_ResponseIdle) {
                    hid_send_packet();
                    
                    for(i=0; i<64; i++){
                        Serial.print(USB_Request[recv_idx][i]);
                        Serial.print(" ");
                        Serial.print("\n");
                    }

                    Serial.print(USB_ResponseIdle);
                    USB_ResponseIdle = 0;
                }
            } 
            // else {
            //     util_assert(0);
            // }
            
            // main_blink_hid_led(led_next_state);

            break;

        case HID_REPORT_FEATURE:
            break;
    }
}
