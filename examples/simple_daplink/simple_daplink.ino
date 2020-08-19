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
#include <stdint.h>
static uint32_t free_count;
static uint32_t send_count;

static uint32_t recv_idx;
static uint32_t send_idx;
static volatile uint8_t  USB_ResponseIdle;

// define usb_hid
Adafruit_USBD_HID usb_hid;

static uint8_t USB_Request [DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // Request  Buffer
uint8_t rawhidResponse[DAP_PACKET_SIZE];

#define FREE_COUNT_INIT          (DAP_PACKET_COUNT)
#define SEND_COUNT_INIT          0


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

uint16_t Serial_u16[33];

static inline bool isInvalidUtf8Octet(uint8_t t) {
  // see bullets in https://tools.ietf.org/html/rfc3629#section-1
  return (t == 0xc0) || (t == 0xC1) || ((t >= 0xF5) && (t <= 0xFF));
}

// up to 32 unicode characters (header make it 33)
static uint16_t _desc_str[33];

//
// This function has an UNWRITTEN CONTRACT that the buffer is either:
// 1. Pre-validated as legal UTF-8, -OR-
// 2. has a trailing zero-value octet/byte/uint8_t  (aka null-terminated string)
//
// If the above are not true, this decoder may read past the end of the allocated
// buffer, by up to three bytes.
//
// U+1F47F == 👿 ("IMP")
//         == 0001_1111_0100_0111_1111 ==> requires four-byte encoding in UTF-8
//            AABB BBBB CCCC CCDD DDDD ==> 0xF0 0x9F 0x91 0xBF
//
// Example sandwich and safety variables are there to cover the
// two most-common stack layouts for declared variables, in unoptimized
// code, so that the bytes surrounding those allocated for 'evilUTF8'
// are guaranteed to be non-zero and valid UTF8 continuation octets.
//     uint8_t safety1      = 0;
//     uint8_t sandwich1[4] = { 0x81, 0x82, 0x83, 0x84 };
//     uint8_t evilUTF8[5]  = { 0xF0, 0x9F, 0x91, 0xBF, 0xF9 };
//     uint8_t sandwich2[4] = { 0x85, 0x86, 0x87, 0x88 };
//     uint8_t safety2      = 0;
//
// NOTE: evilUTF8 could just contain a single byte 0xF9 ....
//
// Attempting to decode evilUTF8 will progress to whatever is next to it on the stack.
// The above should work when optimizations are turned  
//
static int8_t utf8Codepoint(const uint8_t *utf8, uint32_t *codepointp)
{
  const uint32_t CODEPOINT_LOWEST_SURROGATE_HALF  =   0xD800;
  const uint32_t CODEPOINT_HIGHEST_SURROGATE_HALF =   0xDFFF;

  *codepointp = 0xFFFD; // always initialize output to known value ... 0xFFFD (REPLACEMENT CHARACTER) seems the natural choice
  int codepoint;
  int len;

  // The upper bits define both the length of additional bytes for the multi-byte encoding,
  // as well as defining how many bits of the first byte are included in the codepoint.
  // Each additional byte starts with 0b10xxxxxx, encoding six additional bits for the codepoint.
  //
  // For key summary points, see:
  // * https://tools.ietf.org/html/rfc3629#section-3
  //
  if (isInvalidUtf8Octet(utf8[0])) { // do not allow illegal octet sequences (e.g., 0xC0 0x80 should NOT decode to NULL)
    return -1;
  }

  if (utf8[0] < 0x80) {                   // characters 0000 0000..0000 007F (up to  7 significant bits)
    len = 1;
    codepoint = utf8[0];
  } else if ((utf8[0] & 0xe0) == 0xc0) {  // characters 0000 0080..0000 07FF (up to 11 significant bits, so first byte encodes five bits)
    len = 2;
    codepoint = utf8[0] & 0x1f;
  } else if ((utf8[0] & 0xf0) == 0xe0) {  // characters 0000 8000..0000 FFFF (up to 16 significant bits, so first byte encodes four bits)
    len = 3;
    codepoint = utf8[0] & 0x0f;
  } else if ((utf8[0] & 0xf8) == 0xf0) {  // characters 0001 0000..0010 FFFF (up to 21 significant bits, so first byte encodes three bits)
    len = 4;
    codepoint = utf8[0] & 0x07;
  } else {                                // UTF-8 is defined to only map to Unicode -- 0x00000000..0x0010FFFF
    // 5-byte and 6-byte sequences are not legal per RFC3629
    return -1;
  }

  for (int i = 1; i < len; i++) {
    if ((utf8[i] & 0xc0) != 0x80) {
      // the additional bytes in a valid UTF-8 multi-byte encoding cannot have either of the top two bits set
      // This is more restrictive than isInvalidUtf8Octet()
      return -1;
    }
    codepoint <<= 6;             // each continuation byte adds six bits to the codepoint
    codepoint |= utf8[i] & 0x3f; // mask off the top two continuation bits, and add the six relevant bits
  }

  // explicit validation to prevent overlong encodings
  if (       (len == 1) && ((codepoint < 0x000000) || (codepoint > 0x00007F))) {
    return -1;
  } else if ((len == 2) && ((codepoint < 0x000080) || (codepoint > 0x0007FF))) {
    return -1;
  } else if ((len == 3) && ((codepoint < 0x000800) || (codepoint > 0x00FFFF))) {
    return -1;
  } else if ((len == 4) && ((codepoint < 0x010000) || (codepoint > 0x10FFFF))) {
    // "You might expect larger code points than U+10FFFF
    // to be expressible, but Unicode is limited in Sections 12
    // of RFC3629 to match the limits of UTF-16." -- Wikipedia UTF-8 note
    // See https://tools.ietf.org/html/rfc3629#section-12
    return -1;
  }

  // high and low surrogate halves (U+D800 through U+DFFF) used by UTF-16 are
  // not legal Unicode values ... see RFC 3629.
  if ((codepoint >= CODEPOINT_LOWEST_SURROGATE_HALF) && (codepoint <= CODEPOINT_HIGHEST_SURROGATE_HALF)) {
    return -1;
  }

  *codepointp = codepoint;
  return len;
}


static int strcpy_utf16(const char *s, uint16_t *buf, int bufsize)
{
  int i = 0;
  int buflen = 0;

  while (s[i] != 0) {
    uint32_t codepoint;
    int8_t utf8len = utf8Codepoint((const uint8_t *)s + i, &codepoint);

    if (utf8len < 0) {
      // Invalid utf8 sequence, skip it
      i++;
      continue;
    }

    i += utf8len;

    if (codepoint <= 0xffff) {
      if (buflen == bufsize)
        break;

      buf[buflen++] = codepoint;

    } else {
      if (buflen + 1 >= bufsize)
        break;

      // Surrogate pair
      codepoint -= 0x10000;
      buf[buflen++] = (codepoint >> 10) + 0xd800;
      buf[buflen++] = (codepoint & 0x3ff) + 0xdc00;
    }
  }

  return buflen;
}


// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
// Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
// https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors
const uint16_t * tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  uint8_t chr_count;

  switch (index)
  {
    case 0:
      _desc_str[1] = USBDevice.getLanguageDescriptor();
      chr_count = 1;
    break;
    case 1:
      chr_count = strcpy_utf16(USBDevice.getManufacturerDescriptor(), _desc_str + 1, 32);
    break;

    case 2:
      chr_count = strcpy_utf16(USBDevice.getProductDescriptor(), _desc_str + 1, 32);
    break;

    case 3:
      // serial Number
      chr_count = USBDevice.getSerialDescriptor(_desc_str+1);
    break;
    case 4:
      // interface name
      chr_count = strcpy_utf16("CMSIS-DAP", _desc_str + 1, 32);
    break;
    default: return NULL;
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

  return _desc_str;
}




void setup() {
    usb_hid.enableOutEndpoint(true);
    usb_hid.setPollInterval(2);
    usb_hid.setBootProtocol(0);
    USBDevice.setProductDescriptor("CMSIS-DAP");
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

    USBDevice.getSerialDescriptor(Serial_u16);
}


void loop() {}
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
