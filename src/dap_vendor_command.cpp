/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
//#include <RTL.h>
#include <stdint.h>
#include <string.h>

//#include "rl_usb.h"
// #include "version.h"
#include "DAP_config.h"
#include "uart.h"
#include "DAP.h"

// Process DAP Vendor command and prepare response
// Default function (can be overridden)
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response
// this function is declared as __weak in DAP.c
uint32_t DAP_ProcessVendorCommand(const uint8_t *request, uint8_t *response) {

   // get unique ID command
   if (*request == 0) {
#define FOO "900900000fbc497b0000000000000000"
       *response++ = ID_DAP_Vendor0;
       *response++ = strlen(FOO);
       strcpy((char *)response, FOO);
       return (strlen(FOO) + 2);
   }

   // else return invalid command
   else {
       *response = ID_DAP_Invalid;
   }
   return (1);
}



