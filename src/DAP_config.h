/* CMSIS-DAP ported to run on the Arduino Micro
 * Copyright (C) 2016 Phillip Pearson <pp@myelin.co.nz>
 *
 * CMSIS-DAP Interface Firmware
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

// #ifdef TEENSYDUINO
// #define TEENSY_RAWHID
// #include "usb_desc.h"
// #else
// #define HIDPROJECT_RAWHID
// #include "HID-Project.h"
// #endif


#ifndef __DAP_CONFIG_H__
#define __DAP_CONFIG_H__

#define __forceinline __attribute__((always_inline))
#define __weak
#define OS_TID int
#define __task
#define U64 uint64_t
#define os_dly_wait delayMicroseconds

#define DAP_VENDOR "Seeed"
#define DAP_PRODUCT "Seeed CMSIS-DAP"
#define DAP_SER_NUM "1234"

//**************************************************************************************************
/**
\defgroup DAP_Config_Debug_gr CMSIS-DAP Debug Unit Information
\ingroup DAP_ConfigIO_gr
@{
Provides definitions about:
 - Definition of Cortex-M processor parameters used in CMSIS-DAP Debug Unit.
 - Debug Unit communication packet size.
 - Debug Access Port communication mode (JTAG or SWD).
 - Optional information about a connected Target Device (for Evaluation Boards).
*/

#include <Arduino.h>                             // Debug Unit Cortex-M Processor Header File
#include "usb_desc.h"
/// Processor Clock of the Cortex-M MCU used in the Debug Unit.
/// This value is used to calculate the SWD/JTAG clock speed.
#define CPU_CLOCK               F_CPU        ///< Specifies the CPU Clock in Hz

/// Number of processor cycles for I/O Port write operations.
/// This value is used to calculate the SWD/JTAG clock speed that is generated with I/O
/// Port write operations in the Debug Unit by a Cortex-M MCU. Most Cortex-M processors
/// requrie 2 processor cycles for a I/O Port Write operation.  If the Debug Unit uses
/// a Cortex-M0+ processor with high-speed peripheral I/O only 1 processor cycle might be
/// requrired.
#define IO_PORT_WRITE_CYCLES    1               ///< I/O Cycles: 2=default, 1=Cortex-M0+ fast I/0

#if !defined(DAP_SERIAL_LOG)
#define DAP_SERIAL_LOG          1
#endif

/// Indicate that Serial Wire Debug (SWD) communication mode is available at the Debug Access Port.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#if !defined(DAP_SWD)
#define DAP_SWD                 1               ///< SWD Mode:  1 = available, 0 = not available
#endif

/// Indicate that JTAG communication mode is available at the Debug Port.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#if !defined(DAP_JTAG)
#define DAP_JTAG                1               ///< JTAG Mode: 1 = available, 0 = not available.
#endif

/// Configure maximum number of JTAG devices on the scan chain connected to the Debug Access Port.
/// This setting impacts the RAM requirements of the Debug Unit. Valid range is 1 .. 255.
#define DAP_JTAG_DEV_CNT        4               ///< Maximum number of JTAG devices on scan chain

/// Default communication mode on the Debug Access Port.
/// Used for the command \ref DAP_Connect when Port Default mode is selected.
#define DAP_DEFAULT_PORT        1               ///< Default JTAG/SWJ Port Mode: 1 = SWD, 2 = JTAG.

/// Default communication speed on the Debug Access Port for SWD and JTAG mode.
/// Used to initialize the default SWD/JTAG clock frequency.
/// The command \ref DAP_SWJ_Clock can be used to overwrite this default setting.
#define DAP_DEFAULT_SWJ_CLOCK   5000000         ///< Default SWD/JTAG clock frequency in Hz.

/// Maximum Package Size for Command and Response data.
/// This configuration settings is used to optimized the communication performance with the
/// debugger and depends on the USB peripheral. Change setting to 1024 for High-Speed USB.
#define DAP_PACKET_SIZE         RAWHID_RX_SIZE              ///< USB: 64 = Full-Speed, 1024 = High-Speed.

/// Maximum Package Buffers for Command and Response data.
/// This configuration settings is used to optimized the communication performance with the
/// debugger and depends on the USB peripheral. For devices with limited RAM or USB buffer the
/// setting can be reduced (valid range is 1 .. 255). Change setting to 4 for High-Speed USB.
#define DAP_PACKET_COUNT        1              ///< Buffers: 64 = Full-Speed, 4 = High-Speed.

/// Indicate that UART Serial Wire Output (SWO) trace is available.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#define SWO_UART                0               ///< SWO UART:  1 = available, 0 = not available

/// Maximum SWO UART Baudrate
#define SWO_UART_MAX_BAUDRATE   10000000U       ///< SWO UART Maximum Baudrate in Hz

/// Indicate that Manchester Serial Wire Output (SWO) trace is available.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#define SWO_MANCHESTER          0               ///< SWO Manchester:  1 = available, 0 = not available

/// SWO Trace Buffer Size.
#define SWO_BUFFER_SIZE         4096U           ///< SWO Trace Buffer Size in bytes (must be 2^n)

/// SWO Streaming Trace.
#define SWO_STREAM              0               ///< SWO Streaming Trace: 1 = available, 0 = not available.

/// Clock frequency of the Test Domain Timer. Timer value is returned with \ref TIMESTAMP_GET.
#define TIMESTAMP_CLOCK         48000000U      ///< Timestamp clock in Hz (0 = timestamps not supported).
/// Debug Unit is connected to fixed Target Device.
/// The Debug Unit may be part of an evaluation board and always connected to a fixed
/// known device.  In this case a Device Vendor and Device Name string is stored which
/// may be used by the debugger or IDE to configure device parameters.
#define TARGET_DEVICE_FIXED     0               ///< Target Device: 1 = known, 0 = unknown;

#if TARGET_DEVICE_FIXED
#define TARGET_DEVICE_VENDOR    ""              ///< String indicating the Silicon Vendor
#define TARGET_DEVICE_NAME      ""              ///< String indicating the Target Device
#endif

///@}


// Debug Port I/O Pins

// #ifdef TEENSYDUINO
// #define PIN_SWDIO 2
// #define PIN_SWCLK 3
// #define PIN_TDO 4
// #define PIN_TDI 5
// #define PIN_nRESET 6
// #define PIN_LED_CONNECTED LED_BUILTIN
// #define PIN_LED_RUNNING LED_BUILTIN
// #elif AT16U2_DUE
// #define PIN_SWDIO   2//PB2 // MOSI pin
// #define PIN_SWCLK   1//PB1 // SCK
// #define PIN_TDO     6//PB6
// #define PIN_TDI     7//PB7
// #define PIN_nRESET  8//PC7
// #define PIN_LED_CONNECTED LED_BUILTIN_TX
// #define PIN_LED_RUNNING LED_BUILTIN_RX
// #else
// #define PIN_SWDIO 2
// #define PIN_SWCLK 3
// #define PIN_TDO 4
// #define PIN_TDI 5
// #define PIN_nRESET 6
// #define PIN_LED_CONNECTED LED_BUILTIN_TX
// #define PIN_LED_RUNNING LED_BUILTIN_RX
// #endif
#ifdef WIO_TERMINAL
#define PIN_SWDIO         D0
#define PIN_SWCLK         D1
#define PIN_TDO           D2
#define PIN_TDI           D3
#define PIN_nRESET        D4
#define PIN_LED_CONNECTED PIN_LED_TXL
#define PIN_LED_RUNNING   PIN_LED_RXL
#define Fast              1

#elif ARDUINO_SEEED_XIAO_M0

#define PIN_SWDIO         A9
#define PIN_SWCLK         A10
#define PIN_TDO           A2
#define PIN_TDI           A3
#define PIN_nRESET        A8
#define PIN_LED_CONNECTED PIN_LED_TXL
#define PIN_LED_RUNNING   PIN_LED_RXL
#define Fast              1

#else

#define PIN_SWDIO         A0
#define PIN_SWCLK         A1
#define PIN_TDO           A3
#define PIN_TDI           A4
#define PIN_nRESET        A2
#define PIN_LED_CONNECTED LED_BUILTIN
#define PIN_LED_RUNNING   LED_BUILTIN
#define Fast              0

#endif 

#if Fast

struct PortPin
{
  int pin;
  int mode;
  PortGroup* group;
  uint32_t bit;
  PortPin(int pin, int mode) : pin(pin), mode(mode) {
    this->group = &PORT->Group[g_APinDescription[pin].ulPort];
    this->bit = (1ul << g_APinDescription[pin].ulPin);
    pinMode(mode);
  }
  void setPin(int new_pin) {
    auto output = this->readOutput();
    this->pin = new_pin;
    this->group = &PORT->Group[g_APinDescription[new_pin].ulPort];
    this->bit = (1ul << g_APinDescription[new_pin].ulPin);
    this->write(output);
    this->pinMode(this->mode);
  }
  void pinMode(int mode) {
    ::pinMode(this->pin, mode);
    this->mode = mode;
  }
  void enableOutput() {
    this->pinMode(OUTPUT);
  }
  void disableOutput() {
    this->pinMode(INPUT);
  }
  void enablePullUp() {
    this->pinMode(INPUT_PULLUP);
  }
  void set() {
    this->group->OUTSET.reg = this->bit;
  }
  void clear() {
    this->group->OUTCLR.reg = this->bit;
  }
  bool read() {
    return this->group->IN.reg & this->bit;
  }
  bool readOutput() {
    return this->group->OUT.reg & this->bit;
  }
  void write(bool value) {
    if( value ) {
      this->set();
    }
    else {
      this->clear();
    }
  }
};

#else

struct PortPin
{
  int pin;
  int mode;
  bool output;

  PortPin(int pin, int mode) : pin(pin), mode(mode), output(false) {
    pinMode(mode);
  }
  void setPin(int new_pin) {
    this->pin = new_pin;
    this->write(this->output);
    this->pinMode(this->mode);
  }
  void pinMode(int mode) {
    ::pinMode(this->pin, mode);
    this->mode = mode;
  }
  void enableOutput() {
    this->pinMode(OUTPUT);
  }
  void disableOutput() {
    this->pinMode(INPUT);
  }
  void enablePullUp() {
    this->pinMode(INPUT_PULLUP);
  }
  void set() {
    this->output = true;
    digitalWrite(this->pin, true);
  }
  void clear() {
    this->output = false;
    digitalWrite(this->pin, false);
  }
  bool read() {
    return digitalRead(this->pin) != 0;
  }
  bool readOutput() {
    return this->output;
  }
  void write(bool value) {
    this->output = value;
    digitalWrite(this->pin, value);
  }
};

#endif

struct DummyPin
{
  bool output;
  DummyPin(int pin, int mode) : output(false) {}
  void setPin(int new_pin) {}
  void pinMode(int mode) {}
  void enableOutput() {}
  void disableOutput() {}
  void enablePullUp() {}
  void set() { this->output = true; }
  void clear() { this->output = false; }
  bool read() { return this->output; }
  bool readOutput() { return this->output; }
  void write(bool value) { this->output = value; }
};

#ifdef PIN_LED_CONNECTED 
typedef PortPin LED_CONNECTEDPinType;
#else
typedef DummyPin LED_CONNECTEDPinType;
#endif
#ifdef PIN_LED_RUNNING 
typedef PortPin LED_RUNNINGPinType;
#else
typedef DummyPin LED_RUNNINGPinType;
#endif

extern PortPin SWDIOPin;
extern PortPin SWCLKPin;
extern PortPin TDIPin;
extern PortPin TDOPin;
extern PortPin nRESETPin;
extern LED_CONNECTEDPinType LED_CONNECTEDPin;
extern LED_RUNNINGPinType LED_RUNNINGPin;
//**
//**************************************************************************************************
//**************************************************************************************************
/**
\defgroup DAP_Config_PortIO_gr CMSIS-DAP Hardware I/O Pin Access
\ingroup DAP_ConfigIO_gr
@{

Standard I/O Pins of the CMSIS-DAP Hardware Debug Port support standard JTAG mode
and Serial Wire Debug (SWD) mode. In SWD mode only 2 pins are required to implement the debug
interface of a device. The following I/O Pins are provided:

JTAG I/O Pin                 | SWD I/O Pin          | CMSIS-DAP Hardware pin mode
---------------------------- | -------------------- | ---------------------------------------------
TCK: Test Clock              | SWCLK: Clock         | Output Push/Pull
TMS: Test Mode Select        | SWDIO: Data I/O      | Output Push/Pull; Input (for receiving data)
TDI: Test Data Input         |                      | Output Push/Pull
TDO: Test Data Output        |                      | Input
nTRST: Test Reset (optional) |                      | Output Open Drain with pull-up resistor
nRESET: Device Reset         | nRESET: Device Reset | Output Open Drain with pull-up resistor


DAP Hardware I/O Pin Access Functions
-------------------------------------
The various I/O Pins are accessed by functions that implement the Read, Write, Set, or Clear to
these I/O Pins.

For the SWDIO I/O Pin there are additional functions that are called in SWD I/O mode only.
This functions are provided to achieve faster I/O that is possible with some advanced GPIO
peripherals that can independently write/read a single I/O pin without affecting any other pins
of the same I/O port. The following SWDIO I/O Pin functions are provided:
 - \ref PIN_SWDIO_OUT_ENABLE to enable the output mode from the DAP hardware.
 - \ref PIN_SWDIO_OUT_DISABLE to enable the input mode to the DAP hardware.
 - \ref PIN_SWDIO_IN to read from the SWDIO I/O pin with utmost possible speed.
 - \ref PIN_SWDIO_OUT to write to the SWDIO I/O pin with utmost possible speed.
*/


// Configure DAP I/O pins ------------------------------

/** Setup JTAG I/O pins: TCK, TMS, TDI, TDO, nTRST, and nRESET.
Configures the DAP Hardware I/O pins for JTAG mode:
 - TCK, TMS, TDI, nTRST, nRESET to output mode and set to high level.
 - TDO to input mode.
*/
static __inline void PORT_JTAG_SETUP (void) {
  SWCLKPin.pinMode(OUTPUT);
  SWDIOPin.pinMode(OUTPUT);
  nRESETPin.pinMode(OUTPUT);
  TDIPin.pinMode(OUTPUT);
  TDOPin.pinMode(INPUT);
}

/** Setup SWD I/O pins: SWCLK, SWDIO, and nRESET.
Configures the DAP Hardware I/O pins for Serial Wire Debug (SWD) mode:
 - SWCLK, SWDIO, nRESET to output mode and set to default high level.
 - TDI, TMS, nTRST to HighZ mode (pins are unused in SWD mode).
*/
static __inline void PORT_SWD_SETUP (void) {
  SWCLKPin.pinMode(OUTPUT);
  SWDIOPin.pinMode(OUTPUT);
  nRESETPin.pinMode(OUTPUT);
  TDIPin.pinMode(INPUT);
  TDOPin.pinMode(INPUT);
}

/** Disable JTAG/SWD I/O Pins.
Disables the DAP Hardware I/O pins which configures:
 - TCK/SWCLK, TMS/SWDIO, TDI, TDO, nTRST, nRESET to High-Z mode.
*/
static __inline void PORT_OFF (void) {
  SWCLKPin.pinMode(INPUT);
  SWDIOPin.pinMode(INPUT_PULLUP);
  nRESETPin.pinMode(INPUT);
  TDIPin.pinMode(INPUT_PULLUP);
  TDOPin.pinMode(INPUT);
}


// SWCLK/TCK I/O pin -------------------------------------

/** SWCLK/TCK I/O pin: Get Input.
\return Current status of the SWCLK/TCK DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_SWCLK_TCK_IN  (void) {
  return (0);   // Not available
}

/** SWCLK/TCK I/O pin: Set Output to High.
Set the SWCLK/TCK DAP hardware I/O pin to high level.
*/
static __forceinline void     PIN_SWCLK_TCK_SET (void) {
  SWCLKPin.set();
}

/** SWCLK/TCK I/O pin: Set Output to Low.
Set the SWCLK/TCK DAP hardware I/O pin to low level.
*/
static __forceinline void     PIN_SWCLK_TCK_CLR (void) {
  SWCLKPin.clear();
}


// SWDIO/TMS Pin I/O --------------------------------------

/** SWDIO/TMS I/O pin: Get Input.
\return Current status of the SWDIO/TMS DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_SWDIO_TMS_IN  (void) {
  return SWDIOPin.read() ? 1 : 0;
}

/** SWDIO/TMS I/O pin: Set Output to High.
Set the SWDIO/TMS DAP hardware I/O pin to high level.
*/
static __forceinline void     PIN_SWDIO_TMS_SET (void) {
  SWDIOPin.set();
}

/** SWDIO/TMS I/O pin: Set Output to Low.
Set the SWDIO/TMS DAP hardware I/O pin to low level.
*/
static __forceinline void     PIN_SWDIO_TMS_CLR (void) {
  SWDIOPin.clear();
}

/** SWDIO I/O pin: Get Input (used in SWD mode only).
\return Current status of the SWDIO DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_SWDIO_IN      (void) {
  return SWDIOPin.read() ? 1 : 0;
}

/** SWDIO I/O pin: Set Output (used in SWD mode only).
\param bit Output value for the SWDIO DAP hardware I/O pin.
*/
static __forceinline void     PIN_SWDIO_OUT     (uint32_t bit) {
  SWDIOPin.write(bit & 1);
}

/** SWDIO I/O pin: Switch to Output mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to output mode. This function is
called prior \ref PIN_SWDIO_OUT function calls.
*/
static __forceinline void     PIN_SWDIO_OUT_ENABLE  (void) {
  SWDIOPin.enableOutput();
}

/** SWDIO I/O pin: Switch to Input mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to input mode. This function is
called prior \ref PIN_SWDIO_IN function calls.
*/
static __forceinline void     PIN_SWDIO_OUT_DISABLE (void) {
  SWDIOPin.pinMode(INPUT_PULLUP);
}


// TDI Pin I/O ---------------------------------------------

/** TDI I/O pin: Get Input.
\return Current status of the TDI DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_TDI_IN  (void) {
  return TDIPin.read() ? 1 : 0;
}

/** TDI I/O pin: Set Output.
\param bit Output value for the TDI DAP hardware I/O pin.
*/
static __forceinline void     PIN_TDI_OUT (uint32_t bit) {
  TDIPin.write(bit & 1);
}


// TDO Pin I/O ---------------------------------------------

/** TDO I/O pin: Get Input.
\return Current status of the TDO DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_TDO_IN  (void) {
  return TDOPin.read() ? 1 : 0;
}


// nTRST Pin I/O -------------------------------------------

/** nTRST I/O pin: Get Input.
\return Current status of the nTRST DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_nTRST_IN   (void) {
  return (0);   // Not available
}

/** nTRST I/O pin: Set Output.
\param bit JTAG TRST Test Reset pin status:
           - 0: issue a JTAG TRST Test Reset.
           - 1: release JTAG TRST Test Reset.
*/
static __forceinline void     PIN_nTRST_OUT  (uint32_t bit) {
  ;             // Not available
}

// nRESET Pin I/O------------------------------------------

/** nRESET I/O pin: Get Input.
\return Current status of the nRESET DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_nRESET_IN  (void) {
  return nRESETPin.read() ? 1 : 0;
}

/** nRESET I/O pin: Set Output.
\param bit target device hardware reset pin status:
           - 0: issue a device hardware reset.
           - 1: release device hardware reset.
*/
static __forceinline void     PIN_nRESET_OUT (uint32_t bit) {
  nRESETPin.write(bit & 1);
}

///@}


//**************************************************************************************************
/**
\defgroup DAP_Config_LEDs_gr CMSIS-DAP Hardware Status LEDs
\ingroup DAP_ConfigIO_gr
@{

CMSIS-DAP Hardware may provide LEDs that indicate the status of the CMSIS-DAP Debug Unit.

It is recommended to provide the following LEDs for status indication:
 - Connect LED: is active when the DAP hardware is connected to a debugger.
 - Running LED: is active when the debugger has put the target device into running state.
*/

/** Debug Unit: Set status of Connected LED.
\param bit status of the Connect LED.
           - 1: Connect LED ON: debugger is connected to CMSIS-DAP Debug Unit.
           - 0: Connect LED OFF: debugger is not connected to CMSIS-DAP Debug Unit.
*/
static __inline void LED_CONNECTED_OUT (uint32_t bit) {
  LED_CONNECTEDPin.write(bit & 1);
}

/** Debug Unit: Set status Target Running LED.
\param bit status of the Target Running LED.
           - 1: Target Running LED ON: program execution in target started.
           - 0: Target Running LED OFF: program execution in target stopped.
*/
static __inline void LED_RUNNING_OUT (uint32_t bit) {
  LED_RUNNINGPin.write(bit & 1);
}

///@}

//**************************************************************************************************
/**
\defgroup DAP_Config_Timestamp_gr CMSIS-DAP Timestamp
\ingroup DAP_ConfigIO_gr
@{
Access function for Test Domain Timer.

The value of the Test Domain Timer in the Debug Unit is returned by the function \ref TIMESTAMP_GET. By
default, the DWT timer is used.  The frequency of this timer is configured with \ref TIMESTAMP_CLOCK.

*/

/** Get timestamp of Test Domain Timer.
\return Current timestamp value.
*/
__STATIC_INLINE uint32_t TIMESTAMP_GET (void) {
  uint32_t ticks = SysTick->VAL;
  	// Configure SysTick to trigger every millisecond using the CPU Clock
	SysTick->CTRL = 0;					    // Disable SysTick
	SysTick->LOAD = 0xFFFFFF;				// Set reload register for MAX Value
	SysTick->VAL = 0;					      // Reset the SysTick counter value
	SysTick->CTRL = 0x00000005;			// Enable SysTick,No Interrupt, Use CPU Clock
  return(ticks);
  //return (DWT->CYCCNT) / (CPU_CLOCK / TIMESTAMP_CLOCK);
}

///@}

//**************************************************************************************************
/**
\defgroup DAP_Config_Initialization_gr CMSIS-DAP Initialization
\ingroup DAP_ConfigIO_gr
@{

CMSIS-DAP Hardware I/O and LED Pins are initialized with the function \ref DAP_SETUP.
*/

/** Setup of the Debug Unit I/O pins and LEDs (called when Debug Unit is initialized).
This function performs the initialization of the CMSIS-DAP Hardware I/O Pins and the
Status LEDs. In detail the operation of Hardware I/O and LED pins are enabled and set:
 - I/O clock system enabled.
 - all I/O pins: input buffer enabled, output pins are set to HighZ mode.
 - for nTRST, nRESET a weak pull-up (if available) is enabled.
 - LED output pins are enabled and LEDs are turned off.
*/
static __inline void DAP_SETUP (void) {
  PORT_OFF();
  LED_CONNECTEDPin.enableOutput();
  LED_CONNECTEDPin.clear();
  LED_RUNNINGPin.enableOutput();
  LED_RUNNINGPin.clear();
}

/** Reset Target Device with custom specific I/O pin or command sequence.
This function allows the optional implementation of a device specific reset sequence.
It is called when the command \ref DAP_ResetTarget and is for example required
when a device needs a time-critical unlock sequence that enables the debug port.
\return 0 = no device specific reset sequence is implemented.\n
        1 = a device specific reset sequence is implemented.
*/
static __inline uint32_t RESET_TARGET (void) {
  return (0);              // change to '1' when a device reset sequence is implemented
}

///@}


#endif /* __DAP_CONFIG_H__ */
