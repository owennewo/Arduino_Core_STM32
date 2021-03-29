/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins (STM32F405RG and STM32F415RG)
 *----------------------------------------------------------------------------*/

//                  | DIGITAL | ANALOG IN  | ANALOG OUT | UART/USART            | TWI                  | SPI                               | SPECIAL   |
//                  |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|
#define PA0  A0  // | 0       | A0 (ADC1)  |            | UART4_TX              |                      |                                   |           |
#define PA1  A1  // | 1       | A1 (ADC1)  |            | UART4_RX              |                      |                                   |           |
#define PA2  A2  // | 2       | A2 (ADC1)  |            | USART2_TX             |                      |                                   |           |
#define PA3  A3  // | 3       | A3 (ADC1)  |            | USART2_RX             |                      |                                   |           |
#define PA4  A4  // | 4       | A4 (ADC1)  | DAC_OUT1   |                       |                      | SPI1_SS, (SPI3_SS)                |           |
#define PA5  A5  // | 5       | A5 (ADC1)  | DAC_OUT2   |                       |                      | SPI1_SCK                          |           |
#define PA6  A6  // | 6       | A6 (ADC1)  |            |                       |                      | SPI1_MISO                         |           |
#define PA7  A7  // | 7       | A7 (ADC1)  |            |                       |                      | SPI1_MOSI                         |           |
#define PA8  8   // | 8       |            |            |                       | TWI3_SCL             |                                   |           |
#define PA9  9   // | 9       |            |            | USART1_TX             |                      | SPI2_SCK                          |           |
#define PA10 10  // | 10      |            |            | USART1_RX             |                      |                                   |           |
#define PA11 11  // | 11      |            |            |                       |                      |                                   |           |
#define PA12 12  // | 12      |            |            |                       |                      |                                   |           |
#define PA13 13  // | 13      |            |            |                       |                      |                                   | SWD_SWDIO |
#define PA14 14  // | 14      |            |            |                       |                      |                                   | SWD_SWCLK |
#define PA15 15  // | 15      |            |            |                       |                      | SPI3_SS, (SPI1_SS)                |           |
//                  |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|
#define PB0  A8  // | 16      | A8 (ADC1)  |            |                       |                      |                                   |           |
#define PB1  A9  // | 17      | A9 (ADC1)  |            |                       |                      |                                   |           |
#define PB2  18  // | 18      |            |            |                       |                      |                                   | BOOT1     |
#define PB3  19  // | 19      |            |            |                       |                      | SPI3_SCK,  (SPI1_SCK)             |           |
#define PB4  20  // | 20      |            |            |                       |                      | SPI3_MISO, (SPI1_MISO)            |           |
#define PB5  21  // | 21      |            |            |                       |                      | SPI3_MOSI, (SPI1_MOSI)            |           |
#define PB6  22  // | 22      |            |            | USART1_TX             | TWI1_SCL             |                                   |           |
#define PB7  23  // | 23      |            |            | USART1_RX             | TWI1_SDA             |                                   |           |
#define PB8  24  // | 24      |            |            |                       | TWI1_SCL             |                                   |           |
#define PB9  25  // | 25      |            |            |                       | TWI1_SDA             | SPI2_SS                           |           |
#define PB10 26  // | 26      |            |            | USART3_TX             | TWI2_SCL             | SPI2_SCK                          |           |
#define PB11 27  // | 27      |            |            | USART3_RX             | TWI2_SDA             |                                   |           |
#define PB12 28  // | 28      |            |            |                       |                      | SPI2_SS                           |           |
#define PB13 29  // | 29      |            |            |                       |                      | SPI2_SCK                          |           |
#define PB14 30  // | 30      |            |            |                       |                      | SPI2_MISO                         |           |
#define PB15 31  // | 31      |            |            |                       |                      | SPI2_MOSI                         |           |
//                  |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|
#define PC0  A10 // | 32      | A10 (ADC1) |            |                       |                      |                                   |           |
#define PC1  A11 // | 33      | A11 (ADC1) |            |                       |                      |                                   |           |
#define PC2  A12 // | 34      | A12 (ADC1) |            |                       |                      | SPI2_MISO                         |           |
#define PC3  A13 // | 35      | A13 (ADC1) |            |                       |                      | SPI2_MOSI                         |           |
#define PC4  A14 // | 36      | A14 (ADC1) |            |                       |                      |                                   |           |
#define PC5  A15 // | 37      | A15 (ADC1) |            |                       |                      |                                   |           |
#define PC6  38  // | 38      |            |            | USART6_TX             |                      |                                   |           |
#define PC7  39  // | 39      |            |            | USART3_RX             |                      | SPI2_SCK                          |           |
#define PC8  40  // | 40      |            |            |                       |                      |                                   |           |
#define PC9  41  // | 41      |            |            |                       | TWI3_SDA             |                                   |           |
#define PC10 42  // | 42      |            |            | USART3_TX, (UART4_TX) |                      | SPI3_SCK                          |           |
#define PC11 43  // | 43      |            |            | USART3_RX, (UART4_RX) |                      | SPI3_MISO                         |           |
#define PC12 44  // | 44      |            |            | UART5_TX              |                      | SPI3_MOSI                         |           |
#define PC13 45  // | 45      |            |            |                       |                      |                                   |           |
#define PC14 46  // | 46      |            |            |                       |                      |                                   | OSC32_IN  |
#define PC15 47  // | 47      |            |            |                       |                      |                                   | OSC32_OUT |
//                  |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|
#define PD2  48  // | 48      |            |            | UART5_RX              |                      |                                   |           |
//                  |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|
#define PH0  49  // | 49      |            |            |                       |                      |                                   | OSC_IN    |
#define PH1  50  // | 50      |            |            |                       |                      |                                   | OSC_OUT   |
//                  |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|

/// This must be a literal
#define NUM_DIGITAL_PINS        51
#define NUM_ANALOG_INPUTS       16

// PORT #1:  CAN  
// =============
#define CAN_RX        PB8
#define CAN_TX        PB9

// PORT #2: HALL, ENCODER, I2C1 or USART1
// ======================================
#define HALL_1        PB6
#define HALL_2        PB7
#define HALL_3        PC11
// OR
#define ENC_A         PB6
#define ENC_B         PB7
#define ENC_I         PC11
// OR (NOT WORKING?)
#define I2C1_SCL      PB6
#define I2C1_SDA      PB7
// OR (NOT WORKING?)
#define USART1_TX    PB6
#define USART1_RX    PB7
// AND
#define TEMP_MOTOR    PC0

// PORT #3: USART6, I2C2 or SPI1
// =============================
#define USART6_TX     PC6
#define USART6_RX     PC7 
// OR
#define I2C2_SCL      PB10
#define I2C2_SDA      PB11
// OR
#define SPI1_NSS      PA4
#define SPI1_SCLK     PA5 
#define SPI1_MISO     PA6
#define SPI1_MOSI     PA7

// PORT #4: SWD
// ============
#define SWDIO         PA13
#define SWCLK         PA14

// PORT #5: SERVO
// ==============
#define SERVO         PB5

// PORT #6: USB
// ============
#define USB_DM        PA11
#define USB_DP        PA12

// #7: INTERNAL PINS
// =================
#define SENS3         PA0
#define SENS2         PA1
#define SENS1         PA2
#define ADC_TEMP      PA3
#define H3            PA8
#define H2            PA9
#define H1            PA10
#define BR_SO2        PB0
#define BR_SO1        PB1
#define DC_CAL        PB12
#define L3            PB13
#define L2            PB14
#define L1            PB15
#define AN_IN         PC2
#define LED_GREEN     PC4
#define LED_RED       PC5
#define EN_GATE       PC10
#define FAULT         PC12


// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#define TIMER_TONE              TIM6
#define TIMER_SERVO             TIM7

// UART Definitions
// Define here Serial instance number to map on Serial generic name
#define SERIAL_UART_INSTANCE    6

// Default pin used for 'Serial' instance
// Mandatory for Firmata
#define PIN_SERIAL_RX           PC7
#define PIN_SERIAL_TX           PC6

/* Extra HAL modules */
#define HAL_DAC_MODULE_ENABLED
#define HAL_CAN_MODULE_ENABLED

#ifdef __cplusplus
} // extern "C"
#endif
/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
  // These serial port names are intended to allow libraries and architecture-neutral
  // sketches to automatically default to the correct port name for a particular type
  // of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
  // the first hardware serial port whose RX/TX pins are not dedicated to another use.
  //
  // SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
  //
  // SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
  //
  // SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
  //
  // SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
  //
  // SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
  //                            pins are NOT connected to anything by default.
  #define SERIAL_PORT_MONITOR     Serial
  #define SERIAL_PORT_HARDWARE    Serial
#endif

// default speed to 400K
#define I2C_TIMING_FM           0x0010020A

#endif /* _VARIANT_ARDUINO_STM32_ */
