/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2020 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifndef MyHwLGT8F_h
#define MyHwLGT8F_h

//#include <avr/eeprom.h>
#include <EEPROM.h>
//#include <avr/pgmspace.h>
#include <avr/sleep.h>
//#include <avr/power.h>
//#include <avr/interrupt.h>
#include <WDT.h>
//#include <util/atomic.h>
#include <SPI.h>


#ifdef __cplusplus
#include <Arduino.h>
#endif

#define CRYPTO_LITTLE_ENDIAN

#ifndef MY_SERIALDEVICE
#define MY_SERIALDEVICE Serial
#endif

#ifndef MY_DEBUGDEVICE
#define MY_DEBUGDEVICE MY_SERIALDEVICE
#endif

 //(x86)\Arduino\hardware\tools\avr\avr\include\inttypes.h
#define		PRIu8			"u"
#define		PRIu16			"u"
#define		PRIi8			"i"
#define		PRIu32			"lu"

// Define these as macros to save valuable space
#define hwDigitalWrite(__pin, __value) digitalWrite(__pin, __value)
#define hwDigitalRead(__pin) digitalRead(__pin)
#define hwPinMode(__pin, __value) pinMode(__pin, __value)

//#define hwDigitalWrite(__pin, __value) digitalWriteFast(__pin, __value)
//#define hwDigitalRead(__pin) digitalReadFast(__pin)
//#define hwPinMode(__pin, __value) pinModeFast(__pin, __value)

bool hwInit(void);

#define hwWatchdogReset() wdt_reset()
#define hwReboot() wdt_enable(WDTO_15MS); while (1)
#define hwMillis() millis()

void hwReadConfigBlock(void *buf, void *addr, size_t length);
void hwWriteConfigBlock(void *buf, void *addr, size_t length);
void hwWriteConfig(const int addr, uint8_t value);
uint8_t hwReadConfig(const int addr);


inline void hwRandomNumberInit(void);
uint32_t hwInternalSleep(uint32_t ms);

#if defined(MY_SOFTSPI)
SoftSPI<MY_SOFT_SPI_MISO_PIN, MY_SOFT_SPI_MOSI_PIN, MY_SOFT_SPI_SCK_PIN, 0> hwSPI; //!< hwSPI
#else
#define hwSPI SPI //!< hwSPI
#endif

#ifndef DOXYGEN
#define MY_CRITICAL_SECTION     ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif  /* DOXYGEN */

#endif
