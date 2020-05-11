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

#include "MyHwlgt8f.h"
#include "avr/boot.h"
#include "lgtx8p.h"
#include <PMU.h>


bool hwInit(void)
{
#if !defined(MY_DISABLED_SERIAL)
	MY_SERIALDEVICE.begin(MY_BAUD_RATE);
#if defined(MY_GATEWAY_SERIAL)
	while (!MY_SERIALDEVICE) {}
#endif
#endif
	return true;
}

#define WDTO_SLEEP_FOREVER		(0xFFu)

volatile uint8_t _wokeUpByInterrupt =
    INVALID_INTERRUPT_NUM;    // Interrupt number that woke the mcu.
volatile uint8_t _wakeUp1Interrupt  =
    INVALID_INTERRUPT_NUM;    // Interrupt number for wakeUp1-callback.
volatile uint8_t _wakeUp2Interrupt  =
    INVALID_INTERRUPT_NUM;    // Interrupt number for wakeUp2-callback.

static uint32_t sleepRemainingMs = 0ul;

void wakeUp1(void)
{
	// Disable sleep. When an interrupt occurs after attachInterrupt,
	// but before sleeping the CPU would not wake up.
	// Ref: http://playground.arduino.cc/Learning/ArduinoSleepCode
	sleep_disable();
	detachInterrupt(_wakeUp1Interrupt);
	// First interrupt occurred will be reported only
	if (INVALID_INTERRUPT_NUM == _wokeUpByInterrupt) {
		_wokeUpByInterrupt = _wakeUp1Interrupt;
	}
}
void wakeUp2(void)
{
	sleep_disable();
	detachInterrupt(_wakeUp2Interrupt);
	// First interrupt occurred will be reported only
	if (INVALID_INTERRUPT_NUM == _wokeUpByInterrupt) {
		_wokeUpByInterrupt = _wakeUp2Interrupt;
	}
}

inline bool interruptWakeUp(void)
{
	return _wokeUpByInterrupt != INVALID_INTERRUPT_NUM;
}

void clearPendingInterrupt(const uint8_t interrupt)
{
	EIFR = _BV(interrupt);
}

// Watchdog Timer interrupt service routine. This routine is required
// to allow automatic WDIF and WDIE bit clearance in hardware.
//ISR (WDT_vect)
//{
//}

void hwPowerDown(const uint8_t wdto)
{
	// Let serial prints finish (debug, log etc)
#ifndef MY_DISABLED_SERIAL
	MY_SERIALDEVICE.flush();
#endif

	PMU.sleep(PM_POFFS1, (period_t)wdto);

//	// disable ADC for power saving
//	ADCSRA &= ~(1 << ADEN);
//	// save WDT settings
//	const uint8_t WDTsave = WDTCSR;
//	if (wdto != WDTO_SLEEP_FOREVER) {
//		wdt_enable(wdto);
//		// enable WDT interrupt before system reset
//		WDTCSR |= (1 << WDCE) | (1 << WDIE);
//	} else {
//		// if sleeping forever, disable WDT
//		wdt_disable();
//	}
//	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//	cli();
//	sleep_enable();
//#if defined(__AVR_ATmega328P__)
//	sleep_bod_disable();
//#endif
//	// Enable interrupts & sleep until WDT or ext. interrupt
//	sei();
//	// Directly sleep CPU, to prevent race conditions!
//	// Ref: chapter 7.7 of ATMega328P datasheet
//	sleep_cpu();
//	sleep_disable();
//	// restore previous WDT settings
//	cli();
//	wdt_reset();
//	// enable WDT changes
//	WDTCSR |= (1 << WDCE) | (1 << WDE);
//	// restore saved WDT settings
//	WDTCSR = WDTsave;
//	sei();
//	// enable ADC
//	ADCSRA |= (1 << ADEN);
}

uint32_t hwInternalSleep(uint32_t ms)
{
	// WDP3 - WDP0
	// 32KHZ / 2MHZ
	// Sleeping with watchdog only supports multiples of 64ms/1ms.
	// Round up to next multiple of 16ms, to assure we sleep at least the
	// requested amount of time. Sleep of 0ms will not sleep at all!
	//9
	ms += 63u;

	while (!interruptWakeUp() && ms >= 64) {
		for (uint16_t period = 5u; ; --period) {
			const uint16_t comparatorMS = 1 << (period + 7);
			if ( ms >= comparatorMS) {

				Serial.print("ms sleep ");
				Serial.println(comparatorMS);

				Serial.print("period ");
				Serial.println(period);

				hwPowerDown(period); // 8192ms => 9, 16ms => 0
				ms -= comparatorMS;
				break;
			}
		}
	}
	if (interruptWakeUp()) {
		return ms;
	}
	return 0ul;
}

int8_t hwSleep(uint32_t ms)
{
	// Return what woke the mcu.
	// Default: no interrupt triggered, timer wake up
	int8_t ret = MY_WAKE_UP_BY_TIMER;
	sleepRemainingMs = 0ul;
	if (ms > 0u) {
		// sleep for defined time
		sleepRemainingMs = hwInternalSleep(ms);
	} else {
		// sleep until ext interrupt triggered
		PMU.sleep(PM_POFFS0, SLEEP_FOREVER);
		//hwPowerDown(WDTO_SLEEP_FOREVER);
	}
	if (interruptWakeUp()) {
		ret = static_cast<int8_t>(_wokeUpByInterrupt);
	}
	// Clear woke-up-by-interrupt flag, so next sleeps won't return immediately.
	_wokeUpByInterrupt = INVALID_INTERRUPT_NUM;

	return ret;
}

int8_t hwSleep(const uint8_t interrupt, const uint8_t mode, uint32_t ms)
{
	return hwSleep(interrupt, mode, INVALID_INTERRUPT_NUM, 0u, ms);
}

int8_t hwSleep(const uint8_t interrupt1, const uint8_t mode1, const uint8_t interrupt2,
               const uint8_t mode2,
               uint32_t ms)
{
	// ATMega328P supports following modes to wake from sleep: LOW, CHANGE, RISING, FALLING
	// Datasheet states only LOW can be used with INT0/1 to wake from sleep, which is incorrect.
	// Ref: http://gammon.com.au/interrupts

	// Disable interrupts until going to sleep, otherwise interrupts occurring between attachInterrupt()
	// and sleep might cause the ATMega to not wakeup from sleep as interrupt has already be handled!
	cli();
	// attach interrupts
	_wakeUp1Interrupt  = interrupt1;
	_wakeUp2Interrupt  = interrupt2;

	// Attach external interrupt handlers, and clear any pending interrupt flag
	// to prevent waking immediately again.
	// Ref: https://forum.arduino.cc/index.php?topic=59217.0
	if (interrupt1 != INVALID_INTERRUPT_NUM) {
		clearPendingInterrupt(interrupt1);
		attachInterrupt(interrupt1, wakeUp1, mode1);
	}
	if (interrupt2 != INVALID_INTERRUPT_NUM) {
		clearPendingInterrupt(interrupt2);
		attachInterrupt(interrupt2, wakeUp2, mode2);
	}

	sleepRemainingMs = 0ul;
	if (ms > 0u) {
		// sleep for defined time
		sleepRemainingMs = hwInternalSleep(ms);
	} else {
		// sleep until ext interrupt triggered
		hwPowerDown(WDTO_SLEEP_FOREVER);
	}

	// Assure any interrupts attached, will get detached when they did not occur.
	if (interrupt1 != INVALID_INTERRUPT_NUM) {
		detachInterrupt(interrupt1);
	}
	if (interrupt2 != INVALID_INTERRUPT_NUM) {
		detachInterrupt(interrupt2);
	}

	// Return what woke the mcu.
	// Default: no interrupt triggered, timer wake up
	int8_t ret = MY_WAKE_UP_BY_TIMER;
	if (interruptWakeUp()) {
		ret = static_cast<int8_t>(_wokeUpByInterrupt);
	}
	// Clear woke-up-by-interrupt flag, so next sleeps won't return immediately.
	_wokeUpByInterrupt = INVALID_INTERRUPT_NUM;

	return ret;
}

uint32_t hwGetSleepRemaining(void)
{
	return sleepRemainingMs;
}

inline void hwRandomNumberInit(void)
{
	// This function initializes the random number generator with a seed
	// of 32 bits.  This method is good enough to earn FIPS 140-2 conform
	// random data. This should reach to generate 32 Bit for randomSeed().
	uint32_t seed = 0;
	uint32_t timeout = millis() + 20;

	// Trigger floating effect of an unconnected pin
	pinMode(MY_SIGNING_SOFT_RANDOMSEED_PIN, INPUT_PULLUP);
	pinMode(MY_SIGNING_SOFT_RANDOMSEED_PIN, INPUT);
	delay(10);

	// Generate 32 bits of datas
	for (uint8_t i=0; i<32; i++) {
		const int pinValue = analogRead(MY_SIGNING_SOFT_RANDOMSEED_PIN);
		// Wait until the analog value has changed
		while ((pinValue == analogRead(MY_SIGNING_SOFT_RANDOMSEED_PIN)) && (timeout>=millis())) {
			seed ^= (millis() << i);
			// Check of data generation is slow
			if (timeout<=millis()) {
				// Trigger pin again
				pinMode(MY_SIGNING_SOFT_RANDOMSEED_PIN, INPUT_PULLUP);
				pinMode(MY_SIGNING_SOFT_RANDOMSEED_PIN, INPUT);
				// Pause a short while
				delay(seed % 10);
				timeout = millis() + 20;
			}
		}
	}
	randomSeed(seed);
}

void hwReadConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *dst = static_cast<uint8_t *>(buf);
	int pos = reinterpret_cast<int>(addr);
	while (length-- > 0) {
		*dst++ = EEPROM.read(pos++);
	}
}

void hwWriteConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *src = static_cast<uint8_t *>(buf);
	int pos = reinterpret_cast<int>(addr);
	while (length-- > 0) {
		EEPROM.write(pos++, *src++);
	}
}

uint8_t hwReadConfig(const int addr)
{
	uint8_t value;
	hwReadConfigBlock(&value, reinterpret_cast<void *>(addr), 1);
	return value;
}

void hwWriteConfig(const int addr, uint8_t value)
{
	hwWriteConfigBlock(&value, reinterpret_cast<void *>(addr), 1);
}


bool hwUniqueID(unique_id_t *uniqueID)
{
	// padding
	(void)memset(uniqueID, MY_HWID_PADDING_BYTE, sizeof(unique_id_t));

	*((uint8_t *)uniqueID) = GUID0;
	*((uint8_t *)uniqueID + 1) = GUID1;
	*((uint8_t *)uniqueID + 2) = GUID2;
	*((uint8_t *)uniqueID + 3) = GUID3;

	//uint32_t guid = *(uint32_t*)&GUID0;
	//*uniqueID = guid;
	return true;
}

uint16_t hwCPUVoltage(void)
{

	//http://arduino.ru/forum/apparatnye-voprosy/obzor-klona-megi328-lgt8f328p#comment-391675
	ADCSRB = 0;
	ADCSRC = 0;
	ADCSRA = 1 << ADEN | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0; // adc enable , clk/128
	ADMUX = 1 << REFS0 | 1 << 3 | 1 << 2 | 1 << 0; //ref=avcc, input= ivref 
	ADCSRD = 1 << BGEN | 1 << IVSEL0; //ivref=2048
	VCAL = VCAL2;//load 2048 calibrate byte
	uint32_t bgaread = 0;
	//собрать 256 семплов для усреднения
	for (int n = 0x0; n <= 0xff; n++) {
		ADCSRA |= (1 << ADSC);
		while (bit_is_set(ADCSRA, ADSC));
		bgaread += ADC;
	}
	bgaread >>= 8;
	bgaread = ((uint32_t)2048 << 12) / bgaread;

	return (uint16_t)bgaread;
}

uint16_t hwCPUFrequency(void)
{
/*
	cli();
	// save WDT & timer settings
	const uint8_t WDTsave = WDTCSR;
	const uint8_t TCCR1Asave = TCCR1A;
	const uint8_t TCCR1Csave = TCCR1C;
	// setup timer1
	TIFR1 = 0xFF;
	TCNT1 = 0;
	TCCR1A = 0;
	TCCR1C = 0;
	// set wdt
	wdt_enable(WDTO_500MS);
	// enable WDT interrupt mode => first timeout WDIF, 2nd timeout reset
	WDTCSR |= (1 << WDIE);
	wdt_reset();
	// start timer1 with 1024 prescaling
	TCCR1B = _BV(CS12) | _BV(CS10);
	// wait until wdt interrupt
	while (bit_is_clear(WDTCSR,WDIF)) {};
	// stop timer
	TCCR1B = 0;
	// restore WDT settings
	wdt_reset();
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = WDTsave;
	sei();
	// restore timer settings
	TCCR1A = TCCR1Asave;
	TCCR1C = TCCR1Csave;
	// return frequency in 1/10MHz (accuracy +- 10%)
	return TCNT1 * 2048UL / 100000UL;
*/
	return F_CPU / 100000UL;

}

int8_t hwCPUTemperature(void)
{
//#if defined(__AVR_ATmega168A__) || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328BP__) || defined(__AVR_ATmega32U4__)
//	// Set the internal reference and mux.
//	ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
//	ADCSRA |= _BV(ADEN);  // enable the ADC
//	delay(20);            // wait for voltages to become stable.
//	ADCSRA |= _BV(ADSC);  // Start the ADC
//	// Wait until conversion done
//	while (bit_is_set(ADCSRA, ADSC));
//	// temperature is in degrees Celsius
//	return static_cast<int8_t>((ADCW - MY_AVR_TEMPERATURE_OFFSET) / MY_AVR_TEMPERATURE_GAIN);
//#else
	return -127; // not available
//#endif
}

uint16_t hwFreeMem(void)
{
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}




