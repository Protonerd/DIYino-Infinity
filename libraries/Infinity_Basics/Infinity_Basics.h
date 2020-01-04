/******************************************************************************
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 ******************************************************************************/
/*
 * Infinity_Basics.h
 *
 *  Created on: Jan 4, 2020
 *      Author: Protowerkstatt
 */

#ifndef INFINITY_BASICS_H_
#define INFINITY_BASICS_H_

#include <Arduino_nRF5x_lowPower.h> // LowPower Library for nRF5x
#include <Arduino.h>
#include "MPU6050.h" // needed for function to put MPU into sleep mode

// *********************************************************
// Internal connections:
// *********************************************************

#define CONFIG_NFCT_PINS_AS_GPIOS
//const uint32_t UICR_ADDR_0x20C __attribute__ ((section(".uicrNfcPinsAddress"))) __attribute__((used)) = 0xFFFFFFFE;
#define PSW 9 // 3.3V power switch enable shared with NFC1 !!!
#define GPIO1 A0
#define GPIO2 A1
#define GPIO3 A2
#define GPIO4 A3
#define GPIO5 10 // shared with NFC2 !!!
#define GPIO6 20 // shared with DFU !!!
#define GPIO7 18 // shared with SWO !!!
#define P1 A0
#define P2 A1
#define P3 A2
#define P4 A3
#define P5 10 // shared with NFC2 !!!
#define P6 20 // shared with DFU !!!
#define P7 18 // shared with SWO !!!
#define LS1 15
#define LS2 16
#define LS3 17
#define LS4 19
#define I2C_SCL 16
#define I2C_SDA 15

// *********************************************************
// Internal connections:
// *********************************************************
// I2S and Audio Amp Interface
#define PIN_I2S_MCK 13
#define PIN_I2S_BCLK A4
#define PIN_I2S_LRCK A5
#define PIN_I2S_DIN A6
#define PIN_I2S_SD  27
// SPI1 Interface to SD-Card
#define PIN_SPI_CS  11
#define PIN_SCL 16
#define PIN_SDA 15
// MPU Interrupt
#define MINT 7 // Pin connected to MPU Interrupt
// Battery Monitoring input via voltage divider
#define BATMON A7 // Pin connected to the battery to monitor it, via a resistive divider with a division factor of 2MOHms+806kOHms)/806kOHms=~0.712

// *********************************************************
// Battery Montitor defines
// *********************************************************

#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#ifdef NRF52840_XXAA    // if this is for nrf52840
#define VBAT_DIVIDER      (0.5F)               // 150K + 150K voltage divider on VBAT
#define VBAT_DIVIDER_COMP (2.0F)          // Compensation factor for the VBAT divider
#else
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
#endif
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


class Infinity_Basics
{
public:

	Infinity_Basics();
	/**
	 * Initialize. Do anything that should happen on startup.
	 */
	//void Init() = 0;

// Infinity House Keeping Functions
	void enterDeepSleep (uint8_t WakeUpOnPin, void (*f_isr)());
	void enterLowPowerMode ();

//private:
	static void ISR_deepSleep();


}; // class definition



#endif /* INFINITY_BASICS_H_ */
