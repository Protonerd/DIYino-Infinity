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
 * Infinity_Basics.cpp
 *
 *  Created on: Jan 4, 2020
 *      Author: Protowerkstatt
 */

#include "Infinity_Basics.h"

/** Default constructor.
 */
Infinity_Basics::Infinity_Basics() {
}

MPU6050 inf_mpu;

void Infinity_Basics::enterDeepSleep (uint8_t WakeUpOnPin, void (*f_isr)()) {
      Serial.println(F("Deep Sleep Mode"));
      digitalWrite(PSW,HIGH);
      inf_mpu.setSleepEnabled(false);
      inf_mpu.setSleepEnabled(true);
      digitalWrite(PIN_I2S_BCLK,LOW);
      digitalWrite(PIN_I2S_LRCK,LOW);
      digitalWrite(PIN_I2S_DIN,LOW);
      digitalWrite(PIN_I2S_SD,LOW);      
      digitalWrite(PIN_SCL,LOW);
      digitalWrite(PIN_SDA,LOW);
      digitalWrite(PIN_SPI_CS,LOW);
      digitalWrite(SCK,LOW);
      //digitalWrite(MISO,LOW);
      digitalWrite(MOSI,LOW);
digitalWrite(LS1,LOW);
digitalWrite(LS2,LOW);
digitalWrite(LS3,LOW);
digitalWrite(LS4,LOW);
      // enable interrupt on the main button in order to be able to wake up the device, but only in sleep mode

      attachInterrupt(digitalPinToInterrupt(WakeUpOnPin), f_isr, FALLING);

      nRF5x_lowPower.enableWakeupByInterrupt(WakeUpOnPin, LOW);
      nrf_gpio_cfg_sense_input(WakeUpOnPin, NRF_GPIO_PIN_PULLUP , NRF_GPIO_PIN_SENSE_LOW);
      delay(500);
      Serial.println("Going to Deep Sleep...");
      nRF5x_lowPower.powerMode(POWER_MODE_OFF);
      Serial.println("Waking Up from Deep Sleep...");
      nRF5x_lowPower.disableWakeupByInterrupt(WakeUpOnPin);
      detachInterrupt(0);           

}

void Infinity_Basics::enterLowPowerMode () {

      Serial.println(F("Low Power Mode"));
      digitalWrite(PSW,HIGH);
      inf_mpu.setSleepEnabled(false);
      inf_mpu.setSleepEnabled(true);
      digitalWrite(PIN_I2S_BCLK,LOW);
      digitalWrite(PIN_I2S_LRCK,LOW);
      digitalWrite(PIN_I2S_DIN,LOW);
      digitalWrite(PIN_I2S_SD,LOW);      
      digitalWrite(PIN_SCL,LOW);
      digitalWrite(PIN_SDA,LOW);
      digitalWrite(PIN_SPI_CS,LOW);
      digitalWrite(SCK,LOW);
      //digitalWrite(MISO,LOW);
      digitalWrite(MOSI,LOW);
digitalWrite(LS1,LOW);
digitalWrite(LS2,LOW);
digitalWrite(LS3,LOW);
digitalWrite(LS4,LOW);

}

// private:
static void ISR_deepSleep() {
// do nothing
}
