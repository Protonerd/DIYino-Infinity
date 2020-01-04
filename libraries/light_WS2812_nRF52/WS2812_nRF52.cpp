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
* light weight WS2812 lib for nRF52 processor V1.0 - Arduino support
*
* Controls WS2811/WS2812/WS2812B RGB-LEDs
* Author: Andras Kun/Stefan Gerstendoerfer (based on avr library of Matthias Riegler)
*
* Sept 24 2019: First version
*
*						(RGB, GRB, and BRG) --  Windell H. Oskay
* 
* License: GNU GPL v2.1 (see License.txt)
*/

#include "WS2812_nRF52.h"
#include <stdlib.h>


WS2812_easyDMA::WS2812_easyDMA() {
	count_led = MAXPIXEL;
	
	//pixels = (uint8_t*)malloc(MAXPIXEL *3);
	
	offsetGreen = 0;
	offsetRed = 1;
	offsetBlue = 2;
}

cRGB WS2812_easyDMA::get_crgb_at(uint16_t pixelIdx) {
	
	cRGB px_value;
	// function to be provided in the future
}

uint8_t WS2812_easyDMA::set_crgb_at(uint16_t pixelIdx, cRGB color) {

//	if(index < count_led) {
    // go through each bit of the cRGB colors
    // mask the bit we need by shifting it to LSB and ANDing it with 1

    // it is assumed that MSB of each byte is send first (to be clarified with Stefan)
    //                  MSB                         LSB
    //                    0   1   2   3   4   5   6   7
    // spi_buffer[0] ->   1   1 (b0)  0   0   1   1  (b1)
    // spi_buffer[1] ->   0   0   1   1 (b2)  0   0   1
    // spi_buffer[2] ->   1  (b3) 0   0   1   1 (b4)  0
    // spi_buffer[3] ->   0   1   1  (b5) 0   0   1   1
    // spi_buffer[4] -> (b6)  0   0   1   1 (b7)  0   0
    // color Red
    WS2812_easyDMA_Buffer.pixel[pixelIdx].r.pixel_buffer[0]=0x84 + ((color.r & 0x80)>>1) + ((color.r & 0x80)>>2) + ((color.r & 0x40)>>5) + ((color.r & 0x40)>>6);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].r.pixel_buffer[1]=0x21 + ((color.r & 0x20)>>1) + ((color.r & 0x20)>>2);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].r.pixel_buffer[2]=0x08 + ((color.r & 0x10)<<3) + ((color.r & 0x10)<<2) + ((color.r & 0x08)>>1) + ((color.r & 0x08)>>2);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].r.pixel_buffer[3]=0x42 + ((color.r & 0x04)<<3) + ((color.r & 0x04)<<2) + ((color.r & 0x02)>>1);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].r.pixel_buffer[4]=0x10 + ((color.r & 0x02)<<6) + ((color.r & 0x01)<<3) + ((color.r & 0x01)<<2);
    // color Green
    WS2812_easyDMA_Buffer.pixel[pixelIdx].g.pixel_buffer[0]=0x84 + ((color.g & 0x80)>>1) + ((color.g & 0x80)>>2) + ((color.g & 0x40)>>5) + ((color.g & 0x40)>>6);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].g.pixel_buffer[1]=0x21 + ((color.g & 0x20)>>1) + ((color.g & 0x20)>>2);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].g.pixel_buffer[2]=0x08 + ((color.g & 0x10)<<3) + ((color.g & 0x10)<<2) + ((color.g & 0x08)>>1) + ((color.g & 0x08)>>2);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].g.pixel_buffer[3]=0x42 + ((color.g & 0x04)<<3) + ((color.g & 0x04)<<2) + ((color.g & 0x02)>>1);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].g.pixel_buffer[4]=0x10 + ((color.g & 0x02)<<6) + ((color.g & 0x01)<<3) + ((color.g & 0x01)<<2);
    // color Blue
    WS2812_easyDMA_Buffer.pixel[pixelIdx].b.pixel_buffer[0]=0x84 + ((color.b & 0x80)>>1) + ((color.b & 0x80)>>2) + ((color.b & 0x40)>>5) + ((color.b & 0x40)>>6);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].b.pixel_buffer[1]=0x21 + ((color.b & 0x20)>>1) + ((color.b & 0x20)>>2);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].b.pixel_buffer[2]=0x08 + ((color.b & 0x10)<<3) + ((color.b & 0x10)<<2) + ((color.b & 0x08)>>1) + ((color.b & 0x08)>>2);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].b.pixel_buffer[3]=0x42 + ((color.b & 0x04)<<3) + ((color.b & 0x04)<<2) + ((color.b & 0x02)>>1);
    WS2812_easyDMA_Buffer.pixel[pixelIdx].b.pixel_buffer[4]=0x10 + ((color.b & 0x02)<<6) + ((color.b & 0x01)<<3) + ((color.b & 0x01)<<2);    

return 0;  
//}
//return 1;
	
}

void WS2812_easyDMA::sync() {
  WS2812_easyDMA_Buffer.pre_guard = 0x00;
  WS2812_easyDMA_Buffer.post_guard = 0x00;
  NRF_SPIM2->EVENTS_STOPPED = 0x0UL;
  NRF_SPIM2->TXD.MAXCNT = (15 * MAXPIXEL) + 2;
  NRF_SPIM2->TXD.PTR = (uint32_t) &WS2812_easyDMA_Buffer;
  NRF_SPIM2->TXD.LIST = SPIM_TXD_LIST_LIST_ArrayList;
  NRF_SPIM2->TASKS_START = 0x1UL;	
}

void WS2812_easyDMA::setColorOrderGRB() { // Default color order
	offsetGreen = 0;
	offsetRed = 1;
	offsetBlue = 2;
}

void WS2812_easyDMA::setColorOrderRGB() {
	offsetRed = 0;
	offsetGreen = 1;
	offsetBlue = 2;
}

void WS2812_easyDMA::setColorOrderBRG() {
	offsetBlue = 0;
	offsetRed = 1;
	offsetGreen = 2;
}

void WS2812_easyDMA::setColorOrderRBG() {
	offsetRed = 0;
	offsetBlue = 1;
	offsetGreen = 2;
}

WS2812_easyDMA::~WS2812_easyDMA() {
	
	
}

// this function initializes nRF52's easyDMA on SPI module 2
// and sets the 2 pins necessary: MOSI, which will act as
// the input of the WS2812 stripe and SCK which can be any
// digital I/O, even if not broken out. SCK is not connected
// anywhere, but it must be defined in order for easyDMA on
// SPI to work.

void WS2812_easyDMA::setOutput(uint8_t easyDMA_SPI_MOSI, uint8_t easyDMA_SPI_SCK) {
  NRF_SPIM2->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos);

  NRF_SPIM2->PSEL.SCK = easyDMA_SPI_SCK; // Connect SCK
  NRF_SPIM2->PSEL.MOSI = easyDMA_SPI_MOSI; // Connect only MOSI
//  pinMode (GPIO2, OUTPUT);
  NRF_SPIM2->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M4; // 4 MHz
  NRF_SPIM2->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
}
