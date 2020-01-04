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

#ifndef WS2812_EASYDMA_H_
#define WS2812_EASYDMA_H_

//#include <util/delay.h>
#include <stdint.h>

#include <Arduino.h>

struct cRGB { uint8_t g; uint8_t r; uint8_t b; };

#define MAXPIXEL 150

class WS2812_easyDMA {


public: 
	WS2812_easyDMA();
	~WS2812_easyDMA();
	
	void setOutput(uint8_t easyDMA_SPI_MOSI, uint8_t easyDMA_SPI_SCK);
	
	cRGB get_crgb_at(uint16_t pixelIdx);
	uint8_t set_crgb_at(uint16_t pixelIdx, cRGB px_value);
	void sync();
	void setColorOrderRGB();
	void setColorOrderGRB();
	void setColorOrderBRG();
	void setColorOrderRBG();


private:
	uint16_t count_led;
	uint8_t *pixels;
	uint8_t offsetRed;
	uint8_t offsetGreen;
	uint8_t offsetBlue;
// each color needs a buffer of 40 bits (i.e. 5 bytes) to store the needed SPI frames
// SPI is set to 4MHz, i.e. 250ns SPI clock, therefore 5 clock cycles is needed for one WS2812(B) frame which has a period time of 1250ns
typedef struct tcRGB_nRF52_color_buffer
{
  uint8_t pixel_buffer[5];
};

// define a structure for the pixel buffer which will be accessed by the easyDMA
typedef struct tNeoPixelBuffer
{
  tcRGB_nRF52_color_buffer g;
  tcRGB_nRF52_color_buffer r;
  tcRGB_nRF52_color_buffer b;
} tNeoPixelBuffer_t;

typedef struct tWS2812_easyDMA_Buffer
{
  uint8_t pre_guard;
  tNeoPixelBuffer_t pixel[MAXPIXEL];
  uint8_t post_guard;
} tWS2812_easyDMA_Buffer_t;

tWS2812_easyDMA_Buffer_t WS2812_easyDMA_Buffer;

};



#endif /* WS2812_EASYDMA_H_ */