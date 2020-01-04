#define PSW 9 // 3.3V power switch enable shared with NFC1 !!!
#define GPIO1 (2)  //A0
#define GPIO2 (3)  //A1
#define GPIO3 (4)  //A2
#define GPIO4 (5)  //A3
#define GPIO5 (10) // shared with NFC2 !!!
#define GPIO6 (20) // shared with DFU !!!
#define GPIO7 (18) // shared with SWO !!!
#define LS1   (15)
#define LS2   (26)
#define LS3   (17)
#define LS4   (19)

#define NeoPixelDots 8



#include <WS2812_nRF52.h>
WS2812_easyDMA pixels;

cRGB gColor;
// *****************************************************************************************************************

void Init_NeoPixel_SPI_easyDMA() {
  pixels.setOutput(GPIO6, GPIO4); // This initializes the NeoPixel library.
}

void Send_NeoPixel_Frame() {
  pixels.sync();
}

void Set_Pixel(uint8_t pixelIdx, cRGB color) {
    pixels.set_crgb_at(pixelIdx, color);
}

void ClearPixels(uint8_t numPixel) {

  cRGB lColor;
  
  lColor.r=0;
  lColor.g=0;
  lColor.b=0;
  for (uint8_t i=0;i<=numPixel;i++) {
    Set_Pixel(i, lColor);
  }

  Send_NeoPixel_Frame();

}

void setup()
{
  delay(1000);
  Serial.begin(115200);

  pinMode(LS1, OUTPUT);
  pinMode(LS2, OUTPUT);
  digitalWrite(LS1,HIGH);
  digitalWrite(LS2,HIGH);
  
  Serial.println( "*******************");
  Serial.println( "   Neopixel test   ");
  Serial.println( "*******************");

  Init_NeoPixel_SPI_easyDMA();
  delay(1000); // a certain wait time is needed for the SPI configuration to take effect
  
  ClearPixels(7);
  delay(1000);
  Serial.println( "pixel 1: lila");
  gColor.r=10;
  gColor.g=0;
  gColor.b=10;
  Set_Pixel(0, gColor);
  Send_NeoPixel_Frame();
  delay(1000);
  Serial.println( "pixel 2: red");
  gColor.r=10;
  gColor.g=0;
  gColor.b=0;
  Set_Pixel(1, gColor);
  Send_NeoPixel_Frame();
  delay(1000);
  Serial.println( "pixel 3: green");
  gColor.r=0;
  gColor.g=10;
  gColor.b=0;
  Set_Pixel(2, gColor);  
  Send_NeoPixel_Frame();
  delay(1000);
  Serial.println( "pixel 4: blue");
  gColor.r=0;
  gColor.g=0;
  gColor.b=10;
  Set_Pixel(3, gColor);
  Send_NeoPixel_Frame();
  delay(1000);
  Serial.println( "pixel 5: torquise");
  gColor.r=0;
  gColor.g=10;
  gColor.b=10;
  Set_Pixel(4, gColor);
  Send_NeoPixel_Frame();
  delay(1000);
  Serial.println( "pixel 6: yellow");
  gColor.r=10;
  gColor.g=10;
  gColor.b=0;
  Set_Pixel(5, gColor);
  Send_NeoPixel_Frame();
  delay(1000);
  Serial.println( "pixel 7: off");
  gColor.r=0;
  gColor.g=0;
  gColor.b=0;
  Set_Pixel(6, gColor);
  Send_NeoPixel_Frame();
  delay(1000);
  Serial.println( "pixel 8: white");
  gColor.r=10;
  gColor.g=10;
  gColor.b=10;
  Set_Pixel(7, gColor);
  Send_NeoPixel_Frame();
  delay(1000);
}

void loop()
{
  delay(500);

  // toggle the first pixel
  gColor.r=10;
  gColor.g=0;
  gColor.b=0;
  Set_Pixel(1, gColor);
  
  Serial.println( "+++");
  Send_NeoPixel_Frame();
  
  delay(500);
  // toggle the first pixel
  gColor.r=0;
  gColor.g=0;
  gColor.b=0;
  Set_Pixel(1, gColor);
  Send_NeoPixel_Frame();
  
  Serial.println( "***");

}
