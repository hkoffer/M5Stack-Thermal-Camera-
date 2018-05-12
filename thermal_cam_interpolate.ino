
/***************************************************************************
  This is a library for the AMG88xx GridEYE 8x8 IR camera

  This sketch makes an inetrpolated pixel thermal camera with the
  GridEYE sensor and a 2.4" tft featherwing:
	 https://www.adafruit.com/product/3315

  Designed specifically to work with the Adafruit AMG8833 Featherwing
          https://www.adafruit.com/product/3622

  These sensors use I2C to communicate. The device's I2C address is 0x69

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Dean Miller, James DeVito & ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <M5Stack.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
//low range of the sensor (this will be blue on the screen)
byte MINTEMP = 20;

//high range of the sensor (this will be red on the screen)
byte MAXTEMP = 32;

//the colors we will be using
const uint16_t camColors[] = {0x480F,
                              0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810, 0x3010, 0x3010,
                              0x3010, 0x2810, 0x2810, 0x2810, 0x2810, 0x2010, 0x2010, 0x2010, 0x1810, 0x1810,
                              0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811, 0x0811, 0x0811, 0x0011, 0x0011,
                              0x0011, 0x0011, 0x0011, 0x0031, 0x0031, 0x0051, 0x0072, 0x0072, 0x0092, 0x00B2,
                              0x00B2, 0x00D2, 0x00F2, 0x00F2, 0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192,
                              0x0192, 0x01B2, 0x01D2, 0x01F3, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0253, 0x0273,
                              0x0293, 0x02B3, 0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353, 0x0373,
                              0x0394, 0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474,
                              0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554, 0x0574,
                              0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572, 0x0571, 0x0591, 0x0591,
                              0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE, 0x05AE, 0x05AD, 0x05AD,
                              0x05AD, 0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB, 0x05CA, 0x05CA, 0x05CA, 0x05C9,
                              0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7, 0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5,
                              0x05E5, 0x0604, 0x0604, 0x0604, 0x0603, 0x0603, 0x0602, 0x0602, 0x0601, 0x0621,
                              0x0621, 0x0620, 0x0620, 0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640,
                              0x1E40, 0x1E40, 0x2640, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60,
                              0x3E60, 0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680, 0x5E80, 0x5E80, 0x6680,
                              0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0, 0x8EA0,
                              0x8EC0, 0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0, 0xB6E0, 0xB6E0,
                              0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0, 0xCEE0, 0xD6E0, 0xD700, 0xDF00, 0xDEE0,
                              0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640, 0xE620, 0xE600, 0xE5E0, 0xE5C0,
                              0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520, 0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480,
                              0xE460, 0xEC40, 0xEC20, 0xEC00, 0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40,
                              0xEB20, 0xEB00, 0xEAE0, 0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200,
                              0xF1E0, 0xF1C0, 0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0,
                              0xF080, 0xF060, 0xF040, 0xF020, 0xF800,
                             };

Adafruit_AMG88xx amg;
#define AMG_COLS 8
#define AMG_ROWS 8
float pixels[AMG_COLS * AMG_ROWS];
#define INTERPOLATED_COLS 24
#define INTERPOLATED_ROWS 24
int max_v = 0;
int min_v = 80;
float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, float *dest, uint8_t dest_rows, uint8_t dest_cols);
void setup()
{
  M5.begin();
  M5.setWakeupButton(BUTTON_B_PIN);
  M5.Lcd.begin();
  M5.Lcd.setRotation(0);
  M5.Lcd.fillScreen(TFT_BLACK);
  int icolor = 255;
  for (int irow = 16; irow <= 223;  irow++)
  {
    M5.Lcd.drawRect(0, 0, 35, irow, camColors[icolor]);
    icolor--;
  }
  infodisplay();
  if (!amg.begin())
  {
    while (1)
    {
      delay(1);
    }
  }
}

void loop() {
  if (M5.BtnA.pressedFor(1000)) {
    MINTEMP = min_v;
    min_v = 80;
    infodisplay();
  }

  if (M5.BtnA.wasPressed()) {
    if (MINTEMP <= 0)
    {
      MINTEMP = MAXTEMP - 1;
      infodisplay();
    }
    else
    {
      MINTEMP--;
      infodisplay();
    }
  }
  if (M5.BtnB.pressedFor(1000)) {
    M5.powerOFF();
  }

  if (M5.BtnC.pressedFor(1000)) {
    MAXTEMP = max_v;
    max_v = 0;
    infodisplay();
  }

  if (M5.BtnC.wasPressed()) {
    if (MAXTEMP >= 80)
    {
      MAXTEMP = MINTEMP + 1;
      infodisplay();
    }
    else
    {
      MAXTEMP++;
      infodisplay();
    }
  }
  M5.update();
  //read all the pixels
  amg.readPixels(pixels);

  for (int i = 1; i <= AMG88xx_PIXEL_ARRAY_SIZE; i++)
  {
  }
  float dest_2d[INTERPOLATED_ROWS * INTERPOLATED_COLS];

  interpolate_image(pixels, AMG_ROWS, AMG_COLS, dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);

  uint16_t boxsize = min(M5.Lcd.width() / INTERPOLATED_COLS, M5.Lcd.height() / INTERPOLATED_COLS);

  drawpixels(dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS, boxsize, boxsize, false);
  max_v = INT_MIN;

  //  int max_i = 0;
  int spot_v = pixels[28];
  for ( int itemp = 0; itemp < sizeof(pixels) / sizeof(pixels[0]); itemp++ )
  {
    if ( pixels[itemp] > max_v )
    {
      max_v = pixels[itemp];
      //max_i = itemp;
    }

    if ( pixels[itemp] < min_v )
    {
      min_v = pixels[itemp];
      //max_i = itemp;
    }
  }
  M5.Lcd.setTextSize(2);
  M5.Lcd.fillRect(284, 18, 36, 16, TFT_BLACK);
  M5.Lcd.fillRect(284, 130, 36, 16, TFT_BLACK);
  M5.Lcd.setCursor(284, 18);
  M5.Lcd.setTextColor(TFT_WHITE);
  if (max_v > 80 | max_v < 0) {
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.printf("Err", 1);
  }
  else
  { M5.Lcd.print(max_v, 1);
    M5.Lcd.printf("C" , 1);
    M5.Lcd.setCursor(284, 130);
    M5.Lcd.print(spot_v, 1);
    M5.Lcd.printf("C" , 1);
    M5.Lcd.drawCircle(160, 120, 6, TFT_WHITE);
    M5.Lcd.drawLine(160, 110, 160, 130, TFT_WHITE);
    M5.Lcd.drawLine(150, 120, 170, 120, TFT_WHITE);
  }
}
/***infodisplay()*****/
void infodisplay(void) {
  M5.Lcd.setTextColor(TFT_WHITE);
  //     M5.Lcd.setCursor(288, 230);
  //   M5.Lcd.printf("Power" , 1);

  M5.Lcd.fillRect(0, 0, 36, 16, TFT_BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 1);
  M5.Lcd.print(MAXTEMP , 1);
  M5.Lcd.printf("C" , 1);
  M5.Lcd.setCursor(0, 225);
  M5.Lcd.fillRect(0, 225, 36, 16, TFT_BLACK);
  M5.Lcd.print(MINTEMP , 1);
  M5.Lcd.printf("C" , 1);
  M5.Lcd.setCursor(284, 0);
  M5.Lcd.printf("Max", 1);
  M5.Lcd.setCursor(284, 100);
  //  M5.Lcd.printf("Spot", 1);
  M5.Lcd.drawCircle(300, 120, 6, TFT_WHITE);
  M5.Lcd.drawLine(300, 110, 300, 130, TFT_WHITE);
  M5.Lcd.drawLine(290, 120, 310, 120, TFT_WHITE);
}

void drawpixels(float *p, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight, boolean showVal) {
  int colorTemp;
  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      float val = get_point(p, rows, cols, x, y);
      if (val >= MAXTEMP) colorTemp = MAXTEMP;
      else if (val <= MINTEMP) colorTemp = MINTEMP;
      else colorTemp = val;

      uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);
      //draw the pixels!
      uint16_t color;
      color = val * 2;
      M5.Lcd.fillRect(40 + boxWidth * x, boxHeight * y, boxWidth, boxHeight, camColors[colorIndex]);
    }
  }
}
