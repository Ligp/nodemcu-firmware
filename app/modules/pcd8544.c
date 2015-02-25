#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "auxmods.h"
#include "lrotable.h"


#include "c_types.h"
#include "pcd8544_func.h"
#include "easygpio.h"
#include "osapi.h"
#include "ets_sys.h"
#include "os_type.h"
#include "gpio.h"

////////////////////////////////////////////////////////////////
/////////////////////easygpio C ////////////////////////////////

/**
 * Returns the number of active pins in the gpioMask.
 */
uint8_t ICACHE_FLASH_ATTR
easygpio_countBits(uint32_t gpioMask) {

  uint8_t i=0;
  uint8_t numberOfPins=0;
  for (i=0; i<32; i++){
    numberOfPins += (gpioMask & 1<<i)?1:0;
  }
  return numberOfPins;
}

/**
 * Returns the gpio name and func for a specific pin.
 */
bool ICACHE_FLASH_ATTR
easygpio_getGpioNameFunc(uint8_t gpio_pin, uint32_t *gpio_name, uint8_t *gpio_func ) {

  if (gpio_pin == 6 || gpio_pin == 7 || gpio_pin == 8 || gpio_pin == 11 || gpio_pin >= 17) {
    os_printf("easygpio_getGpioNameFunc Error: There is no GPIO%d, check your code\n", gpio_pin);
    return false;
  }
  if (gpio_pin == 16) {
    os_printf("easygpio_getGpioNameFunc Error: GPIO16 is not implemented\n");
    return false;
  }
  switch ( gpio_pin ) {
    case 0:
      *gpio_func = FUNC_GPIO0;
      *gpio_name = PERIPHS_IO_MUX_GPIO0_U;
      return true;
    case 1:
      *gpio_func = FUNC_GPIO1;
      *gpio_name = PERIPHS_IO_MUX_U0TXD_U;
      return true;
    case 2:
      *gpio_func = FUNC_GPIO2;
      *gpio_name = PERIPHS_IO_MUX_GPIO2_U;
      return true;
    case 3:
      *gpio_func = FUNC_GPIO3;
      *gpio_name = PERIPHS_IO_MUX_U0RXD_U;
      return true;
    case 4:
      *gpio_func = FUNC_GPIO4;
      *gpio_name = PERIPHS_IO_MUX_GPIO4_U;
      return true;
    case 5:
      *gpio_func = FUNC_GPIO5;
      *gpio_name = PERIPHS_IO_MUX_GPIO5_U;
      return true;
    case 9:
      *gpio_func = FUNC_GPIO9;
      *gpio_name = PERIPHS_IO_MUX_SD_DATA2_U;
      return true;
    case 10:
      *gpio_func = FUNC_GPIO10;
      *gpio_name = PERIPHS_IO_MUX_SD_DATA3_U;
      return true;
    case 12:
      *gpio_func = FUNC_GPIO12;
      *gpio_name = PERIPHS_IO_MUX_MTDI_U;
      return true;
    case 13:
      *gpio_func = FUNC_GPIO13;
      *gpio_name = PERIPHS_IO_MUX_MTCK_U;
      return true;
    case 14:
      *gpio_func = FUNC_GPIO14;
      *gpio_name = PERIPHS_IO_MUX_MTMS_U;
      return true;
    case 15:
      *gpio_func = FUNC_GPIO15;
      *gpio_name = PERIPHS_IO_MUX_MTDO_U;
      return true;
    default:
      return false;
  }
  return true;
}


/**
 * Sets the pull up and pull down registers for a pin.
 */
static void ICACHE_FLASH_ATTR
easygpio_setupPulls(uint32_t gpio_name, bool pullUp, bool pullDown) {
  if (pullUp){
    PIN_PULLDWN_DIS(gpio_name);
    PIN_PULLUP_EN(gpio_name);
  } else if (pullDown){
    PIN_PULLUP_DIS(gpio_name);
    PIN_PULLDWN_EN(gpio_name);
  } else {
    PIN_PULLDWN_DIS(gpio_name);
    PIN_PULLUP_DIS(gpio_name);
  }
}

/**
 * Sets the 'gpio_pin' pin as an input GPIO and sets the pull up and
 * pull down registers for that pin.
 */
bool ICACHE_FLASH_ATTR
easygpio_setupAsInput(uint8_t gpio_pin, bool pullUp, bool pullDown) {
  uint32_t gpio_name;
  uint8_t gpio_func;

  if (!easygpio_getGpioNameFunc(gpio_pin, &gpio_name, &gpio_func) ) {
    return false;
  }

  PIN_FUNC_SELECT(gpio_name, gpio_func);
  easygpio_setupPulls(gpio_name, pullUp, pullDown);

  GPIO_DIS_OUTPUT(gpio_pin);
  return true;
}

/**
 * Sets the 'gpio_pin' pin as a GPIO output
 */
bool ICACHE_FLASH_ATTR
easygpio_setupAsOutput(uint8_t gpio_pin) {
  uint32_t gpio_name;
  uint8_t gpio_func;

  if (!easygpio_getGpioNameFunc(gpio_pin, &gpio_name, &gpio_func) ) {
    return false;
  }
  PIN_FUNC_SELECT(gpio_name, gpio_func);

  return true;
}


/**
 * Sets the 'gpio_pin' pin as a GPIO and sets the interrupt to trigger on that pin
 */
bool ICACHE_FLASH_ATTR
easygpio_setupInterrupt(uint8_t gpio_pin, bool pullUp, bool pullDown, void (*interruptHandler)(void)) {
  uint32_t gpio_name;
  uint8_t gpio_func;

  if (gpio_pin == 6 || gpio_pin == 7 || gpio_pin == 8 || gpio_pin == 11 || gpio_pin >= 17) {
    os_printf("easygpio_setupInterrupt Error: There is no GPIO%d, check your code\n", gpio_pin);
    return false;
  }
  if (gpio_pin == 16) {
    os_printf("easygpio_setupInterrupt Error: GPIO16 does not have interrupts\n");
    return false;
  }
  if (!easygpio_getGpioNameFunc(gpio_pin, &gpio_name, &gpio_func) ) {
    return false;
  }

  ETS_GPIO_INTR_ATTACH(interruptHandler, NULL);
  ETS_GPIO_INTR_DISABLE();

  PIN_FUNC_SELECT(gpio_name, gpio_func);

  easygpio_setupPulls(gpio_name, pullUp, pullDown);

  // disable output
  GPIO_DIS_OUTPUT(gpio_pin);

  gpio_register_set(GPIO_PIN_ADDR(gpio_pin), GPIO_PIN_INT_TYPE_SET(GPIO_PIN_INTR_DISABLE)
                    | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_DISABLE)
                    | GPIO_PIN_SOURCE_SET(GPIO_AS_PIN_SOURCE));

  //clear gpio14 status
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(gpio_pin));
  ETS_GPIO_INTR_ENABLE();

  return true;
}



///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////



// These default pin definitions can be changed to any valid GPIO pin.
// The code in PCD8544_init() should automagically adapt

static uint8_t pinReset=  4;  // LCD RST .... Pin 1
static uint8_t pinSce  =  5;  // LCD CE  .... Pin 2
static uint8_t pinDc   = 12;  // LCD DC  .... Pin 3
static uint8_t pinSdin = 13;  // LCD Din .... Pin 4
static uint8_t pinSclk = 14;  // LCD Clk .... Pin 5
                              // LCD Vcc .... Pin 6
                              // LCD BL  .... Pin 7
                              // LCD Gnd .... Pin 8

static bool PCD8544_isInitiated = false;

#define LOW       0
#define HIGH      1
#define LCD_CMD   LOW
#define LCD_DATA  HIGH

#define LCD_X     84
#define LCD_Y     48

#define CLOCK_HIGH_TIME 14 // 14 us

void PCD8544_lcdWrite8(bool dc, uint8_t data);
void PCD8544_shiftOut8(bool msbFirst, uint8_t data);
void PCD8544_printBinary(uint32_t data);
uint8_t PCD8544_pixelAt(uint8_t *image, uint8_t x, uint8_t y);
bool PCD8544_enableGPIOs(uint32_t gpioMask);

static const uint8_t ASCII[][5] =
{
    {0x00, 0x00, 0x00, 0x00, 0x00}  // 20
    ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
    ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
    ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
    ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
    ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
    ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
    ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
    ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
    ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
    ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
    ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
    ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
    ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
    ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
    ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
    ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
    ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
    ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
    ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
    ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
    ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
    ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
    ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
    ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
    ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
    ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
    ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
    ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
    ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
    ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
    ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
    ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
    ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
    ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
    ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
    ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
    ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
    ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
    ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
    ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
    ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
    ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
    ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
    ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
    ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
    ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
    ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
    ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
    ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
    ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
    ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
    ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
    ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
    ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
    ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
    ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
    ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
    ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
    ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
    ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c Â¥
    ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
    ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
    ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
    ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
    ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
    ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
    ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
    ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
    ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
    ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
    ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
    ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
    ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
    ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
    ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
    ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
    ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
    ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
    ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
    ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
    ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
    ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
    ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
    ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
    ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
    ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
    ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
    ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
    ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
    ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
    ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
    ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
    ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
    ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e â†�
    ,{0x00, 0x06, 0x09, 0x09, 0x06} // 7f â†’
};

void ICACHE_FLASH_ATTR
PCD8544_printBinary(uint32_t data){
  int i = 0;
  for (i=8*sizeof(uint32_t)-1; i>=0; i--){
    if (i > 1 && (i+1)%4==0) os_printf(" ");
    if (data & 1<<i){
      os_printf("1");
    } else {
      os_printf("0");
    }
  }
}

// PCD8544_gotoXY routine to position cursor
// x - range: 0 to 84
// y - range: 0 to 5
void ICACHE_FLASH_ATTR
PCD8544_gotoXY(int x, int y) {
  PCD8544_lcdWrite8( LCD_CMD, 0x80 | x);  // Column.
  PCD8544_lcdWrite8( LCD_CMD, 0x40 | y);  // Row.
}


void ICACHE_FLASH_ATTR
PCD8544_lcdCharacter(char character) {
  PCD8544_lcdWrite8(LCD_DATA, 0x00);
  if (character<0x20 || character>0x7f) {
    character = '?';
  }
  int index = 0;
  for (; index < 5; index++) {
    PCD8544_lcdWrite8(LCD_DATA, ASCII[character - 0x20][index]);
  }
  PCD8544_lcdWrite8(LCD_DATA, 0x00);
}

void ICACHE_FLASH_ATTR
PCD8544_lcdPrint(char *characters) {
  while (*characters) {
    PCD8544_lcdCharacter(*characters++);
  }
}

void ICACHE_FLASH_ATTR
PCD8544_lcdClear(void) {
  int index = 0;
  for (; index < LCD_X * LCD_Y / 8; index++){
    PCD8544_lcdWrite8(LCD_DATA, 0x00);
  }
}

/**
 * This does *not* work yet
 */
uint8_t ICACHE_FLASH_ATTR
#include "gpio.h"
PCD8544_pixelAt(uint8_t *image, uint8_t x, uint8_t y) {
  int aByte = (11*y+x)>>0x3;
  uint8_t aBit = 7-((11*y+x) & 0x7);
  return (image[aByte] >> aBit) &1;
}

void ICACHE_FLASH_ATTR
PCD8544_lcdImage(uint8_t *image) {
  int index = 0;
  for (; index < LCD_X * LCD_Y / 8; index++) {
    PCD8544_lcdWrite8(LCD_DATA, image[index]);
  }
}

void ICACHE_FLASH_ATTR
PCD8544_shiftOut8(bool msbFirst, uint8_t data) {
  bool dataBit = false;
  int bit = 7; // byte indexes
  if (msbFirst) {
    for (bit = 7; bit>=0; bit--) {
      dataBit = (data>>bit)&1;
      os_delay_us(CLOCK_HIGH_TIME);
      GPIO_OUTPUT_SET(pinSclk, 0);
      GPIO_OUTPUT_SET(pinSdin, dataBit);
      os_delay_us(CLOCK_HIGH_TIME);
      GPIO_OUTPUT_SET(pinSclk, 1);
      os_delay_us(CLOCK_HIGH_TIME);
      GPIO_OUTPUT_SET(pinSclk, 0);
    }
  } else {
    for (bit = 0; bit<=7; bit++) {
      dataBit = (data>>bit)&1;
      os_delay_us(CLOCK_HIGH_TIME);
      GPIO_OUTPUT_SET(pinSclk, 0);
      GPIO_OUTPUT_SET(pinSdin, dataBit);
      os_delay_us(CLOCK_HIGH_TIME);
      GPIO_OUTPUT_SET(pinSclk, 1);
      os_delay_us(CLOCK_HIGH_TIME);
      GPIO_OUTPUT_SET(pinSclk, 0);
    }
  }
  os_delay_us(2*CLOCK_HIGH_TIME);
}

void ICACHE_FLASH_ATTR
PCD8544_lcdWrite8(bool dc, uint8_t data) {
  GPIO_OUTPUT_SET(pinDc, dc);
  GPIO_OUTPUT_SET(pinSce, LOW);
  PCD8544_shiftOut8(true, data);
  GPIO_OUTPUT_SET(pinSce, HIGH);
  os_delay_us(CLOCK_HIGH_TIME);
}

void ICACHE_FLASH_ATTR
PCD8544_drawLine(void) {
  uint8_t  j;
  for(j=0; j<84; j++) { // top
    PCD8544_gotoXY(j,0);
    PCD8544_lcdWrite8(LCD_DATA,0x01);
  }
  for(j=0; j<84; j++) { //Bottom

    PCD8544_gotoXY(j,5);
    PCD8544_lcdWrite8(LCD_DATA,0x80);
  }
  for(j=0; j<6; j++) {// Right

    PCD8544_gotoXY(83,j);
    PCD8544_lcdWrite8(LCD_DATA,0xff);
  }
  for(j=0; j<6; j++) {// Left

    PCD8544_gotoXY(0,j);
    PCD8544_lcdWrite8(LCD_DATA,0xff);
  }
}

/**
 * Set up the GPIO pins and the rest of the environment
 */
/**
 * Sets the 'gpio_pin' pin as a GPIO output
 */

void ICACHE_FLASH_ATTR PCD8544_init(PCD8544_Settings *settings) {

  if (settings!=NULL){
    pinReset = settings->resetPin;
    pinSce   = settings->scePin;
    pinDc    = settings->dcPin;
    pinSdin  = settings->sdinPin;
    pinSclk  = settings->sclkPin;
  }

  if (easygpio_countBits(1<<pinReset|1<<pinSce|1<<pinDc|1<<pinSdin|1<<pinSclk)!=5) {
    os_printf("PCD8544_init Error: you must specify exactly 5 unique pin numbers\n");
    return;
  }

  // Define each used pin as a GPIO output
  if (!(easygpio_setupAsOutput(pinReset) &&
        easygpio_setupAsOutput(pinSce) &&
        easygpio_setupAsOutput(pinDc) &&
        easygpio_setupAsOutput(pinSdin) &&
        easygpio_setupAsOutput(pinSclk))) {
    return;
  }

  // Set default pin output values
  GPIO_OUTPUT_SET(pinReset, HIGH);
  GPIO_OUTPUT_SET(pinSce, HIGH);
  GPIO_OUTPUT_SET(pinDc, HIGH);
  GPIO_OUTPUT_SET(pinSdin, LOW);
  GPIO_OUTPUT_SET(pinSclk, LOW);

  PCD8544_isInitiated = true;
  PCD8544_initLCD(settings);
}

/**
 * initiate the LCD itself
 */
void ICACHE_FLASH_ATTR
PCD8544_initLCD(PCD8544_Settings *settings) {
  if (!PCD8544_isInitiated) {
    os_printf("PCD8544 module is not completely configured. Check your pin definitions.\r\n");
    return;
  }

  os_delay_us(10000);
  GPIO_OUTPUT_SET(pinReset, LOW);
  os_delay_us(CLOCK_HIGH_TIME*3);
  GPIO_OUTPUT_SET(pinReset, HIGH);
  os_delay_us(10000);

  PCD8544_lcdWrite8( LCD_CMD, 0x21 );  // LCD Extended Commands.
  if (settings!=NULL) {
    PCD8544_lcdWrite8( LCD_CMD, settings->lcdVop );     // Set LCD Vop (Contrast). //B1
    PCD8544_lcdWrite8( LCD_CMD, settings->tempCoeff );  // Set Temp coefficent. //0x04
    PCD8544_lcdWrite8( LCD_CMD, settings->biasMode  );  // LCD bias mode 1:48. //0x13
    PCD8544_lcdWrite8( LCD_CMD, settings->inverse?0x0d:0x0C );  // LCD 0x0C in normal mode. 0x0d for inverse
  } else {
    PCD8544_lcdWrite8( LCD_CMD, 0xB1 );  // Set LCD Vop (Contrast). //B1
    PCD8544_lcdWrite8( LCD_CMD, 0x04 );  // Set Temp coefficent. //0x04
    PCD8544_lcdWrite8( LCD_CMD, 0x14 );  // LCD bias mode 1:48. //0x13
    PCD8544_lcdWrite8( LCD_CMD, 0x0C );  // LCD in normal mode. 0x0d for inverse
  }

  PCD8544_lcdWrite8( LCD_CMD, 0x20);
  PCD8544_lcdWrite8( LCD_CMD, 0x0C);
  os_delay_us(100000);
  PCD8544_lcdClear();
  os_delay_us(10000);

}
static PCD8544_Settings pcd8544_settings;
static int pcd8544_setup( lua_State* L )
{
	int resetPin,scePin,dcPin,sdinPin,sclkPin;

	resetPin=lua_tointeger(L, 1);
	scePin=lua_tointeger(L, 2);
	dcPin=lua_tointeger(L, 3);
	sdinPin=lua_tointeger(L, 4);
	sclkPin=lua_tointeger(L, 5);


  pcd8544_settings.lcdVop = 0xB1;
  pcd8544_settings.tempCoeff = 0x04;
  pcd8544_settings.biasMode = 0x14;
  pcd8544_settings.inverse = false;

  pcd8544_settings.resetPin = resetPin;
    pcd8544_settings.scePin = scePin;

    pcd8544_settings.dcPin = dcPin;
    pcd8544_settings.sdinPin = sdinPin;
    pcd8544_settings.sclkPin = sclkPin;

  /*
  pcd8544_settings.resetPin = 4;
  pcd8544_settings.scePin = 5;

  pcd8544_settings.dcPin = 12;
  pcd8544_settings.sdinPin = 13;
  pcd8544_settings.sclkPin = 14;
*/

  PCD8544_init(&pcd8544_settings);

  return 1;
}
static int pcd8544_print( lua_State* L )
{
	int x,y;
	x= lua_tointeger(L, 1);
	y= lua_tointeger(L, 2);

  size_t length;
   const char *buffer = luaL_checklstring(L, 3, &length);
   PCD8544_gotoXY(x,y);
       // Put text in Box
   while (*buffer) {
       PCD8544_lcdCharacter(*buffer++);
     }
  return 1;
}

static int pcd8544_clear( lua_State* L )
{
	int index = 0;
	  for (; index < LCD_X * LCD_Y / 8; index++){
	    PCD8544_lcdWrite8(LCD_DATA, 0x00);
	  }
  return 1;
}

static int pcd8544_drwline( lua_State* L )
{
	int index = 0;
	  for (; index < LCD_X * LCD_Y / 8; index++){
	    PCD8544_lcdWrite8(LCD_DATA, 0x00);
	  }
  return 1;
}
#define PCD8544_FUNCTIONSET 0x20
#define PCD8544_SETVOP 0x80
#define PCD8544_EXTENDEDINSTRUCTION 0x01

static int pcd8544_contrast( lua_State* L )
{
	uint8_t contrast=lua_tointeger(L, 1);
	contrast &= 0x7f;
    char buf[64];
    os_sprintf(buf,"%d   ", contrast);

	  PCD8544_lcdWrite8( LCD_CMD, PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION );
	  PCD8544_lcdWrite8( LCD_CMD, PCD8544_SETVOP | contrast);
	  PCD8544_lcdWrite8( LCD_CMD, PCD8544_FUNCTIONSET);
return luaG_errormsg(L,buf);
}

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"
const LUA_REG_TYPE pcd8544_map[] =
{
		{ LSTRKEY( "init" ), LFUNCVAL( pcd8544_setup )},
		{ LSTRKEY( "print" ), LFUNCVAL( pcd8544_print)},
		  { LSTRKEY( "clear" ), LFUNCVAL( pcd8544_clear)},
		  { LSTRKEY( "drwline" ), LFUNCVAL( pcd8544_drwline)},
		  { LSTRKEY( "contrast" ), LFUNCVAL( pcd8544_contrast)},
  { LNILKEY, LNILVAL}
};

LUALIB_API int luaopen_pcd8544(lua_State *L) {
  // TODO: Make sure that the GPIO system is initialized
  LREGISTER(L, "pcd8544", pcd8544_map);
  return 1;
}

// ----------------------------------------------------------------------------

