#ifndef TEMPERATURE_HEADER
#define TEMPERATURE_HEADER

/******************************************************************************/
/* temperature.h
/* header file for temperature module using BMP280 sensor and SSD1306 display
/* Author: Toon Leijtens, toon@ybzconsulting.com
/* Date: 2021.08.01.07:31PM
/* Version: 1.0
/* License: MIT
/******************************************************************************/

/* PHYSICAL CONNECTIONS AND PINS REQUIRED **************************************/
/*                                                                             */
/* Connect 3V3-OUT (physical pin 36) to Vin of BMP280 board                    */
/* Connect GND (physical pin 38) to Gnd of BMP280 board                        */

#define I2C0_SDA_PORT 4         /* Serial data at physical pin 6               */
                                /* -- connect to SD1 of BMP280 board           */

#define I2C0_SCL_PORT 5         /* Clock at physical pin 7                     */
                                /* -- connect to SCK of BMP280 board           */

#define LED_PIN 25              /* Onboard LED at physical pin 25              */
/*******************************************************************************/

/* ADDRESS OF BMP280 ***********************************************************/
/* when SD0 is not connected this Adafruit BMP280 module appears on address $77*/
static const uint8_t BMP280_ADDR = 0x77;
static const uint8_t BMP280_RESET_VAL = 0xB6;

/* ADDRESS OF SSD1306 128x64bit OLED Display module is $3C                     */
static const uint8_t SSD1306_ADDR = 0x3C;

/* Overscan values for temperature, pressure, and humidity *********************/
static const uint8_t OVERSCAN_DISABLE = 0x00;
static const uint8_t OVERSCAN_X1 = 0x01;
static const uint8_t OVERSCAN_X2 = 0x02;
static const uint8_t OVERSCAN_X4 = 0x03;
static const uint8_t OVERSCAN_X8 = 0x04;
static const uint8_t OVERSCAN_X16 = 0x05;

/* mode values *****************************************************************/
static const uint8_t MODE_SLEEP = 0x00;
static const uint8_t MODE_FORCE = 0x01;
static const uint8_t MODE_NORMAL = 0x03;

/* iir_filter values ***********************************************************/
static const uint8_t IIR_FILTER_DISABLE = 0x00;
static const uint8_t IIR_FILTER_X2 = 0x01;
static const uint8_t IIR_FILTER_X4 = 0x02;
static const uint8_t IIR_FILTER_X8 = 0x03;
static const uint8_t IIR_FILTER_X16 = 0x04;

/* BMP280 REGISTERS ************************************************************/
static const uint8_t BMP280REG_DIG_T1     = 0x88; /* ID register address       */
static const uint8_t BMP280REG_ID         = 0xD0; /* ID register address       */
static const uint8_t BMP280REG_TEMP       = 0xFA; /* Temperature register addr.*/
static const uint8_t BMP280REG_PRESSURE   = 0xF7; /* Pressure register address */
static const uint8_t BMP280REG_RESET      = 0xE0; /* Soft reset address        */
static const uint8_t BMP280REG_STATUS     = 0xF3; /* Status register address   */
static const uint8_t BMP280REG_CTRL_MEAS  = 0xF4; /* Data acquisition ctrl addr*/
static const uint8_t BMP280REG_CONFIG     = 0xF5; /* Config register address   */

/* SSD1306 REGISTERS ***********************************************************/
static const uint8_t SSD1306REG_COMMAND   = 0x80;  /* Command register         */
static const uint8_t SSD1306REG_CONTRAST  = 0x81;  /* Contrast register address*/
static const uint8_t SSD1306REG_ENTIRE_ON = 0xA4;  /* Output follows RAM       */
static const uint8_t SSD1306REG_NORM_INV  = 0xA6;  /* Output not inverted      */
static const uint8_t SSD1306REG_DISP      = 0xAE;  /* Display register         */
static const uint8_t SSD1306REG_MEM_ADDR  = 0x20;  /* Memory address register  */
static const uint8_t SSD1306REG_COL_ADDR  = 0x21;  /* Column address register  */
static const uint8_t SSD1306REG_PAGE_ADDR = 0x22;  /* Page address register    */
static const uint8_t SSD1306REG_DISP_START_LINE = 0x40; /* Startline of displ. */
static const uint8_t SSD1306REG_SEG_REMAP = 0xA0;  /* Segment remap register   */
static const uint8_t SSD1306REG_MUX_RATIO = 0xA8;  /* Mux ratio register       */
static const uint8_t SSD1306REG_COM_OUT_DIR = 0xC8; /* Communication out dir.  */
static const uint8_t SSD1306REG_DISP_OFFSET = 0xD3; /* Display offset reg.     */
static const uint8_t SSD1306REG_COM_PIN_CFG = 0xDA; /* Communication conf. reg.*/
static const uint8_t SSD1306REG_DISP_CLK_DIV = 0xD5; /* Divide ratio register  */
static const uint8_t SSD1306REG_PRECHARGE = 0xD9;  /* Pre-charge per. register */
static const uint8_t SSD1306REG_VCOM_DESEL = 0xDB; /* VCommunication conf. reg.*/
static const uint8_t SSD1306REG_CHARGE_PUMP = 0x8D; /* Enable charge pump reg. */

/* CONSTANTS FOR DISPLAY *******************************************************/
static const int WIDTH = 128;
static const int HEIGHT = 32;
static const bool EXT_POWER = false;

void reg_write( i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf,
          const uint8_t nbytes );

int reg_read( i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf,
          const uint8_t nbytes );

void init_BMP280( i2c_inst_t *i2c );

double get_temperature( uint8_t *data );

double get_pressure( uint8_t *data );

double get_height( double pressure, double p_atsealevel);

void init_SSD1306( i2c_inst_t *i2c, int width, int height, bool ext_pow );

void set_buffer( uint8_t *buffer, uint8_t value);

#endif