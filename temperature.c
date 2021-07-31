#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* SUMMARY *********************************************************************/
/*                   : Build for the Raskpberry PI PICO, rp2040                */
/*                   : T. Leijtens (toon.leijtens@gmail.com                    */
/*                   : 2021.07.24                                              */
/*******************************************************************************/

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
static const uint8_t BMP280REG_TEMP       = 0xFA; /* Temerature register addr. */
static const uint8_t BMP280REG_PRESSURE   = 0xF7; /* Pressure register address */
static const uint8_t BMP280REG_RESET      = 0xE0; /* Soft reset address        */
static const uint8_t BMP280REG_STATUS     = 0xF3; /* Status register address   */
static const uint8_t BMP280REG_CTRL_MEAS  = 0xF4; /* Data aquisition ctrl addr.*/
static const uint8_t BMP280REG_CONFIG     = 0xF5; /* Config register address   */

/* SSD1306 REGISTERS ***********************************************************/
static const uint8_t SSD1306REG_CONTRAST  = 0x81;  /* Contrast register address*/
static const uint8_t SSD1306REG_ENTIRE_ON = 0xA4;  /* Output follows RAM       */
static const uint8_t SSD1306REG_NORM_INV  = 0xA6;  /* Output not inverted      */
static const uint8_t SSD1306REG_DISPLAY   = 0xAE;  /* Display register         */
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

/* FUNCTION HEADERS ************************************************************/
void reg_write(      i2c_inst_t *i2c,
          const uint addr,
          const uint8_t reg,
          uint8_t *buf,
          const uint8_t nbytes );

int reg_read(       i2c_inst_t *i2c,
          const uint addr,
          const uint8_t reg,
          uint8_t *buf,
          const uint8_t nbytes );

void init_BMP280( i2c_inst_t *i2c );

double get_temperature( uint8_t *data );

double get_pressure( uint8_t *data );

double get_height( double pressure, double p_atsealevel);

void init_SSD1306( i2c_inst_t *i2c );

/* GLOBAL VARS *****************************************************************/
uint8_t coeficients[24]; /* coeficients required for temperature and pressure  */
double t_fine; /* needed to pass the var1 + var2 temperature to pressure calc. */

int main(void){

    // Variable to hold the temperature
    int temp_i = 0;

    // Ports
    i2c_inst_t *i2c = i2c0;

    // Allow this program to print to serial
    stdio_init_all();

    // Initialize onboard LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Configure the I2C Communications at 400Mhz
    i2c_init( i2c, 400000);

    // Define the pins that we use for this I2C module
    gpio_set_function(I2C0_SDA_PORT, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PORT, GPIO_FUNC_I2C);

    // Note: no pull-up necessary, this Adafruit module has added pull-up resistors on the board

    // Define number of bytes returned (n) and buffer (data) where we store the result
    int n = 0;
    uint8_t data[3];

    // Vars to hold the temperature and pressure
    double temperature = 0.0;
    double pressure = 0.0;
    double pressure_at_sealevel_for_location = 101100.0; // Eindhoven 2021.07.26 1011 milibar = 101100 Pa
    double height = 0.0;
    char abovebelow[5];

    // Initialize BMP280 Module
    init_BMP280( i2c);

    // Initialize SSD1306 Oled display Module
	init_SSD1306( i2c);

    while(1){

        // Get temperature from module
        n = reg_read( i2c, BMP280_ADDR, BMP280REG_TEMP, data, 3);

        // Get temperature from raw value and stored coeficients
        temperature = get_temperature( data);

        // Get temperature from module
        n = reg_read( i2c, BMP280_ADDR, BMP280REG_PRESSURE, data, 3);

        // Get temperature from raw value and stored coeficients
        pressure = get_pressure( data);

        // Get height at location
        height = get_height( pressure, pressure_at_sealevel_for_location);

        // Find out if we are above or below sealevel
        if( height < 0){
            sprintf(abovebelow, "below");
            height *= -1;
        } else {
            sprintf(abovebelow, "above");
        }

        printf("[-] %3.2f°C, %6.2fPa, %3.2fm %s sealevel.\n", temperature, pressure, height, abovebelow);

        // LED-ON
        gpio_put(LED_PIN, 1);

        sleep_ms(1000);

        // LED-OFF
        gpio_put(LED_PIN, 0);

    sleep_ms(1000);
    }

    return 0;
}

/* FUNCTION DEFINITIONS ********************************************************/

// Calcultate height given the p_measured and p_atsealevel for the location
double get_height( double pressure, double p_atsealevel){
    double p = 0.0;
    double ps = 0.0;
    double h = 0.0;

    p = pressure;
    ps = p_atsealevel;
    h = 44330 * (1.0 - pow( p/ps, 0.1903));

    return h;
}

// Calculate pressure from raw value and coeficients
double get_pressure( uint8_t *data) {
    double p = 0.0;
    double var1 = 0;
    double var2 = 0;
    double var3 = 0;
    int adc_p = 0;

    adc_p = data[0];
    adc_p <<= 8;
    adc_p |= data[1];
    adc_p <<= 4;
    adc_p |= (data[2] >> 4);

    var1 = (t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((int16_t)((coeficients[17] << 8) | coeficients[16])) / 32768.0;
    var2 = var2 + var1 * ((int16_t)((coeficients[15] << 8) | coeficients[14])) * 2.0;
    var2 = ( var2 / 4.0 ) + ((int16_t)((coeficients[13] << 8) | coeficients[12])) * 65536.0;
    var3 = ((int16_t)((coeficients[11] << 8) | coeficients[10])) * var1 * var1 / 524288.0;
    var1 = ( var3 + ((int16_t)((coeficients[9] << 8) | coeficients[8])) * var1) / 524288.0;
    var1 = ( 1.0 + var1 / 32768.0) * ((int32_t)((coeficients[7] << 8) | coeficients[6]));
    p = 1048576.0 - adc_p;
    p = ( p - ( var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((int16_t)((coeficients[23] << 8) | coeficients[22])) * p * p / 2147483648.0;
    var2 = p * ((int16_t)((coeficients[21] << 8) | coeficients[20])) / 32768.0;
    p = p + ( var1 + var2 + ((int16_t)((coeficients[19] << 8) | coeficients[18]))) / 16.0;

    return p;
}

// Calculate temperature from raw value and coeficients
double get_temperature( uint8_t *data) {
    double t = 0.0;
    double var1 = 0;
    double var2 = 0;
    int adc_t = 0;

    adc_t = data[0];
    adc_t <<= 8;
    adc_t |= data[1];
    adc_t <<= 4;
    adc_t |= (data[2] >> 4);

    var1 = (adc_t/16384.0 - ((int32_t)((coeficients[1] << 8) | coeficients[0]))/1024.0) *
           ((int16_t)((coeficients[3] << 8) | coeficients[2]));

    var2 = (adc_t/131072.0 - ((int16_t)((coeficients[1] << 8) | coeficients[0]))/8192.0) *
           (adc_t/131072.0 - ((int16_t)((coeficients[1] << 8) | coeficients[0]))/8192.0) *
           ((int16_t)((coeficients[5] << 8) | coeficients[4]));

    t_fine = var1 + var2;
    t = (var1 + var2) / 5120.0;

    return t;
}

// Init SSD1306 module
void init_SSD1306( i2c_inst_t *i2c){

	// Define data that we use to send init commands
	uint8_t data[3];

	// initialize bytes_read to zero
	int bytes_read = 0;

    // Give it some time to wakeup
    sleep_ms(1000);

    // Set SSD1306 display to off
    data[0] = 0x00;	// OFF
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_DISPL_OFF, &data[0], 1);

	// Set memory addressing to horizontal mode
    data[0] = 0x01;
    data[1] = 0x00;
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_MEM_ADDR, &data[0], 2);

	// Set contrast control
    data[0] = 0x01;
    data[1] = 0xCF;
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_CONTRAST, &data[0], 2);

	// Column 127 is segment 0
    data[0] = 0x00;
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_COLUMN127, &data[0], 1);

	// Set desplay to normal (not inversed)
    data[0] = 0x00;
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_NORM_INV, &data[0], 1);

	// Set desplay to normal
    data[0] = 0x00;
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_NORM_DISP, &data[0], 1);

	// Set mux ratio to 1/64
    data[0] = 0x01;
    data[1] = 0x3F;
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_MUX_RATIO, &data[0], 2);

	// Set divide ratio
    data[0] = 0x01;
    data[1] = 0x80;
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_DIV_RATIO, &data[0], 2);

	// Set pre-charge period
    data[0] = 0x01;
    data[1] = 0xF1;
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_PRE_CHRGE, &data[0], 2);

	// Set com configuration
    data[0] = 0x01;
    data[1] = 0x12;
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_COM_CONF, &data[0], 2);

	// Set vcom configuration
    data[0] = 0x01;
    data[1] = 0x40;
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_VCOM_CONF, &data[0], 2);

	// Enable charge pump
    data[0] = 0x01;
    data[1] = 0x14;
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_EN_CHPMP, &data[0], 2);

    // Set SSD1306 display to on
    data[0] = 0x00;	// ON
    reg_write( i2c, SSD1306_ADDR, SSD1306REG_DISPL_ON, &data[0], 1);
}
 
// Init BMP280 module
void init_BMP280( i2c_inst_t *i2c){

    // Prepare chipID var, which is the size of one byte
    uint8_t data[1];

    // initialize bytes_read to zero
    int bytes_read = 0;

    // Give it some time to wakeup
    sleep_ms(1000);

    // Read 1-byte from BMP280REG_ID and store the result in chipId
    bytes_read = reg_read( i2c, BMP280_ADDR, BMP280REG_ID, data, 1);
    if( bytes_read == 0) {
    printf("[!] Did not find ID if BMP280, read did not yield any value.\n");
    }

    // If chipID is $58, we have found our BMP280 module
    if(data[0] != 0x58) {
        while(1){
            printf("[!] Chip ID incorrect, we got 0x%2x\n",data[0]);
        }
    } else {
        sleep_ms(5000);
        printf("[+] Yes we found the BMP280 module with ID 0x%2x at address 0x%2x\n",data[0], BMP280_ADDR);
    }

    // Write BMP280_RESET_VAL to reset-register, to soft reset the device then wait for 4ms
    data[0] = BMP280_RESET_VAL;
    reg_write( i2c, BMP280_ADDR, BMP280REG_RESET, &data[0], 1);
    sleep_ms(4);

    // Write data aquisition control parameters for temperature and pressure

    data[0] = OVERSCAN_X2;    // SET Overscan for Temperature
    data[0] <<= 3;
    data[0] |= OVERSCAN_X16;    // SET Overscan for Pressure
    data[0] <<= 2;
    data[0] |= MODE_SLEEP;    // SET Mode to SLEEP

    // printf("[i] Data aquisition control set to: %X (SLEEP)\n", data[0]); 

    reg_write( i2c, BMP280_ADDR, BMP280REG_CTRL_MEAS, &data[0], 1);

    data[0] = IIR_FILTER_DISABLE;
    data[0] <<= 2;

    // printf("[i] Config value set to: %X\n", data[0]); 

    reg_write( i2c, BMP280_ADDR, BMP280REG_CONFIG, &data[0], 1);

    // Write data aquisition control parameters for temperature and pressure

    data[0] = OVERSCAN_X2;    // SET Overscan for Temperature
    data[0] <<=3;
    data[0] |= OVERSCAN_X16;    // SET Overscan for Pressure
    data[0] <<= 2;
    data[0] |= MODE_NORMAL;    // SET Mode to NORMAL

    // printf("[i] Data aquisition control set to: %X (NORMAL)\n", data[0]); 

    reg_write( i2c, BMP280_ADDR, BMP280REG_CTRL_MEAS, &data[0], 1);

    // Wait until the NORMAL operation mode has settled
    do {
        sleep_ms(2);
        // Get status and wait until bit-4 is set
        bytes_read = reg_read( i2c, BMP280_ADDR, BMP280REG_STATUS, data, 1);

    } while( data[0] & 0x08);

    // Read coeficients from device, we use this to compute the temperature and pressure
    bytes_read = reg_read( i2c, BMP280_ADDR, BMP280REG_DIG_T1, coeficients, 24);
    if( bytes_read != 24) {
        printf("[!] Did not read the required 24 coeficient bytes.\n");
    }
}

// Write bytes to a specified register
void reg_write( i2c_inst_t *i2c,
        const uint addr,
        const uint8_t reg,
        uint8_t *buf,
        const uint8_t nbytes) {

    uint8_t msg[nbytes + 1];

    // Only respond to requests to write one or more bytes
    if( nbytes < 1) {
        return;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for( int i=0; i<nbytes; i++) {
        msg[i+1] = buf[i];
    }

    // Write data to register(s) over I2C
    i2c_write_blocking( i2c, addr, msg, (nbytes+1), false);
}

// Read bytes from a specified register
int reg_read( i2c_inst_t *i2c,
        const uint addr,
        const uint8_t reg,
        uint8_t *buf,
        const uint8_t nbytes) {

    int num_bytes_read = 0;

    // Only respond to requests to read one or more bytes
    if( nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking( i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking( i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}
