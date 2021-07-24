#include <stdio.h>
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

#define I2C0_SDA_PORT 4		/* Serial data at physical pin 6               */
			 	/* -- connect to SD1 of BMP280 board           */

#define I2C0_SCL_PORT 5		/* Clock at physical pin 7                     */
				/* -- connect to SCK of BMP280 board           */

#define LED_PIN 25		/* Onboard LED at physical pin 25              */
/*******************************************************************************/

/* ADDRESS OF BMP280 ***********************************************************/
/* when SD0 is not connected this Adafruit BMP280 module appears on address $77*/

static const uint8_t BMP280_ADDR = 0x77;
static const uint8_t BMP280_RESET_VAL = 0xB6;

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
static const uint8_t REG_DIG_T1    = 0x88; 	/* ID register address         */
static const uint8_t REG_ID        = 0xD0; 	/* ID register address         */
static const uint8_t REG_TEMP      = 0xFA; 	/* Temerature register address */
static const uint8_t REG_RESET     = 0xE0; 	/* Soft reset address          */
static const uint8_t REG_STATUS    = 0xF3; 	/* Status register address     */
static const uint8_t REG_CTRL_MEAS = 0xF4; 	/* Data aquisition ctrl address*/
static const uint8_t REG_CONFIG    = 0xF5; 	/* Config register address     */

/* FUNCTION HEADERS ************************************************************/
void reg_write(	  i2c_inst_t *i2c,
		  const uint addr,
		  const uint8_t reg,
		  uint8_t *buf,
		  const uint8_t nbytes );

int reg_read( 	  i2c_inst_t *i2c,
		  const uint addr,
		  const uint8_t reg,
		  uint8_t *buf,
		  const uint8_t nbytes );

void init_BMP280( i2c_inst_t *i2c );

double get_temperature( int adc_t );

/* GLOBAL VAR TO HOLD THE COEFICIENTS ******************************************/
uint8_t coeficients[24];

int main(void){

    // Variable to hold the temperature
    int temp_i = 0;

    // Ports
    i2c_inst_t *i2c = i2c0;

    // Allow this programto print to serial
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

    // Vars to hold the temperature
    int adc_T = 0;
    double temperature = 0.0;

    // Initialize BMP280 Module
    init_BMP280( i2c);

    while(1){

        // Get temperature from module
        n = reg_read( i2c, BMP280_ADDR, REG_TEMP, data, 3);

        adc_T = data[0];
        adc_T <<= 8;
        adc_T |= data[1];
        adc_T <<= 4;
        adc_T |= (data[3] >> 4);

        // Get temperature from raw value and stored coeficients
        temperature = get_temperature( adc_T);

        printf("[-] %3.1f°C\n", temperature);

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

// Calculate temperature from raw value and coeficients
double get_temperature( int adc_t) {
    double t = 0.0;
    double var1 = 0;
    double var2 = 0;

    var1 = (adc_t/16384.0 - ((int16_t)((coeficients[1] << 8) | coeficients[0]))/1024.0) * ((int16_t)((coeficients[3] << 8) | coeficients[2]));
    var2 = (adc_t/131072.0 - ((int16_t)((coeficients[1] << 8) | coeficients[0]))/8192.0) * (adc_t/131072.0 - ((int16_t)((coeficients[1] << 8) | coeficients[0]))/8192.0) * ((int16_t)((coeficients[5] << 8) | coeficients[4]));

    t = (var1 + var2) / 5120.0;

    return t;
}

// Init BMP280 module
void init_BMP280( i2c_inst_t *i2c){

    // Prepare chipID var, which is the size of one byte
    uint8_t data[1];

    // initialize bytes_read to zero
    int bytes_read = 0;

    // Give it some time to wakeup
    sleep_ms(1000);

    // Read 1-byte from REG_ID and store the result in chipId
    bytes_read = reg_read( i2c, BMP280_ADDR, REG_ID, data, 1);
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
    reg_write( i2c, BMP280_ADDR, REG_RESET, &data[0], 1);
    sleep_ms(4);

    // Write data aquisition control parameters for temperature and pressure

    data[0] = OVERSCAN_X2;	// SET Overscan for Temperature
    data[0] <<= 3;
    data[0] |= OVERSCAN_X16;	// SET Overscan for Pressure
    data[0] <<= 2;
    data[0] |= MODE_SLEEP;	// SET Mode to SLEEP

    // printf("[i] Data aquisition control set to: %X (SLEEP)\n", data[0]); 

    reg_write( i2c, BMP280_ADDR, REG_CTRL_MEAS, &data[0], 1);

    data[0] = IIR_FILTER_DISABLE;
    data[0] <<= 2;

    // printf("[i] Config value set to: %X\n", data[0]); 

    reg_write( i2c, BMP280_ADDR, REG_CONFIG, &data[0], 1);

    // Write data aquisition control parameters for temperature and pressure

    data[0] = OVERSCAN_X2;	// SET Overscan for Temperature
    data[0] <<=3;
    data[0] |= OVERSCAN_X16;	// SET Overscan for Pressure
    data[0] <<= 2;
    data[0] |= MODE_NORMAL;	// SET Mode to NORMAL

    // printf("[i] Data aquisition control set to: %X (NORMAL)\n", data[0]); 

    reg_write( i2c, BMP280_ADDR, REG_CTRL_MEAS, &data[0], 1);

    // Wait until the NORMAL operation mode has settled
    do {
        sleep_ms(2);
        // Get status and wait until bit-4 is set
        bytes_read = reg_read( i2c, BMP280_ADDR, REG_STATUS, data, 1);

    } while( data[0] & 0x08);

    // Read coeficients from device, we use this to compute the temperature and pressure
    bytes_read = reg_read( i2c, BMP280_ADDR, REG_DIG_T1, coeficients, 24);
    if( bytes_read != 24) {
        printf("[!] Did not read the required 24 coeficient bytes.\n");
    }
}

// Write bytes to a specified register
void reg_write(	i2c_inst_t *i2c,
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
int reg_read(	i2c_inst_t *i2c,
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
