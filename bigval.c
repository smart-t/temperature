#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "temperature.h"
#include "bigval.h"

void write_bigval( i2c_inst_t *i2c,
			uint addr,
			char *buffer,
			char *value,
			uint8_t point_pos){

	int	value_len = sizeof(value);
	uint8_t d[5]; // here we store the uint8_t values of the digits
    uint8_t data[3]; // used to write command to display
	uint8_t j; // used as an index for the digits array

	// Test type and length of value and prev value. Should be max 5 digits [0..9]
	if( value_len > 5) return;

	// Initialize j to 0
	j = 0;

	// Capture digits as uint8_t value
    for( int i=0; i <= value_len; i++){

		if(value[i] != '.'){
			d[j] = value[i] - 48;
			j++;
		}
	}

	// Test to see if the decimal point location is sound [0,1,2,3 or 4] exit when not. 0=off
	if( point_pos > 4) return;

	switch( point_pos){
		case 1:	bigval_p( buffer, 0, false);
				bigval_p( buffer, 1, true);
				bigval_p( buffer, 2, false);
				bigval_p( buffer, 3, false);
				bigval_p( buffer, 4, false);
				break;
		case 2:	bigval_p( buffer, 0, false);
				bigval_p( buffer, 1, false);
				bigval_p( buffer, 2, true);
				bigval_p( buffer, 3, false);
				bigval_p( buffer, 4, false);
				break;
		case 3:	bigval_p( buffer, 0, false);
				bigval_p( buffer, 1, false);
				bigval_p( buffer, 2, false);
				bigval_p( buffer, 3, true);
				bigval_p( buffer, 4, false);
				break;
		case 4:	bigval_p( buffer, 0, false);
				bigval_p( buffer, 1, false);
				bigval_p( buffer, 2, false);
				bigval_p( buffer, 3, false);
				bigval_p( buffer, 4, true);
				break;
		default:bigval_p( buffer, 0, false);
				bigval_p( buffer, 1, false);
				bigval_p( buffer, 2, false);
				bigval_p( buffer, 3, false);
				bigval_p( buffer, 4, false);
				break;
	}

	for( int i=0; i<=4; i++){
		// Print first digit
		switch( d[i]){
			case 0:	bigval_a( buffer, i, true);
					bigval_b( buffer, i, true);
					bigval_c( buffer, i, true);
					bigval_d( buffer, i, true);
					bigval_e( buffer, i, true);
					bigval_f( buffer, i, true);
					bigval_g( buffer, i, false);
					break;
			case 1:	bigval_a( buffer, i, false);
					bigval_b( buffer, i, true);
					bigval_c( buffer, i, true);
					bigval_d( buffer, i, false);
					bigval_e( buffer, i, false);
					bigval_f( buffer, i, false);
					bigval_g( buffer, i, false);
					break;
			case 2:	bigval_a( buffer, i, true);
					bigval_b( buffer, i, true);
					bigval_c( buffer, i, false);
					bigval_d( buffer, i, true);
					bigval_e( buffer, i, true);
					bigval_f( buffer, i, false);
					bigval_g( buffer, i, true);
					break;
			case 3:	bigval_a( buffer, i, true);
					bigval_b( buffer, i, true);
					bigval_c( buffer, i, true);
					bigval_d( buffer, i, true);
					bigval_e( buffer, i, false);
					bigval_f( buffer, i, false);
					bigval_g( buffer, i, true);
					break;
			case 4:	bigval_a( buffer, i, false);
					bigval_b( buffer, i, true);
					bigval_c( buffer, i, true);
					bigval_d( buffer, i, false);
					bigval_e( buffer, i, false);
					bigval_f( buffer, i, true);
					bigval_g( buffer, i, true);
					break;
			case 5:	bigval_a( buffer, i, true);
					bigval_b( buffer, i, false);
					bigval_c( buffer, i, true);
					bigval_d( buffer, i, true);
					bigval_e( buffer, i, false);
					bigval_f( buffer, i, true);
					bigval_g( buffer, i, true);
					break;
			case 6:	bigval_a( buffer, i, true);
					bigval_b( buffer, i, false);
					bigval_c( buffer, i, true);
					bigval_d( buffer, i, true);
					bigval_e( buffer, i, true);
					bigval_f( buffer, i, true);
					bigval_g( buffer, i, true);
					break;
			case 7:	bigval_a( buffer, i, true);
					bigval_b( buffer, i, true);
					bigval_c( buffer, i, true);
					bigval_d( buffer, i, false);
					bigval_e( buffer, i, false);
					bigval_f( buffer, i, false);
					bigval_g( buffer, i, false);
					break;
			case 8:	bigval_a( buffer, i, true);
					bigval_b( buffer, i, true);
					bigval_c( buffer, i, true);
					bigval_d( buffer, i, true);
					bigval_e( buffer, i, true);
					bigval_f( buffer, i, true);
					bigval_g( buffer, i, true);
					break;
			case 9:	bigval_a( buffer, i, true);
					bigval_b( buffer, i, true);
					bigval_c( buffer, i, true);
					bigval_d( buffer, i, true);
					bigval_e( buffer, i, false);
					bigval_f( buffer, i, true);
					bigval_g( buffer, i, true);
					break;
			default: break;
		}
	}

    for( int i=0; i < 512; i =i+32){
    	reg_write( i2c, SSD1306_ADDR, SSD1306REG_DISP_START_LINE, &buffer[i], 32);
	}
}

// 7-SEG Display layout with point p. There are 4 digits in the display
//
//   ***a***
//  *       *
//  f       b
//	*       *
//   ***g***
//  *       *
//  e       c
//  *       *
//   ***d***  p
//

// a-segment of digit in position pos
void bigval_a(char *buffer, uint8_t pos, bool on){
	uint8_t x = 0;
	uint8_t y = 0;
	uint8_t z = 0;

	// from pos determine z on display
	switch( pos){
		case 0: z = 4;
				break;
		case 1: z = 29;
				break;
		case 2: z = 54;
				break;
		case 3:	z = 79;
				break;
		case 4:	z = 104;
				break;
		default: return;
	}

	for( uint8_t i=1; i<=15; i++){
		x = i+z;
		y = 0;

		x &= 0x7f;
  		y &= 0x1f;
		if(on){
  			buffer[((y & 0xf8) << 4) + x] |= 1 << (y & 7);
		} else {
  			buffer[((y & 0xf8) << 4) + x] &= 0 << (y & 7);
		}
	}
}

// b-segment of digit in position pos
void bigval_b(char *buffer, uint8_t pos, bool on){
	uint8_t x = 0;
	uint8_t y = 0;
	uint8_t z = 0;

	// from pos determine z on display
	switch( pos){
		case 0: z = 4;
				break;
		case 1: z = 29;
				break;
		case 2: z = 54;
				break;
		case 3:	z = 79;
				break;
		case 4:	z = 104;
				break;
		default: return;
	}

	for( uint8_t i=2; i<=13; i++){
		x = 16+z;
		y = i;

		x &= 0x7f;
  		y &= 0x1f;
		if(on){
  			buffer[((y & 0xf8) << 4) + x] |= 1 << (y & 7);
		} else {
  			buffer[((y & 0xf8) << 4) + x] &= 0 << (y & 7);
		}
	}
}

// c-segment of digit in position pos
void bigval_c(char *buffer, uint8_t pos, bool on){
	uint8_t x = 0;
	uint8_t y = 0;
	uint8_t z = 0;

	// from pos determine z on display
	switch( pos){
		case 0: z = 4;
				break;
		case 1: z = 29;
				break;
		case 2: z = 54;
				break;
		case 3:	z = 79;
				break;
		case 4:	z = 104;
				break;
		default: return;
	}

	for( uint8_t i=17; i<=29; i++){
		x = 16+z;
		y = i;

		x &= 0x7f;
  		y &= 0x1f;
		if(on){
  			buffer[((y & 0xf8) << 4) + x] |= 1 << (y & 7);
		} else {
  			buffer[((y & 0xf8) << 4) + x] &= 0 << (y & 7);
		}
	}
}

// d-segment of digit in position pos
void bigval_d(char *buffer, uint8_t pos, bool on){
	uint8_t x = 0;
	uint8_t y = 0;
	uint8_t z = 0;

	// from pos determine z on display
	switch( pos){
		case 0: z = 4;
				break;
		case 1: z = 29;
				break;
		case 2: z = 54;
				break;
		case 3:	z = 79;
				break;
		case 4:	z = 104;
				break;
		default: return;
	}

	for( uint8_t i=1; i<=15; i++){
		x = i+z;
		y = 31;

		x &= 0x7f;
  		y &= 0x1f;
		if(on){
  			buffer[((y & 0xf8) << 4) + x] |= 1 << (y & 7);
		} else {
  			buffer[((y & 0xf8) << 4) + x] &= 0 << (y & 7);
		}
	}
}

// e-segment of digit in position pos
void bigval_e(char *buffer, uint8_t pos, bool on){
	uint8_t x = 0;
	uint8_t y = 0;
	uint8_t z = 0;

	// from pos determine z on display
	switch( pos){
		case 0: z = 4;
				break;
		case 1: z = 29;
				break;
		case 2: z = 54;
				break;
		case 3:	z = 79;
				break;
		case 4:	z = 104;
				break;
		default: return;
	}

	for( uint8_t i=17; i<=29; i++){
		x = 0+z;
		y = i;

		x &= 0x7f;
  		y &= 0x1f;
		if(on){
  			buffer[((y & 0xf8) << 4) + x] |= 1 << (y & 7);
		} else {
  			buffer[((y & 0xf8) << 4) + x] &= 0 << (y & 7);
		}
	}
}

// f-segment of digit in position pos
void bigval_f(char *buffer, uint8_t pos, bool on){
	uint8_t x = 0;
	uint8_t y = 0;
	uint8_t z = 0;

	// from pos determine z on display
	switch( pos){
		case 0: z = 4;
				break;
		case 1: z = 29;
				break;
		case 2: z = 54;
				break;
		case 3:	z = 79;
				break;
		case 4:	z = 104;
				break;
		default: return;
	}

	for( uint8_t i=2; i<=13; i++){
		x = 0+z;
		y = i;

		x &= 0x7f;
  		y &= 0x1f;
		if(on){
  			buffer[((y & 0xf8) << 4) + x] |= 1 << (y & 7);
		} else {
  			buffer[((y & 0xf8) << 4) + x] &= 0 << (y & 7);
		}
	}
}

// g-segment of digit in position pos
void bigval_g(char *buffer, uint8_t pos, bool on){
	uint8_t x = 0;
	uint8_t y = 0;
	uint8_t z = 0;

	// from pos determine z on display
	switch( pos){
		case 0: z = 4;
				break;
		case 1: z = 29;
				break;
		case 2: z = 54;
				break;
		case 3:	z = 79;
				break;
		case 4:	z = 104;
				break;
		default: return;
	}

	for( uint8_t i=1; i<=15; i++){
		x = i+z;
		y = 15;

		x &= 0x7f;
  		y &= 0x1f;
		if(on){
  			buffer[((y & 0xf8) << 4) + x] |= 1 << (y & 7);
		} else {
  			buffer[((y & 0xf8) << 4) + x] &= 0 << (y & 7);
		}
	}
}

// p-segment (point) at position pos
void bigval_p(char *buffer, uint8_t pos, bool on){
	uint8_t x = 0;
	uint8_t y = 0;
	uint8_t z = 0;

	// from pos determine z on display
	switch( pos){
		case 1: z = 4;
				break;
		case 2: z = 29;
				break;
		case 3:	z = 54;
				break;
		case 4:	z = 79;
				break;
		default: return;
	}

	for( uint8_t i=29; i<=31; i++){
		x = 20+z;
		y = i;

		x &= 0x7f;
  		y &= 0x1f;
		if(on){
  			buffer[((y & 0xf8) << 4) + x] |= 1 << (y & 7);
		} else {
  			buffer[((y & 0xf8) << 4) + x] &= 0 << (y & 7);
		}
	}

	for( uint8_t i=20; i<=22; i++){
		x = i+z;
		y = 31;

		x &= 0x7f;
  		y &= 0x1f;
		if(on){
  			buffer[((y & 0xf8) << 4) + x] |= 1 << (y & 7);
		} else {
  			buffer[((y & 0xf8) << 4) + x] &= 0 << (y & 7);
		}
	}

	for( uint8_t i=29; i<=31; i++){
		x = 22+z;
		y = i;

		x &= 0x7f;
  		y &= 0x1f;
		if(on){
  			buffer[((y & 0xf8) << 4) + x] |= 1 << (y & 7);
		} else {
  			buffer[((y & 0xf8) << 4) + x] &= 0 << (y & 7);
		}
	}

	for( uint8_t i=20; i<=22; i++){
		x = i+z;
		y = 29;

		x &= 0x7f;
  		y &= 0x1f;
		if(on){
  			buffer[((y & 0xf8) << 4) + x] |= 1 << (y & 7);
		} else {
  			buffer[((y & 0xf8) << 4) + x] &= 0 << (y & 7);
		}
	}
}
