#ifndef BIGVAL_HEADER
#define BIGVAL_HEADER

/******************************************************************************/
/* bigval.h
/* header file for big decimal values on a 128x32 pixel SSD1306 display
/* Author: Toon Leijtens, toon@ybzconsulting.com
/* Date: 2021.08.01.07:22PM
/* Version: 1.0
/* License: MIT
/*
/* To integrate in your project, simply provide an instance to the i2c bus and
/* the address of the SSD1306 device, the 128x32 version is on 0x3C, then
/* initialize a linear buffer with uint8_t type elements. You need to have
/* 512 elements (bytes) in total (=128*32/8).
/******************************************************************************/

void write_bigval( i2c_inst_t *i2c, const uint addr, char *buffer, char *value,
			uint8_t point_pos);

void bigval_a(char *buffer, uint8_t z, bool on);
void bigval_b(char *buffer, uint8_t z, bool on);
void bigval_c(char *buffer, uint8_t z, bool on);
void bigval_d(char *buffer, uint8_t z, bool on);
void bigval_e(char *buffer, uint8_t z, bool on);
void bigval_f(char *buffer, uint8_t z, bool on);
void bigval_g(char *buffer, uint8_t z, bool on);
void bigval_p(char *buffer, uint8_t z, bool on);

#endif