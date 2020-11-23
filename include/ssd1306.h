// MIT License

// Copyright (c) 2020 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _SSD1306_H_
#define _SSD1306_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stm_err.h"
#include "driver/i2c.h"
#include "driver/spi.h"

typedef struct ssd1306 *ssd1306_handle_t;

typedef enum {
	SSD1306_COLOR_BLACK = 0,
	SSD1306_COLOR_WHITE,
	SSD1306_COLOR_MAX
} ssd1306_color_t;

typedef enum {
	SSD1306_SIZE_128_32 = 0,
	SSD1306_SIZE_128_64,
	SSD1306_SIZE_MAX
} ssd1306_size_t;

typedef enum {
	SSD1306_COMM_MODE_I2C = 0,
	SSD1306_COMM_MODE_SPI,
	SSD1306_COMM_MODE_MAX
} ssd1306_comm_mode_t;

typedef struct {
	i2c_num_t 				i2c_num;
	i2c_pins_pack_t 		i2c_pins_pack;
	uint32_t				i2c_speed;
	spi_num_t 				spi_num;
	spi_pins_pack_t 		spi_pins_pack;
	bool 					is_init;
} ssd1306_hw_info_t;

typedef struct {
	ssd1306_hw_info_t		hw_info;
	ssd1306_size_t 			size;
	ssd1306_comm_mode_t 	comm_mode;
} ssd1306_cfg_t;

ssd1306_handle_t ssd1306_init(ssd1306_cfg_t *config);
stm_err_t ssd1306_clear(ssd1306_handle_t handle);
stm_err_t ssd1306_fill(ssd1306_handle_t handle, ssd1306_color_t color);
stm_err_t ssd1306_write_pixel(ssd1306_handle_t handle, uint8_t x, uint8_t y, ssd1306_color_t color);
stm_err_t ssd1306_write_char(ssd1306_handle_t handle, uint8_t chr);
stm_err_t ssd1306_gotoxy(ssd1306_handle_t handle, uint8_t x, uint8_t y);


#ifdef __cplusplus
}
#endif

#endif /* _SSD1306_H_ */