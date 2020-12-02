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

#include "fonts.h"
#include "driver/i2c.h"
#include "driver/spi.h"

typedef struct ssd1306 *ssd1306_handle_t;

typedef enum {
	SSD1306_COLOR_BLACK = 0,						/*!< Color black */
	SSD1306_COLOR_WHITE,							/*!< Color white */
	SSD1306_COLOR_MAX
} ssd1306_color_t;

typedef enum {
	SSD1306_SIZE_128_32 = 0,						/*!< Screen resolution 128x32 */
	SSD1306_SIZE_128_64,							/*!< Screen resolution 128x64 */
	SSD1306_SIZE_MAX
} ssd1306_size_t;

typedef enum {
	SSD1306_COMM_MODE_I2C = 0,						/*!< Communicate over I2C */
	SSD1306_COMM_MODE_SPI,							/*!< Communicate over SPI */
	SSD1306_COMM_MODE_MAX
} ssd1306_comm_mode_t;

typedef struct {
	i2c_num_t 				i2c_num;				/*!< I2C num */
	i2c_pins_pack_t 		i2c_pins_pack;			/*!< I2C pins pack */
	uint32_t				i2c_speed;				/*!< I2C speed */
	spi_num_t 				spi_num;				/*!< SPI num */
	spi_pins_pack_t 		spi_pins_pack;			/*!< SPI pins pack */
	bool 					is_init;				/*!< Check hardware already initialized */
} ssd1306_hw_info_t;

typedef struct {
	ssd1306_hw_info_t		hw_info;				/*!< Hardware information */
	ssd1306_size_t 			size;					/*!< Screen resolution */
	ssd1306_comm_mode_t 	comm_mode;				/*!< Communicate protocol */
	bool 					inverse;				/*!< Inverse display */
} ssd1306_cfg_t;

/*
 * @brief   Initialize SSD1306 driver.
 * @param   config Struct pointer.
 * @return
 *      - SSD1306 handle structure: Success.
 *      - 0: Fail.
 */
ssd1306_handle_t ssd1306_init(ssd1306_cfg_t *config);

/*
 * @brief   Clear all display.
 * @param   handle Handle structure.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t ssd1306_clear(ssd1306_handle_t handle);

/*
 * @brief   Fill screen with color.
 * @param   handle Handle structure.
 * @param 	color Fill color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t ssd1306_fill(ssd1306_handle_t handle, ssd1306_color_t color);

/*
 * @brief   Write ASCII character.
 * @param   handle Handle structure.
 * @param 	font_size Font size.
 * @param 	chr ASCII character.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t ssd1306_write_char(ssd1306_handle_t handle, font_size_t font_size, uint8_t chr);

/*
 * @brief   Write string.
 * @param   handle Handle structure.
 * @param 	font_size Font size.
 * @param 	str Pointer to string.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t ssd1306_write_string(ssd1306_handle_t handle, font_size_t font_size, uint8_t *str);

/*
 * @brief   Draw color to pixel.
 * @param   handle Handle structure.
 * @param 	x Column index.
 * @param 	y Row index.
 * @param 	color Color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t ssd1306_draw_pixel(ssd1306_handle_t handle, uint8_t x, uint8_t y, ssd1306_color_t color);

/*
 * @brief   Draw line.
 * @param   handle Handle structure.
 * @param 	x_start x start position.
 * @param 	y_start y start position.
 * @param 	x_end x end position.
 * @param 	y_end y end position.
 * @param 	color Line color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t ssd1306_draw_line(ssd1306_handle_t handle, uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end, ssd1306_color_t color);

/*
 * @brief   Draw rectangle.
 * @param   handle Handle structure.
 * @param 	x_origin x origin position.
 * @param 	y_origin y origin position.
 * @param 	width Width.
 * @param 	height Height.
 * @param 	color Line color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t ssd1306_draw_rectangle(ssd1306_handle_t handle, uint8_t x_origin, uint8_t y_origin, uint8_t width, uint8_t height, ssd1306_color_t color);

/*
 * @brief   Draw circle.
 * @param   handle Handle structure.
 * @param 	x_origin x origin position.
 * @param 	y_origin y origin position.
 * @param 	radius Radius.
 * @param 	color Line color.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t ssd1306_draw_circle(ssd1306_handle_t handle, uint8_t x_origin, uint8_t y_origin, uint8_t radius, ssd1306_color_t color);

/*
 * @brief   Draw mono image.
 * @param   handle Handle structure.
 * @param 	x_origin x origin position.
 * @param 	y_origin y origin position.
 * @param 	width Image width in pixel.
 * @param 	height Image height in pixel.
 * @param 	image_src Image source.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t ssd1306_draw_image(ssd1306_handle_t handle, uint8_t x_origin, uint8_t y_origin, uint8_t width, uint8_t height, uint8_t *image_src);

/*
 * @brief   Goto position.
 * @param   handle Handle structure.
 * @param 	x Column index.
 * @param 	y Row index.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t ssd1306_set_position(ssd1306_handle_t handle, uint8_t x, uint8_t y);

/*
 * @brief   Get position.
 * @param   handle Handle structure.
 * @param 	x Pointer to column index.
 * @param 	y Pointer to row index.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t ssd1306_get_position(ssd1306_handle_t handle, uint8_t *x, uint8_t *y);

/*
 * @brief   Destroy SSD1306 handle structure.
 * @param   handle Handle structure.
 * @return	None.
 */
void ssd1306_destroy(ssd1306_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* _SSD1306_H_ */