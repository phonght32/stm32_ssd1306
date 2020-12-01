#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "include/ssd1306.h"

#define SSD1306_INIT_ERR_STR				"ssd1306 init error"
#define SSD1306_WRITE_ERR_STR				"ssd1306 write error"
#define SSD1306_CLEAR_ERR_STR				"ssd1306 clear error"
#define SSD1306_FILL_ERR_STR				"ssd1306 fill error"
#define SSD1306_UPDATE_ERR_STR 				"ssd1306 update screen error"
#define SSD1306_WRITE_PIXEL_ERR_STR			"ssd1306 write pixel error"
#define SSD1306_WRITE_CHAR_ERR_STR			"ssd1306 write char error"
#define SSD1306_WRITE_STR_ERR_STR			"ssd1306 write string error"
#define SSD1306_DRAW_LINE_ERR_STR			"ssd1306 draw line error"
#define SSD1306_DRAW_POLYLINE_ERR_STR  		"ssd1306 draw polyline error"
#define SSD1306_DRAW_ARC_ERR_STR  			"ssd1306 draw arc error"
#define SSD1306_DRAW_CIRCLE_ERR_STR  		"ssd1306 draw circle error"
#define SSD1306_DRAW_REC_ERR_STR  			"ssd1306 draw rectangle error"
#define SSD1306_SET_POSITION_ERR_STR		"ssd1306 set position error"
#define SSD1306_GET_POSITION_ERR_STR 		"ssd1306 get position error"

#define TIMEOUT_MS 							100

#define SSD1306_ADDR						(0x3C<<1)
#define SSD1306_REG_DATA_ADDR				0x40
#define SSD1306_REG_CMD_ADDR				0x00
/*!<  */
#define SSD1306_SET_CONTRAST				0x81 		/*!< 0x81 + 0x00~0xFF Contrast ... reset = 0x7F */
#define SSD1306_DISPLAYALLON_RESUME 		0xA4		/*!< Resume to RAM content display */
#define SSD1306_DISPLAYALLON_IGNORE 		0xA5		/*!< Ignore RAM content display */
#define SSD1306_DISPLAY_NORMAL 				0xA6		/*!< White: 1; Black: 0 */
#define SSD1306_DISPLAY_INVERSE 			0xA7		/*!< White: 0; Black: 1 */
#define SSD1306_DISPLAY_OFF 				0xAE		/*!< Screen OFF */
#define SSD1306_DISPLAY_ON 					0xAF		/*!< Screen ON */

#define SSD1306_SET_MEMORYMODE 				0x20 		/*!< 0x20 + 0x00: horizontal; 0x01: vertical; 0x02: page */
#define SSD1306_SET_MEMORYMODE_HOR 			0x00
#define SSD1306_SET_MEMORYMODE_VER 			0x01
#define SSD1306_SET_MEMORYMODE_PAGE 		0x02

#define SSD1306_SET_COLUMN_ADDR 			0x21  		/*!< 0x21 + 0~127 + 0~127: colum start address + column end address */
#define SSD1306_SET_PAGE_ADDR 				0x22		/*!< 0x22 + 0~7 + 0~7: page start address + page end address */

#define SSD1306_SET_STARTLINE_ZERO 			0x40
#define SSD1306_SET_SEGREMAP_NORMAL  		0xA0
#define SSD1306_SET_SEGREMAP_INV 			0xA1
#define SSD1306_SET_MULTIPLEX 				0XA8
#define SSD1306_COMSCAN_INC 				0xC0
#define SSD1306_COMSCAN_DEC 				0xC8
#define SSD1306_SET_DISPLAYOFFSET 			0xD3
#define SSD1306_SET_COMPINS 				0xDA

#define SSD1306_SET_CLKDIV 					0xD5
#define SSD1306_SET_PRECHARGE 				0xD9
#define SSD1306_SET_COMDESELECT 			0xDB
#define SSD1306_NOP 						0xE3

#define SSD1306_CHARGEPUMP 					0x8D
#define SSD1306_CHARGEPUMP_ON 				0x14
#define SSD1306_CHARGEPUMP_OFF 				0x10

#define mutex_lock(x)			while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) 		xSemaphoreGive(x)
#define mutex_create()			xSemaphoreCreateMutex()
#define mutex_destroy(x) 		vQueueDelete(x)

static const char *TAG = "SSD1306";

#define SSD1306_CHECK(a, str, action) if(!(a)) {							\
	STM_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);	\
	action;																	\
}

typedef stm_err_t (*init_func)(ssd1306_hw_info_t hw_info);
typedef stm_err_t (*write_cmd_func)(ssd1306_hw_info_t hw_info, uint8_t cmd);
typedef stm_err_t (*write_data_func)(ssd1306_hw_info_t hw_info, uint8_t *data, uint16_t len);

typedef struct ssd1306 {
	ssd1306_hw_info_t		hw_info;
	ssd1306_size_t 			size;
	ssd1306_comm_mode_t 	comm_mode;
	uint8_t					height;
	uint8_t					width;
	write_data_func			_write_data;
	write_cmd_func 			_write_cmd;
	uint8_t  				*buf_display;
	uint8_t 				cur_x;
	uint8_t					cur_y;
	bool 					inverse;
	SemaphoreHandle_t		lock;
} ssd1306_t;

static stm_err_t _init_i2c(ssd1306_hw_info_t hw_info)
{
	i2c_cfg_t i2c_cfg = {
		.i2c_num = hw_info.i2c_num,
		.i2c_pins_pack = hw_info.i2c_pins_pack,
		.clk_speed = hw_info.i2c_speed,
	};
	SSD1306_CHECK(!i2c_config(&i2c_cfg), SSD1306_INIT_ERR_STR, return STM_FAIL);

	return STM_OK;
}

static stm_err_t _init_spi(ssd1306_hw_info_t hw_info)
{
	spi_cfg_t spi_cfg = {
		.spi_num = hw_info.spi_num,
		.spi_pins_pack = hw_info.spi_pins_pack,
		.mode = SPI_MODE_MASTER_HALF_DUPLEX,
		.cap_edge = SPI_CAP_FALLING_EDGE,
		.firstbit = SPI_TRANS_FIRSTBIT_MSB
	};
	SSD1306_CHECK(!spi_config(&spi_cfg), SSD1306_INIT_ERR_STR, return STM_FAIL);

	return STM_OK;
}

static stm_err_t _i2c_write_data(ssd1306_hw_info_t hw_info, uint8_t *data, uint16_t len)
{
	uint8_t buf_send[len + 1];
	buf_send[0] = SSD1306_REG_DATA_ADDR;
	for (uint8_t i = 0; i < len; i++)
	{
		buf_send[i + 1] = data[i];
	}

	SSD1306_CHECK(!i2c_master_write_bytes(hw_info.i2c_num, SSD1306_ADDR, buf_send, len + 1, TIMEOUT_MS), SSD1306_WRITE_ERR_STR, return STM_FAIL);
	return STM_OK;
}

static stm_err_t _spi_write_data(ssd1306_hw_info_t hw_info, uint8_t *data, uint16_t len)
{
	return STM_OK;
}

static stm_err_t _i2c_write_cmd(ssd1306_hw_info_t hw_info, uint8_t cmd)
{
	uint8_t buf_send[2];
	buf_send[0] = SSD1306_REG_CMD_ADDR;
	buf_send[1] = cmd;

	SSD1306_CHECK(!i2c_master_write_bytes(hw_info.i2c_num, SSD1306_ADDR, buf_send, 2, TIMEOUT_MS), SSD1306_WRITE_ERR_STR, return STM_FAIL);
	return STM_OK;
}

static stm_err_t _spi_write_cmd(ssd1306_hw_info_t hw_info, uint8_t cmd)
{
	return STM_OK;
}

static init_func _get_init_func(ssd1306_comm_mode_t comm_mode)
{
	if (comm_mode == SSD1306_COMM_MODE_I2C) {
		return _init_i2c;
	} else {
		return _init_spi;
	}

	return NULL;
}

static write_data_func _get_write_data_func(ssd1306_comm_mode_t comm_mode)
{
	if (comm_mode == SSD1306_COMM_MODE_I2C) {
		return _i2c_write_data;
	} else {
		return _spi_write_data;
	}

	return NULL;
}

static write_cmd_func _get_write_cmd_func(ssd1306_comm_mode_t comm_mode)
{
	if (comm_mode == SSD1306_COMM_MODE_I2C) {
		return _i2c_write_cmd;
	} else {
		return _spi_write_cmd;
	}

	return NULL;
}

static uint8_t _get_screen_width(ssd1306_size_t size) {
	if (size == SSD1306_SIZE_128_32) {
		return 128;
	} else {
		return 128;
	}
}

static uint8_t _get_screen_height(ssd1306_size_t size) {
	if (size == SSD1306_SIZE_128_32) {
		return 32;
	} else {
		return 64;
	}
}

static stm_err_t _update_screen(ssd1306_handle_t handle, uint8_t *buf_screen)
{
	for (uint8_t i = 0; i < (handle->height / 8); i++) {
		SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0xB0 + i), SSD1306_UPDATE_ERR_STR, return STM_FAIL);
		SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x00), SSD1306_UPDATE_ERR_STR, return STM_FAIL);
		SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x10), SSD1306_UPDATE_ERR_STR, return STM_FAIL);
		SSD1306_CHECK(!handle->_write_data(handle->hw_info, &buf_screen[i * handle->width], handle->width), SSD1306_UPDATE_ERR_STR, return STM_FAIL);
	}

	return STM_OK;
}

static void _ssd1306_cleanup(ssd1306_handle_t handle)
{
	free(handle->buf_display);
	free(handle);
}

ssd1306_handle_t ssd1306_init(ssd1306_cfg_t *config)
{
	SSD1306_CHECK(config, SSD1306_INIT_ERR_STR, return NULL);

	int ret;
	ssd1306_handle_t handle;
	uint8_t width = _get_screen_width(config->size);
	uint8_t height = _get_screen_height(config->size);

	ret = ( (handle = calloc(1, sizeof(ssd1306_t))) &&
	        (handle->buf_display = calloc(width * height / 8, sizeof(uint8_t))));
	if (!ret) {
		STM_LOGE(TAG, SSD1306_INIT_ERR_STR);
		_ssd1306_cleanup(handle);
		return NULL;
	}

	if (!config->hw_info.is_init) {
		init_func _init = _get_init_func(config->comm_mode);
		SSD1306_CHECK(!_init(config->hw_info), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	}

	write_cmd_func _write_cmd = _get_write_cmd_func(config->comm_mode);

	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_DISPLAY_OFF), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_MEMORYMODE), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_MEMORYMODE_HOR), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	// SSD1306_CHECK(!_write_cmd(config->hw_info, 0xB0), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_COMSCAN_DEC), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x00), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x10), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_STARTLINE_ZERO), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_SEGREMAP_INV), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, config->inverse == false ? SSD1306_DISPLAY_NORMAL : SSD1306_DISPLAY_INVERSE), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xFF), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x3F), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_DISPLAYALLON_RESUME), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_DISPLAYOFFSET), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x00), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_CLKDIV), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xF0), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_PRECHARGE), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x22), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_COMPINS), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x12), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_SET_COMDESELECT), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x20), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_CHARGEPUMP), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_CHARGEPUMP_ON), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, SSD1306_DISPLAY_ON), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});

	handle->hw_info = config->hw_info;
	handle->size = config->size;
	handle->comm_mode = config->comm_mode;
	handle->width = width;
	handle->height = height;
	handle->_write_cmd = _get_write_cmd_func(config->comm_mode);
	handle->_write_data = _get_write_data_func(config->comm_mode);
	handle->cur_x = 0;
	handle->cur_x = 0;
	handle->inverse = config->inverse;
	handle->lock = mutex_create();

	return handle;
}

stm_err_t ssd1306_clear(ssd1306_handle_t handle)
{
	SSD1306_CHECK(handle, SSD1306_CLEAR_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	uint32_t buf_screen_size = handle->width * handle->height / 8;
	uint8_t buf_screen[buf_screen_size];

	for (uint32_t i = 0; i < (handle->width * handle->height / 8); i++) {
		buf_screen[i] = 0x00;
	}

	SSD1306_CHECK(!_update_screen(handle, buf_screen), SSD1306_CLEAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	memcpy(handle->buf_display, buf_screen, buf_screen_size);
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t ssd1306_fill(ssd1306_handle_t handle, ssd1306_color_t color)
{
	SSD1306_CHECK(handle, SSD1306_CLEAR_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	uint32_t buf_screen_size = handle->width * handle->height / 8;
	uint8_t buf_screen[buf_screen_size];

	for (uint32_t i = 0; i < (handle->width * handle->height / 8); i++) {
		buf_screen[i] = handle->inverse == false ?
		                ((color == SSD1306_COLOR_WHITE) ? 0xFF : 0x00) :
		                ((color == SSD1306_COLOR_WHITE) ? 0x00 : 0xFF);
	}

	SSD1306_CHECK(!_update_screen(handle, buf_screen), SSD1306_CLEAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	memcpy(handle->buf_display, buf_screen, buf_screen_size);
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t ssd1306_write_pixel(ssd1306_handle_t handle, uint8_t x, uint8_t y, ssd1306_color_t color)
{
	SSD1306_CHECK(handle, SSD1306_WRITE_PIXEL_ERR_STR, return STM_ERR_INVALID_ARG);
	SSD1306_CHECK(x < handle->width, SSD1306_WRITE_PIXEL_ERR_STR, return STM_ERR_INVALID_ARG);
	SSD1306_CHECK(y < handle->height, SSD1306_WRITE_PIXEL_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	uint32_t buf_screen_size = handle->width * handle->height / 8;
	uint8_t buf_screen[buf_screen_size];
	memcpy(buf_screen, handle->buf_display, buf_screen_size);

	if (handle->inverse) {
		if (color == SSD1306_COLOR_WHITE) {
			buf_screen[x + (y / 8)*handle->width] &= ~ (1 << (y % 8));
		} else {
			buf_screen[x + (y / 8)*handle->width] |= (1 << (y % 8));
		}
	} else {
		if (color == SSD1306_COLOR_WHITE) {
			buf_screen[x + (y / 8)*handle->width] |= (1 << (y % 8));
		} else {
			buf_screen[x + (y / 8)*handle->width] &= ~ (1 << (y % 8));
		}
	}

	SSD1306_CHECK(!_update_screen(handle, buf_screen), SSD1306_WRITE_PIXEL_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	memcpy(handle->buf_display, buf_screen, buf_screen_size);

	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t ssd1306_write_char(ssd1306_handle_t handle, font_size_t font_size, uint8_t chr)
{
	SSD1306_CHECK(handle, SSD1306_WRITE_CHAR_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	font_t font;
	SSD1306_CHECK(get_font(chr, font_size, &font) > 0, SSD1306_WRITE_CHAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});

	uint32_t buf_screen_size = handle->width * handle->height / 8;
	uint8_t buf_screen[buf_screen_size];
	memcpy(buf_screen, handle->buf_display, buf_screen_size);

	uint8_t num_byte_per_row = font.data_len / font.height;

	for (uint8_t height_idx = 0; height_idx < font.height; height_idx ++) {
		for ( uint8_t byte_idx = 0; byte_idx < num_byte_per_row; byte_idx++) {
			for (uint8_t width_idx = 0; width_idx < 8; width_idx++) {
				uint8_t x = handle->cur_x + width_idx + byte_idx * 8;
				uint8_t y = handle->cur_y + height_idx;

				if (((font.data[height_idx * num_byte_per_row + byte_idx] << width_idx) & 0x80) == 0x80) {
					buf_screen[x + (y / 8)*handle->width] |= (1 << (y % 8));
				} else {
					buf_screen[x + (y / 8)*handle->width] &= ~ (1 << (y % 8));
				}
			}
		}
	}

	SSD1306_CHECK(!_update_screen(handle, buf_screen), SSD1306_WRITE_CHAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	memcpy(handle->buf_display, buf_screen, buf_screen_size);
	handle->cur_x += font.width + num_byte_per_row;
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t ssd1306_write_string(ssd1306_handle_t handle, font_size_t font_size, uint8_t *str)
{
	SSD1306_CHECK(handle, SSD1306_WRITE_STR_ERR_STR, return STM_ERR_INVALID_ARG);
	SSD1306_CHECK(str, SSD1306_WRITE_STR_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	uint32_t buf_screen_size = handle->width * handle->height / 8;
	uint8_t buf_screen[buf_screen_size];
	memcpy(buf_screen, handle->buf_display, buf_screen_size);

	uint8_t cur_x = handle->cur_x;
	uint8_t cur_y = handle->cur_y;

	while (*str) {
		font_t font;
		SSD1306_CHECK(get_font(*str, font_size, &font) > 0, SSD1306_WRITE_STR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});

		uint8_t num_byte_per_row = font.data_len / font.height;

		for (uint8_t height_idx = 0; height_idx < font.height; height_idx ++) {
			for ( uint8_t byte_idx = 0; byte_idx < num_byte_per_row; byte_idx++) {
				for (uint8_t width_idx = 0; width_idx < 8; width_idx++) {
					uint8_t x = cur_x + width_idx + byte_idx * 8;
					uint8_t y = cur_y + height_idx;

					if (((font.data[height_idx * num_byte_per_row + byte_idx] << width_idx) & 0x80) == 0x80) {
						buf_screen[x + (y / 8)*handle->width] |= (1 << (y % 8));
					} else {
						buf_screen[x + (y / 8)*handle->width] &= ~ (1 << (y % 8));
					}
				}
			}
		}
		cur_x += font.width + num_byte_per_row;
		str++;
	}

	SSD1306_CHECK(!_update_screen(handle, buf_screen), SSD1306_WRITE_STR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	memcpy(handle->buf_display, buf_screen, buf_screen_size);
	handle->cur_x = cur_x;
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t ssd1306_set_position(ssd1306_handle_t handle, uint8_t x, uint8_t y)
{
	SSD1306_CHECK(handle, SSD1306_SET_POSITION_ERR_STR, return STM_ERR_INVALID_ARG);
	SSD1306_CHECK(x < handle->width, SSD1306_SET_POSITION_ERR_STR, return STM_ERR_INVALID_ARG);
	SSD1306_CHECK(y < handle->height, SSD1306_SET_POSITION_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);
	handle->cur_x = x;
	handle->cur_y = y;
	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t ssd1306_get_position(ssd1306_handle_t handle, uint8_t *x, uint8_t *y)
{
	SSD1306_CHECK(handle, SSD1306_GET_POSITION_ERR_STR, return STM_ERR_INVALID_ARG);
	SSD1306_CHECK(x, SSD1306_GET_POSITION_ERR_STR, return STM_ERR_INVALID_ARG);
	SSD1306_CHECK(y, SSD1306_GET_POSITION_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);
	*x = handle->cur_x;
	*y = handle->cur_y;
	mutex_unlock(handle->lock);

	return STM_OK;
}

void ssd1306_destroy(ssd1306_handle_t handle)
{
	_ssd1306_cleanup(handle);
}

