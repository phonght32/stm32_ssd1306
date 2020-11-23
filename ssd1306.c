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
#define SSD1306_GOTYXY_ERR_STR				"ssd1306 gotoxy error"

#define TIMEOUT_MS 							100

#define SSD1306_ADDR						(0x3C<<1)
#define SSD1306_REG_DATA_ADDR				0x40
#define SSD1306_REG_CMD_ADDR				0x00

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
	uint8_t					height;
	uint8_t					width;
	ssd1306_comm_mode_t 	comm_mode;
	write_data_func			_write_data;
	write_cmd_func 			_write_cmd;
	uint8_t  				*buf_display;
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

static stm_err_t _update_screen(ssd1306_handle_t handle)
{
	for (uint8_t i = 0; i < (handle->height / 8); i++) {
		SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0xB0 + i), SSD1306_UPDATE_ERR_STR, return STM_FAIL);
		SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x00), SSD1306_UPDATE_ERR_STR, return STM_FAIL);
		SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x10), SSD1306_UPDATE_ERR_STR, return STM_FAIL);
		SSD1306_CHECK(!handle->_write_data(handle->hw_info, &handle->buf_display[i * handle->width], handle->width), SSD1306_UPDATE_ERR_STR, return STM_FAIL);
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

	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xAE), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x20), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x00), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xB0), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xC8), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x00), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x10), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x40), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xA1), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xA6), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xFF), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x3F), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xA4), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xD3), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x00), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xD5), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xF0), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xD9), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x22), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xDA), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x12), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xDB), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x20), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x8D), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0x14), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});
	SSD1306_CHECK(!_write_cmd(config->hw_info, 0xAF), SSD1306_INIT_ERR_STR, {_ssd1306_cleanup(handle); return NULL;});

	handle->hw_info = config->hw_info;
	handle->size = config->size;
	handle->width = width;
	handle->height = height;
	handle->comm_mode = config->comm_mode;
	handle->_write_cmd = _get_write_cmd_func(config->comm_mode);
	handle->_write_data = _get_write_data_func(config->comm_mode);
	handle->lock = mutex_create();

	return handle;
}
uint8_t buf[128 * 64 / 8];
stm_err_t ssd1306_clear(ssd1306_handle_t handle)
{
	SSD1306_CHECK(handle, SSD1306_CLEAR_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	for (uint32_t i = 0; i < (handle->width * handle->height / 8); i++) {
		handle->buf_display[i] = 0x00;
	}

	SSD1306_CHECK(!_update_screen(handle), SSD1306_CLEAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});

	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x21), SSD1306_CLEAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x00), SSD1306_CLEAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x7F), SSD1306_CLEAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x22), SSD1306_CLEAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x00), SSD1306_CLEAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x07), SSD1306_CLEAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});

	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t ssd1306_fill(ssd1306_handle_t handle, ssd1306_color_t color)
{
	SSD1306_CHECK(handle, SSD1306_CLEAR_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	for (uint32_t i = 0; i < (handle->width * handle->height / 8); i++) {
		handle->buf_display[i] = (color == SSD1306_COLOR_WHITE) ? 0xFF : 0x00;
	}

	SSD1306_CHECK(!_update_screen(handle), SSD1306_CLEAR_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});

	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t ssd1306_write_pixel(ssd1306_handle_t handle, uint8_t x, uint8_t y, ssd1306_color_t color)
{
	SSD1306_CHECK(handle, SSD1306_WRITE_PIXEL_ERR_STR, return STM_ERR_INVALID_ARG);
	SSD1306_CHECK(x < handle->width, SSD1306_WRITE_PIXEL_ERR_STR, return STM_ERR_INVALID_ARG);
	SSD1306_CHECK(y < handle->height, SSD1306_WRITE_PIXEL_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	if (color == SSD1306_COLOR_WHITE) {
		handle->buf_display[x + (y / 8)*handle->width] |= 1 << (y % 8);
	} else {
		handle->buf_display[x + (y / 8)*handle->width] &= ~ 1 << (y % 8);
	}

	SSD1306_CHECK(!_update_screen(handle), SSD1306_WRITE_PIXEL_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});

	mutex_unlock(handle->lock);

	return STM_OK;
}

stm_err_t ssd1306_write_char(ssd1306_handle_t handle, uint8_t chr)
{
	SSD1306_CHECK(handle, SSD1306_WRITE_PIXEL_ERR_STR, return STM_ERR_INVALID_ARG);


	return STM_OK;
}

stm_err_t ssd1306_gotoxy(ssd1306_handle_t handle, uint8_t x, uint8_t y)
{
	SSD1306_CHECK(handle, SSD1306_GOTYXY_ERR_STR, return STM_ERR_INVALID_ARG);
	SSD1306_CHECK(x < handle->width, SSD1306_GOTYXY_ERR_STR, return STM_ERR_INVALID_ARG);
	SSD1306_CHECK(y < handle->height, SSD1306_GOTYXY_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);
	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x21), SSD1306_GOTYXY_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info,    x), SSD1306_GOTYXY_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x7F), SSD1306_GOTYXY_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x22), SSD1306_GOTYXY_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info,    y), SSD1306_GOTYXY_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	SSD1306_CHECK(!handle->_write_cmd(handle->hw_info, 0x07), SSD1306_GOTYXY_ERR_STR, {mutex_unlock(handle->lock); return STM_FAIL;});
	mutex_unlock(handle->lock);

	return STM_OK;
}

