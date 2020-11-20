#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "include/ssd1306.h"

#define mutex_lock(x)			while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) 		xSemaphoreGive(x)
#define mutex_create()			xSemaphoreCreateMutex()
#define mutex_destroy(x) 		vQueueDelete(x)

static const char *TAG = "SSD1306";

#define SSD1306_CHECK(a, str, action) if(!(a)) {							\
	STM_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);	\
	action;																	\
}