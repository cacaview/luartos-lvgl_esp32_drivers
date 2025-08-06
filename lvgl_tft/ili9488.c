/**
 * @file ili9488.c
 */

/*********************
 *      INCLUDES
 *********************/
#include "ili9488.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/
 #define TAG "ILI9488"

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ili9488_set_orientation(uint8_t orientation);

static void ili9488_send_cmd(uint8_t cmd);
static void ili9488_send_data(void * data, uint16_t length);
static void ili9488_send_color(void * data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
// From github.com/jeremyjh/ESP32_TFT_library
// From github.com/mvturnho/ILI9488-lvgl-ESP32-WROVER-B
void ili9488_init(void)
{
    ESP_LOGI(TAG, "开始初始化ILI9488显示器...");
    
	lcd_init_cmd_t ili_init_cmds[]={
                {ILI9488_CMD_SLEEP_OUT, {0x00}, 0x80},
		{ILI9488_CMD_POSITIVE_GAMMA_CORRECTION, {0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F}, 15},
		{ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION, {0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F}, 15},
		{ILI9488_CMD_POWER_CONTROL_1, {0x17, 0x15}, 2},
		{ILI9488_CMD_POWER_CONTROL_2, {0x41}, 1},
		{ILI9488_CMD_VCOM_CONTROL_1, {0x00, 0x12, 0x80}, 3},
		{ILI9488_CMD_MEMORY_ACCESS_CONTROL, {0x28}, 1},  // 简化为横屏模式
		{ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET, {0x66}, 1},  // 保持18位模式
		{ILI9488_CMD_INTERFACE_MODE_CONTROL, {0x00}, 1},
		{ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL, {0xA0}, 1},
		{ILI9488_CMD_DISPLAY_INVERSION_CONTROL, {0x02}, 1},
		{ILI9488_CMD_DISPLAY_FUNCTION_CONTROL, {0x02, 0x02}, 2},
		{ILI9488_CMD_SET_IMAGE_FUNCTION, {0x00}, 1},
		{ILI9488_CMD_WRITE_CTRL_DISPLAY, {0x28}, 1},
		{ILI9488_CMD_WRITE_DISPLAY_BRIGHTNESS, {0x7F}, 1},
		{ILI9488_CMD_ADJUST_CONTROL_3, {0xA9, 0x51, 0x2C, 0x02}, 4},
		{ILI9488_CMD_DISPLAY_ON, {0x00}, 0x80},
		{0, {0}, 0xff},
	};

	//Initialize non-SPI GPIOs
    ESP_LOGI(TAG, "初始化DC引脚 (GPIO %d)...", ILI9488_DC);
    esp_rom_gpio_pad_select_gpio(ILI9488_DC);
	gpio_set_direction(ILI9488_DC, GPIO_MODE_OUTPUT);

#if ILI9488_USE_RST
    ESP_LOGI(TAG, "初始化RST引脚 (GPIO %d)...", ILI9488_RST);
    esp_rom_gpio_pad_select_gpio(ILI9488_RST);
	gpio_set_direction(ILI9488_RST, GPIO_MODE_OUTPUT);

	//Reset the display
	ESP_LOGI(TAG, "重置显示器...");
	gpio_set_level(ILI9488_RST, 0);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	gpio_set_level(ILI9488_RST, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	ESP_LOGI(TAG, "显示器重置完成");
#else
    ESP_LOGW(TAG, "RST引脚未使用 - 尝试软件重置");
    // 手动配置RST引脚进行硬件重置
    ESP_LOGI(TAG, "强制配置RST引脚 (GPIO 17)...");
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << 17);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    // 执行硬件重置
    gpio_set_level(17, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(17, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_LOGI(TAG, "强制硬件重置完成");
#endif

	ESP_LOGI(TAG, "ILI9488 initialization.");

	// Exit sleep
	ESP_LOGI(TAG, "发送软件重置命令...");
	ili9488_send_cmd(0x01);	/* Software reset */
	vTaskDelay(100 / portTICK_PERIOD_MS);

	//Send all the commands
	ESP_LOGI(TAG, "发送初始化命令序列...");
	uint16_t cmd = 0;
	while (ili_init_cmds[cmd].databytes!=0xff) {
		ESP_LOGD(TAG, "发送命令 0x%02X, 数据长度: %d", ili_init_cmds[cmd].cmd, ili_init_cmds[cmd].databytes & 0x1F);
		ili9488_send_cmd(ili_init_cmds[cmd].cmd);
		ili9488_send_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
		if (ili_init_cmds[cmd].databytes & 0x80) {
			ESP_LOGD(TAG, "等待延迟...");
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
		cmd++;
	}
	ESP_LOGI(TAG, "初始化命令序列发送完成，共发送 %d 个命令", cmd);

    ESP_LOGI(TAG, "设置显示方向...");
    ili9488_set_orientation(CONFIG_LV_DISPLAY_ORIENTATION);
    
    // 强制发送一个显示开启命令
    ESP_LOGI(TAG, "确保显示器开启...");
    ili9488_send_cmd(ILI9488_CMD_DISPLAY_ON);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 发送退出睡眠模式命令
    ESP_LOGI(TAG, "退出睡眠模式...");
    ili9488_send_cmd(ILI9488_CMD_SLEEP_OUT);
    vTaskDelay(pdMS_TO_TICKS(120)); // 至少120ms延迟
    
    ESP_LOGI(TAG, "ILI9488初始化完成");
}

// Flush function based on mvturnho repo
void ili9488_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    static uint32_t flush_count = 0;
    flush_count++;
    
    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);
    
    // 前5次刷新打印详细信息
    if (flush_count <= 5) {
        ESP_LOGI(TAG, "ILI9488刷新第%d次 - 尺寸: %dx%d, 像素数: %d", 
                 flush_count, lv_area_get_width(area), lv_area_get_height(area), size);
    } else if (flush_count % 50 == 0) {
        ESP_LOGD(TAG, "ILI9488刷新第%d次", flush_count);
    }

    lv_color16_t *buffer_16bit = (lv_color16_t *) color_map;
    uint8_t *mybuf;
    do {
        mybuf = (uint8_t *) heap_caps_malloc(3 * size * sizeof(uint8_t), MALLOC_CAP_DMA);
        if (mybuf == NULL) {
            ESP_LOGW(TAG, "无法分配足够的DMA内存!");
            vTaskDelay(pdMS_TO_TICKS(1)); // 短暂延迟后重试
        }
    } while (mybuf == NULL);

    if (flush_count <= 5) {
        ESP_LOGD(TAG, "DMA缓冲区分配成功，地址: %p, 大小: %d bytes", mybuf, 3 * size);
    }

    uint32_t LD = 0;
    uint32_t j = 0;

    // RGB565转RGB888色彩转换
    for (uint32_t i = 0; i < size; i++) {
        LD = buffer_16bit[i].full;
        mybuf[j] = (uint8_t) (((LD & 0xF800) >> 8) | ((LD & 0x8000) >> 13));
        j++;
        mybuf[j] = (uint8_t) ((LD & 0x07E0) >> 3);
        j++;
        mybuf[j] = (uint8_t) (((LD & 0x001F) << 3) | ((LD & 0x0010) >> 2));
        j++;
    }
    
    // 打印前几个像素的颜色值用于调试
    if (flush_count <= 3) {
        ESP_LOGI(TAG, "前4个像素的RGB565值: 0x%04X, 0x%04X, 0x%04X, 0x%04X", 
                 buffer_16bit[0].full, buffer_16bit[1].full, buffer_16bit[2].full, buffer_16bit[3].full);
        ESP_LOGI(TAG, "转换后的RGB888值: R=%d G=%d B=%d (第1个像素)", 
                 mybuf[0], mybuf[1], mybuf[2]);
    }

	/* Column addresses  */
	uint8_t xb[] = {
	    (uint8_t) (area->x1 >> 8) & 0xFF,
	    (uint8_t) (area->x1) & 0xFF,
	    (uint8_t) (area->x2 >> 8) & 0xFF,
	    (uint8_t) (area->x2) & 0xFF,
	};

	/* Page addresses  */
	uint8_t yb[] = {
	    (uint8_t) (area->y1 >> 8) & 0xFF,
	    (uint8_t) (area->y1) & 0xFF,
	    (uint8_t) (area->y2 >> 8) & 0xFF,
	    (uint8_t) (area->y2) & 0xFF,
	};

	if (flush_count <= 5) {
	    ESP_LOGD(TAG, "设置显示窗口 - X: %d-%d, Y: %d-%d", area->x1, area->x2, area->y1, area->y2);
	}

	/*Column addresses*/
	ili9488_send_cmd(ILI9488_CMD_COLUMN_ADDRESS_SET);
	ili9488_send_data(xb, 4);

	/*Page addresses*/
	ili9488_send_cmd(ILI9488_CMD_PAGE_ADDRESS_SET);
	ili9488_send_data(yb, 4);

	/*Memory write*/
	ili9488_send_cmd(ILI9488_CMD_MEMORY_WRITE);

	if (flush_count <= 5) {
	    ESP_LOGD(TAG, "开始发送色彩数据，字节数: %d", size * 3);
	}
	ili9488_send_color((void *) mybuf, size * 3);
	heap_caps_free(mybuf);
	
	if (flush_count <= 5) {
	    ESP_LOGD(TAG, "ILI9488刷新完成");
	}
}

/**********************
 *   STATIC FUNCTIONS
 **********************/


static void ili9488_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ILI9488_DC, 0);	 /*Command mode*/
    disp_spi_send_data(&cmd, 1);
}

static void ili9488_send_data(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ILI9488_DC, 1);	 /*Data mode*/
    disp_spi_send_data(data, length);
}

static void ili9488_send_color(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ILI9488_DC, 1);   /*Data mode*/
    disp_spi_send_colors(data, length);
}

static void ili9488_set_orientation(uint8_t orientation)
{
    // ESP_ASSERT(orientation < 4);

    const char *orientation_str[] = {
        "PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"
    };

    ESP_LOGI(TAG, "Display orientation: %s (%d)", orientation_str[orientation], orientation);

#if defined (CONFIG_LV_PREDEFINED_DISPLAY_NONE)
    // 修正的方向数据 - 针对ILI9488优化
    uint8_t data[] = {
        0x48,  // Portrait: MY=0, MX=1, MV=0, ML=0, BGR=1, MH=0, 0, 0
        0x88,  // Portrait Inverted: MY=1, MX=0, MV=0, ML=0, BGR=1, MH=0, 0, 0  
        0x28,  // Landscape: MY=0, MX=0, MV=1, ML=0, BGR=1, MH=0, 0, 0
        0xE8   // Landscape Inverted: MY=1, MX=1, MV=1, ML=0, BGR=1, MH=0, 0, 0
    };
#endif

    ESP_LOGI(TAG, "0x36 command value: 0x%02X", data[orientation]);

    ili9488_send_cmd(0x36);
    ili9488_send_data((void *) &data[orientation], 1);
    
    // 添加小延迟确保命令执行
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "显示方向设置完成");
}
