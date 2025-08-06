/**
 * @file disp_driver.c
 */

#include "disp_driver.h"
#include "disp_spi.h"
#include "esp_lcd_backlight.h"
#include "sdkconfig.h"
#include "esp_log.h"

static const char *TAG = "DISP_DRIVER";

void *disp_driver_init(void)
{
    ESP_LOGI(TAG, "初始化显示驱动器...");
    
#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9341
    ESP_LOGI(TAG, "使用ILI9341控制器");
    ili9341_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9481
    ESP_LOGI(TAG, "使用ILI9481控制器");
    ili9481_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9488
    ESP_LOGI(TAG, "使用ILI9488控制器");
    ili9488_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7789
    ESP_LOGI(TAG, "使用ST7789控制器");
    st7789_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7796S
    ESP_LOGI(TAG, "使用ST7796S控制器");
    st7796s_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7735S
    ESP_LOGI(TAG, "使用ST7735S控制器");
    st7735s_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_HX8357
    ESP_LOGI(TAG, "使用HX8357控制器");
    hx8357_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9486
    ESP_LOGI(TAG, "使用ILI9486控制器");
    ili9486_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SH1107
    ESP_LOGI(TAG, "使用SH1107控制器");
    sh1107_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306
    ESP_LOGI(TAG, "使用SSD1306控制器");
    ssd1306_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_FT81X
    ESP_LOGI(TAG, "使用FT81X控制器");
    FT81x_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820
    ESP_LOGI(TAG, "使用IL3820控制器");
    il3820_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_RA8875
    ESP_LOGI(TAG, "使用RA8875控制器");
    ra8875_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_GC9A01
   ESP_LOGI(TAG, "使用GC9A01控制器");
   GC9A01_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A
   ESP_LOGI(TAG, "使用JD79653A控制器");
   jd79653a_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D
   ESP_LOGI(TAG, "使用UC8151D控制器");
   uc8151d_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9163C
    ESP_LOGI(TAG, "使用ILI9163C控制器");
    ili9163c_init();
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_PCD8544
    ESP_LOGI(TAG, "使用PCD8544控制器");
    pcd8544_init();
#endif

    // We still use menuconfig for these settings
    // It will be set up during runtime in the future
#if (defined(CONFIG_LV_DISP_BACKLIGHT_SWITCH) || defined(CONFIG_LV_DISP_BACKLIGHT_PWM))
    ESP_LOGI(TAG, "初始化背光控制...");
    const disp_backlight_config_t bckl_config = {
        .gpio_num = CONFIG_LV_DISP_PIN_BCKL,
#if defined CONFIG_LV_DISP_BACKLIGHT_PWM
        .pwm_control = true,
#else
        .pwm_control = false,
#endif
#if defined CONFIG_LV_BACKLIGHT_ACTIVE_LVL
        .output_invert = false, // Backlight on high
#else
        .output_invert = true, // Backlight on low
#endif
        .timer_idx = 0,
        .channel_idx = 0 // @todo this prevents us from having two PWM controlled displays
    };
    ESP_LOGI(TAG, "背光GPIO引脚: %d", bckl_config.gpio_num);
    ESP_LOGI(TAG, "PWM控制: %s", bckl_config.pwm_control ? "是" : "否");
    ESP_LOGI(TAG, "输出反转: %s", bckl_config.output_invert ? "是" : "否");
    
    disp_backlight_h bckl_handle = disp_backlight_new(&bckl_config);
    if (bckl_handle != NULL) {
        disp_backlight_set(bckl_handle, 100);
        ESP_LOGI(TAG, "背光初始化成功，亮度设置为100%%");
    } else {
        ESP_LOGE(TAG, "背光初始化失败");
    }
    return bckl_handle;
#else
    ESP_LOGW(TAG, "背光控制未启用");
    return NULL;
#endif
}

void disp_driver_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    static uint32_t flush_count = 0;
    flush_count++;
    
    // 每100次刷新打印一次详细信息，前10次每次都打印
    if (flush_count <= 10 || flush_count % 100 == 0) {
        ESP_LOGI(TAG, "刷新第%d次 - 区域: x1=%d, y1=%d, x2=%d, y2=%d, 宽度=%d, 高度=%d", 
                 flush_count, area->x1, area->y1, area->x2, area->y2, 
                 lv_area_get_width(area), lv_area_get_height(area));
        ESP_LOGI(TAG, "色彩缓冲区地址: %p", color_map);
    }
    
#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9341
    ili9341_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9481
    ili9481_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9488
    ili9488_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7789
    st7789_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7796S
    st7796s_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7735S
    st7735s_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_HX8357
	hx8357_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9486
    ili9486_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SH1107
	sh1107_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306
    ssd1306_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_FT81X
    FT81x_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820
    il3820_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_RA8875
    ra8875_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_GC9A01
    GC9A01_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A
    jd79653a_lv_fb_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D
    uc8151d_lv_fb_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9163C
    ili9163c_flush(drv, area, color_map);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_PCD8544
    pcd8544_flush(drv, area, color_map);
#endif
}

void disp_driver_rounder(lv_disp_drv_t * disp_drv, lv_area_t * area)
{
#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306
    ssd1306_rounder(disp_drv, area);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SH1107
    sh1107_rounder(disp_drv, area);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820
    il3820_rounder(disp_drv, area);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A
    jd79653a_lv_rounder_cb(disp_drv, area);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D
    uc8151d_lv_rounder_cb(disp_drv, area);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_PCD8544
    pcd8544_rounder(disp_drv, area);
#endif
}

void disp_driver_set_px(lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa)
{
#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306
    ssd1306_set_px_cb(disp_drv, buf, buf_w, x, y, color, opa);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SH1107
    sh1107_set_px_cb(disp_drv, buf, buf_w, x, y, color, opa);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820
    il3820_set_px_cb(disp_drv, buf, buf_w, x, y, color, opa);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A
    jd79653a_lv_set_fb_cb(disp_drv, buf, buf_w, x, y, color, opa);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D
    uc8151d_lv_set_fb_cb(disp_drv, buf, buf_w, x, y, color, opa);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_PCD8544
   pcd8544_set_px_cb(disp_drv, buf, buf_w, x, y, color, opa);
#endif
}
