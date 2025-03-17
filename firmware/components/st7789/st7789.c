#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"

#include "esp_log.h"
#include "esp_check.h"

#include "st7789.h"

static const char *TAG = "st7789";

static esp_err_t panel_st7789_del(esp_lcd_panel_t *panel);
static esp_err_t panel_st7789_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_st7789_init(esp_lcd_panel_t *panel);
static esp_err_t panel_st7789_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_st7789_invert_color(esp_lcd_panel_t *panel, bool invert);
static esp_err_t panel_st7789_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_st7789_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_st7789_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_st7789_disp_on_off(esp_lcd_panel_t *panel, bool on_off);
static esp_err_t panel_st7789_disp_sleep(esp_lcd_panel_t *panel, bool sleep);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t madctl_val;
} st7789_panel_t;

esp_err_t esp_lcd_new_panel_st7789(const esp_lcd_panel_io_handle_t io,
                                   const esp_lcd_panel_dev_config_t *panel_dev_cfg,
                                   esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    st7789_panel_t *st7789 = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_cfg && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    st7789 = (st7789_panel_t *)calloc(1, sizeof(st7789_panel_t));
    ESP_GOTO_ON_FALSE(st7789, ESP_ERR_NO_MEM, err, TAG, "no mem for st7789 panel");

    if (panel_dev_cfg->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_cfg->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    st7789->madctl_val = 0x00;

    st7789->io = io;
    st7789->reset_gpio_num = panel_dev_cfg->reset_gpio_num;
    st7789->reset_level = panel_dev_cfg->flags.reset_active_high;
    st7789->base.del = panel_st7789_del;
    st7789->base.reset = panel_st7789_reset;
    st7789->base.init = panel_st7789_init;
    st7789->base.draw_bitmap = panel_st7789_draw_bitmap;
    st7789->base.invert_color = panel_st7789_invert_color;
    st7789->base.mirror = panel_st7789_mirror;
    st7789->base.swap_xy = panel_st7789_swap_xy;
    st7789->base.set_gap = panel_st7789_set_gap;
    st7789->base.disp_on_off = panel_st7789_disp_on_off;
    st7789->base.disp_sleep = panel_st7789_disp_sleep;

    *ret_panel = &(st7789->base);

    ESP_LOGI(TAG, "new st7789 panel @%p", st7789);

    return ESP_OK;

err:
    if (st7789) {
        if (panel_dev_cfg->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_cfg->reset_gpio_num);
        }
        free(st7789);
    }
    return ret;
}

static esp_err_t panel_st7789_del(esp_lcd_panel_t *panel)
{
    st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);

    if (st7789->reset_gpio_num >= 0) {
        gpio_reset_pin(st7789->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del st7789 panel @%p", st7789);
    free(st7789);
    return ESP_OK;
}

static esp_err_t panel_st7789_reset(esp_lcd_panel_t *panel)
{
    st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;

    // perform hardware reset
    if (st7789->reset_gpio_num >= 0) {
        gpio_set_level(st7789->reset_gpio_num, st7789->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(st7789->reset_gpio_num, !st7789->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    else {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_OK;
}

static esp_err_t panel_st7789_init(esp_lcd_panel_t *panel)
{
    st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;
    esp_err_t ret;

    // Software Reset
    ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to reset the panel");
    vTaskDelay(pdMS_TO_TICKS(20));

    // Sleep Out to wake up from sleep mode
    ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to wake up from sleep mode");
    vTaskDelay(pdMS_TO_TICKS(120));

    // Display Inversion Off
    ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_INVOFF, NULL, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to disable display inversion");

    // Set Color Mode to 16-bit color
    uint8_t color_mode[] = { 0x55 };
    ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, color_mode, sizeof(color_mode));
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to set color mode");

    // Set Display Orientation
    // uint8_t madctl[] = { 0xC0 };  // MADCTL register setting for 90 degrees rotation
    // ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, madctl, sizeof(madctl));
    // ESP_RETURN_ON_ERROR(ret, TAG, "failed to set memory data access control");

    // Set Gamma Curve
    uint8_t gamma_curve[] = { 0x01 };  // Gamma curve 1 (choose 0, 1, 2, or 3)
    ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_GAMSET, gamma_curve, sizeof(gamma_curve));
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to set gamma curve");

    // Positive Gamma Control
    // uint8_t p_gamma[] = { 
    //     0x0F, 0x1F, 0x1C, 0x2F, 0x3F, 0x3F, 0x3F, 0x2F, 0x2F, 0x1F, 
    //     0x1C, 0x0F, 0x00, 0x00, 0x00, 0x00 
    // };  // Example values for positive gamma (adjust based on datasheet)
    // ret = esp_lcd_panel_io_tx_param(io, 0xE0, p_gamma, sizeof(p_gamma));
    // ESP_RETURN_ON_ERROR(ret, TAG, "failed to set positive gamma");

    // Negative Gamma Control
    // uint8_t n_gamma[] = { 
    //     0x00, 0x0F, 0x1F, 0x2F, 0x3F, 0x3F, 0x3F, 0x2F, 0x2F, 0x1F, 
    //     0x1C, 0x0F, 0x00, 0x00, 0x00, 0x00 
    // };  // Example values for negative gamma (adjust based on datasheet)
    // ret = esp_lcd_panel_io_tx_param(io, 0xE1, n_gamma, sizeof(n_gamma));
    // ESP_RETURN_ON_ERROR(ret, TAG, "failed to set negative gamma");

    // Set Frame Rate
    // uint8_t frame_rate[] = { 0x0B, 0x0B };
    // ret = esp_lcd_panel_io_tx_param(io, 0xB1, frame_rate, sizeof(frame_rate));
    // ESP_RETURN_ON_ERROR(ret, TAG, "failed to set frame rate");

    // Enable Display
    ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_DISPON, NULL, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to turn on display");

    // // Set the Address Window
    // uint8_t caset[] = {0x00, 0x00, 0x00, 0x84};
    // ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, caset, 4);  // Column address setting
    // ESP_RETURN_ON_ERROR(ret, TAG, "failed to set column address (CASET)");

    // uint8_t raset[] = {0x00, 0x00, 0x00, 0xF0};
    // ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, raset, 4);  // Row address setting
    // ESP_RETURN_ON_ERROR(ret, TAG, "failed to set row address (RASET)");

    ESP_LOGI(TAG, "ST7789 panel initialized successfully");

    return ESP_OK;
}

static esp_err_t panel_st7789_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;
    esp_err_t ret;

    x_start += st7789->x_gap;
    x_end += st7789->x_gap;
    y_start += st7789->y_gap;
    y_end += st7789->y_gap;

    uint8_t col_addr[] = {
        (uint8_t)(x_start >> 8), (uint8_t)(x_start & 0xFF),
        (uint8_t)(x_end >> 8), (uint8_t)(x_end & 0xFF),
    };
    ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, col_addr, sizeof(col_addr));
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to set column address (CASET)");

    uint8_t row_addr[] = {
        (uint8_t)(y_start >> 8), (uint8_t)(y_start & 0xFF),
        (uint8_t)(y_end >> 8), (uint8_t)(y_end & 0xFF),
    };
    ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, row_addr, sizeof(row_addr));
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to set row address (RASET)");

    size_t len = (x_end - x_start + 1) * (y_end - y_start + 1);
    ret = esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len * 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to write pixel data (RAMWR)");

    return ESP_OK;
}

static esp_err_t panel_st7789_invert_color(esp_lcd_panel_t *panel, bool invert)
{
    st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;

    uint8_t cmd = invert ? LCD_CMD_INVON : LCD_CMD_INVOFF;

    esp_err_t ret = esp_lcd_panel_io_tx_param(io, cmd, NULL, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "io tx param failed invert color");

    return ESP_OK;
}

static esp_err_t panel_st7789_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;

    if (mirror_x) {
        st7789->madctl_val |= 0x40;
    }
    else {
        st7789->madctl_val &= ~0x40;
    }

    if (mirror_y) {
        st7789->madctl_val |= 0x80;
    }
    else {
        st7789->madctl_val &= ~0x80;
    }

    esp_err_t ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){st7789->madctl_val}, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "io tx param failed mirror");

    return ESP_OK;
}

static esp_err_t panel_st7789_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;

    if (swap_axes) {
        st7789->madctl_val |= 0x20;
    }
    else {
        st7789->madctl_val &= ~0x20;
    }

    esp_err_t ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){st7789->madctl_val}, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "io tx param failed swap xy");

    return ESP_OK;
}

static esp_err_t panel_st7789_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);

    st7789->x_gap = x_gap;
    st7789->y_gap = y_gap;

    return ESP_OK;
}

static esp_err_t panel_st7789_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;

    uint8_t cmd = on_off ? LCD_CMD_DISPON : LCD_CMD_DISPOFF;

    esp_err_t ret = esp_lcd_panel_io_tx_param(io, cmd, NULL, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "io tx param failed disp on off");

    return ESP_OK;
}

static esp_err_t panel_st7789_disp_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;

    uint8_t cmd = sleep ? LCD_CMD_SLPIN : LCD_CMD_SLPOUT;

    esp_err_t ret = esp_lcd_panel_io_tx_param(io, cmd, NULL, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "io tx param failed sleep");

    return ESP_OK;
}