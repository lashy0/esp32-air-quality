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
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val;
    uint8_t colmod_val;
} st7789_panel_t;

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes;
} lcd_init_cmd_t;

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

    switch (panel_dev_cfg->rgb_ele_order) {
    case LCD_RGB_ELEMENT_ORDER_RGB:
        st7789->madctl_val = 0x00;
        break;
    case LCD_RGB_ELEMENT_ORDER_BGR:
        st7789->madctl_val = 0x08;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported RGB element order");
        break;
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_cfg->bits_per_pixel) {
    case 16:
        st7789->colmod_val = 0x55;
        fb_bits_per_pixel = 16;
        break;
    case 18:
        st7789->colmod_val = 0x66;
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    st7789->io = io;
    st7789->fb_bits_per_pixel = fb_bits_per_pixel;
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

    if (st7789->reset_gpio_num >= 0) {
        gpio_set_level(st7789->reset_gpio_num, st7789->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(st7789->reset_gpio_num, !st7789->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    else {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send reset failed");
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_OK;
}

static esp_err_t panel_st7789_init(esp_lcd_panel_t *panel)
{
    st7789_panel_t *st7789 = __containerof(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;
    esp_err_t ret;

    // Extended initialization commands for ST7789
    // lcd_init_cmd_t st7789_init_cmds[] = {
    //     {0x13, {0}, 0}, // NORON
    //     // {0x36, {st7789->madctl_val}, 1}, // Memory Data Access Control
    //     {0x36, {0x00}, 1}, // Memory Data Access Control
    //     // {0xB6, {0x0A, 0x82}, 2},
    //     // {0xB0, {0x00, 0xF0}, 2}, // RAMCTRL
    //     {0x3A, {st7789->colmod_val}, 1}, // Set Color Mode
    //     {0xB2, {0x0b, 0x0b, 0x00, 0x33, 0x33}, 5}, // PORCTRL
    //     {0xB7, {0x75}, 1}, // GCTRL
    //     {0xBB, {0x28}, 1}, // VCOMS
    //     {0xC0, {0x2C}, 1}, // LCMCTRL
    //     {0xC2, {0x01}, 1}, // VDVVRHEN
    //     {0xC3, {0x1F}, 1}, //VRHS
    //     // {0xC4, {0x20}, 1}, // VDVSET
    //     {0xC6, {0x13}, 1}, // FRCTR2
    //     {0xD0, {0xa7}, 1}, // PWCTRL1
    //     {0xD0, {0xa4, 0xa1}, 2}, // PWCTRL1
    //     {0xD6, {0xa1}, 1},
    //     {0xE0, {0xf0, 0x05, 0x0a, 0x06, 0x06, 0x03, 0x2b, 0x32, 0x43, 0x36, 0x11, 0x10, 0x2b, 0x32}, 14}, // Positive Gamma Correction
    //     {0xE1, {0xf0, 0x08, 0x0c, 0x0b, 0x09, 0x24, 0x2b, 0x22, 0x43, 0x38, 0x15, 0x16, 0x2f, 0x37}, 14}, // Negative Gamma Correction
    //     // {0x21, {0}, 0}, // INVON
    //     // {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4}, // CASET
    //     // {0x2B, {0x00, 0x00, 0x01, 0x3F}, 4}, // RASET
    //     {0, {0}, 0xFF}                   // End of commands
    // };

    // lcd_init_cmd_t st7789_init_cmds[] = {
    //     {0xCF, {0x00, 0x83, 0X30}, 3},
    //     {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    //     {0xE8, {0x85, 0x01, 0x79}, 3}, // PWCTRL2
    //     {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    //     {0xF7, {0x20}, 1},
    //     {0xEA, {0x00, 0x00}, 2},
    //     {0xC0, {0x26}, 1}, // LCMCTRL
    //     {0xC1, {0x11}, 1}, // IDSET
    //     {0xC5, {0x35, 0x3E}, 2}, // VCMOFSET
    //     {0xC7, {0xBE}, 1}, // CABCCTRL
    //     {0x36, {0x00}, 1}, // Memory Data Access Control
    //     {0x3A, {0x55}, 1}, // Set Color Mode
    //     {0x21, {0}, 0}, // INVON
    //     // {0x20, {0}, 0}, // ONVOFF
    //     {0xB1, {0x00, 0x1B}, 2}, // RGBCTRL
    //     {0xF2, {0x00}, 1},
    //     {0x26, {0xa1}, 1}, // GAMSET
    //     {0xE0, {0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x32, 0x44, 0x42, 0x06, 0x0E, 0x12, 0x14, 0x17}, 14}, // Positive Gamma Correction
    //     {0xE1, {0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x31, 0x54, 0x47, 0x0E, 0x1C, 0x17, 0x1B, 0x1E}, 14}, // Negative Gamma Correction
    //     {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4}, // CASET
    //     {0x2B, {0x00, 0x00, 0x01, 0x3F}, 4}, // RASET
    //     {0x2C, {0}, 0}, // RAMWR
    //     {0xB7, {0x07}, 1}, // GCTRL
    //     {0xB6, {0x0A, 0xB2, 0x27, 0x00}, 4},
    //     {0, {0}, 0xFF}                   // End of commands
    // };

    lcd_init_cmd_t st7789_init_cmds[] = {
        {0x36, {st7789->madctl_val}, 1}, // Memory Data Access Control
        {0x3A, {st7789->colmod_val}, 1}, // Set Color Mode
        {0xB0, {0x00, 0xF0}, 2}, // RAMCTRL
        {0, {0}, 0xFF},
    };

    // Software Reset
    ret = esp_lcd_panel_io_tx_param(io, 0x01, NULL, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to reset the panel");
    vTaskDelay(pdMS_TO_TICKS(20));

    // Sleep Out to wake up from sleep mode
    ret = esp_lcd_panel_io_tx_param(io, 0x11, NULL, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to wake up from sleep mode");
    vTaskDelay(pdMS_TO_TICKS(120));

    // Send initialization commands
    uint16_t cmd = 0;
    while (st7789_init_cmds[cmd].databytes != 0xFF) {
        ret = esp_lcd_panel_io_tx_param(
            io, st7789_init_cmds[cmd].cmd, st7789_init_cmds[cmd].data, st7789_init_cmds[cmd].databytes
        );
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed to send init command");
        vTaskDelay(pdMS_TO_TICKS(10));
        cmd++;
    }

    // Enable Display
    // ret = esp_lcd_panel_io_tx_param(io, 0x29, NULL, 0);
    // ESP_RETURN_ON_ERROR(ret, TAG, "failed to turn on display");
    // vTaskDelay(pdMS_TO_TICKS(100));

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
        (uint8_t)((x_start >> 8) & 0xFF), (uint8_t)(x_start & 0xFF),
        (uint8_t)(((x_end - 1) >> 8) & 0xFF), (uint8_t)((x_end - 1) & 0xFF),
    };
    ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, col_addr, sizeof(col_addr));
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to set column address (CASET)");

    uint8_t row_addr[] = {
        (uint8_t)((y_start >> 8) & 0xFF), (uint8_t)(y_start & 0xFF),
        (uint8_t)(((y_end - 1) >> 8) & 0xFF), (uint8_t)((y_end - 1) & 0xFF),
    };
    ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, row_addr, sizeof(row_addr));
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to set row address (RASET)");

    size_t len = (x_end - x_start) * (y_end - y_start) * st7789->fb_bits_per_pixel / 8;
    ret = esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len);
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
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}