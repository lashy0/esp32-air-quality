#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"

static const char *TAG = "main";

#define PIN_NUM_MOSI    19
#define PIN_NUM_CLK     18
#define PIN_NUM_CS      5
#define PIN_NUM_DC      16
#define PIN_NUM_RST     23
#define PIN_NUM_BCKL    4

spi_device_handle_t spi;

void st7789_send_command(uint8_t cmd) {
    // DC = 0 => режим отправки команды
    gpio_set_level(PIN_NUM_DC, 0);

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_transmit(spi, &t);
}

void st7789_send_data(const uint8_t *data, size_t len) {
    // DC = 1 => режим отправки данных
    gpio_set_level(PIN_NUM_DC, 1);

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    spi_device_transmit(spi, &t);
}

void st7789_init() {
    // Сброс (RST)
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    st7789_send_command(0x01); // Software reset
    vTaskDelay(pdMS_TO_TICKS(150));

    st7789_send_command(0x11); // Выход из режима сна (Sleep Out)
    vTaskDelay(pdMS_TO_TICKS(150));

    // Устанавливаем формат пикселя (16-бит RGB565)
    st7789_send_command(0x3A);
    uint8_t data = 0x55; // 0x55 = 16bit, 0x66 = 18bit
    st7789_send_data(&data, 1);

    // Настраиваем адресное окно: 135x240
    // Важно учитывать, что ST7789 часто требует "offset", 
    // но для TTGO T-Display обычно подойдёт такая конфигурация:
    st7789_send_command(0x2A); // CASET
    uint8_t caset[] = {0x00, 0x00, 0x00, 0x84}; // 0..132(0x84) или 0..134(0x86) 
    st7789_send_data(caset, sizeof(caset));

    st7789_send_command(0x2B); // RASET
    uint8_t raset[] = {0x00, 0x00, 0x00, 0xF0}; // 0..240(0xF0) или 0..239(0xEF)
    st7789_send_data(raset, sizeof(raset));

    st7789_send_command(0x29); // Включаем дисплей (Display ON)

    ESP_LOGI(TAG, "ST7789 initialization done.");
}

void app_main(void) {
    // Настройка SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,   // не используется
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1, 
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 26 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7
    };

    // Инициализируем шину SPI
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));

    // Настройка GPIO для DC, RST, BL
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    // Включаем подсветку
    gpio_set_level(PIN_NUM_BCKL, 1);

    // Инициализация дисплея
    st7789_init();
}