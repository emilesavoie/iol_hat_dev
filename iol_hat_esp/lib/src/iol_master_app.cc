#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#include "osal.h"
#include "osal_log.h"
#include "osal_irq.h"
#include "iolink.h"
#include "iolink_max14819.h"
#include "iolink_max14819_pl.h"
#include "iolink_handler.h"
#include "server.h"
#include "status.h"
#include "common.h"

static const char* TAG = "IOL_HAT_ESP";

// Pin definitions (adjust as needed)
#define PIN_SPI_MISO    19
#define PIN_SPI_MOSI    23
#define PIN_SPI_SCLK    18
#define PIN_CS          5
#define PIN_IRQ         4

static spi_device_handle_t spi_dev;
static iolink_hw_drv_t* hw_drv;

// SPI interrupt handler wrapper
static void IRAM_ATTR max14819_isr(void* arg) {
    iolink_14819_isr((iolink_hw_drv_t*)arg);
}

// Initialize SPI and attach MAX14819 device
static esp_err_t init_spi_driver() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_SPI_MISO,
        .mosi_io_num = PIN_SPI_MOSI,
        .sclk_io_num = PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,           // 1 MHz
        .mode = 0,                            // SPI mode 0
        .spics_io_num = PIN_CS,
        .queue_size = 3,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    return spi_bus_add_device(VSPI_HOST, &devcfg, &spi_dev);
}

// Set up GPIO interrupt for IRQ line
static void init_gpio_irq(iolink_hw_drv_t* drv) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << PIN_IRQ,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_IRQ, max14819_isr, drv);
}

// FreeRTOS tasks
static void iolink_handler_task(void* arg) {
    iolink_m_cfg_t* cfg = (iolink_m_cfg_t*)arg;
    iolink_handler(*cfg);
    vTaskDelete(NULL);
}

static void tcp_server_task(void* arg) {
    socket_thread_t* ctx = (socket_thread_t*)arg;
    runServer(ctx);
    vTaskDelete(NULL);
}

static void status_task(void* arg) {
    runStatus(NULL);
    vTaskDelete(NULL);
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "IOL HAT master starting on ESP32");

    // Initialize NVS for Wi-Fi
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize SPI
    ESP_ERROR_CHECK(init_spi_driver());

    // Configure MAX14819
    iolink_14819_cfg_t cfg = {};
    cfg.chip_address = IOLINK_APP_CHIP0_ADDRESS;
    cfg.CQCfgA       = 0x02;
    cfg.LPCnfgA      = 0x00;
    cfg.IOStCfgA     = 0x25;
    cfg.CQCfgB       = 0x02;
    cfg.LPCnfgB      = 0x00;
    cfg.IOStCfgB     = 0x25;
    cfg.DrvCurrLim   = 0xC0;
    cfg.Clock        = 0x11;
    cfg.cs_channel   = 0;
    cfg.spi_handle   = &spi_dev;  // assume extended field

    hw_drv = iolink_14819_init(&cfg);
    assert(hw_drv);

    // IRQ setup
    init_gpio_irq(hw_drv);

    // IOLink port config
    iolink_pl_mode_t mode = iolink_mode_SDCI;
    iolink_port_cfg_t port_cfg = {
        .name = "/iolink0/0",
        .mode = &mode,
        .drv  = hw_drv,
        .arg  = (void*)0,
    };
    iolink_m_cfg_t mcfg = {};
    mcfg.port_cnt = 1;
    mcfg.port_cfgs = &port_cfg;
    mcfg.master_thread_prio = 6;
    mcfg.master_thread_stack_size = 4 * 1024;
    mcfg.dl_thread_prio = 7;
    mcfg.dl_thread_stack_size = 1500;

    // Start handler task
    xTaskCreate(iolink_handler_task, "iolink_handler", 4096, &mcfg, 6, NULL);

    // Start TCP server task
    static socket_thread_t tcp_arg;
    tcp_arg.iolink_hw = hw_drv;
    tcp_arg.tcpPort = DEFAULT_TCP_12;
    xTaskCreate(tcp_server_task, "tcp_server", 4096, &tcp_arg, 5, NULL);

    // Start status task
    xTaskCreate(status_task, "status", 2048, NULL, 5, NULL);

    // Main loop idle
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
