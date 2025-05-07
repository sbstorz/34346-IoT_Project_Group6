#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "adxl345.h"
#include "uI2C.h"

static const char *TAG_MAIN = "APP_MAIN";
static const char *TAG_MOTION = "MOTION_CTRL";

#define ADXL345_INT1_GPIO GPIO_NUM_4
#define ACTIVITY_THRESHOLD_VALUE 8
#define INACTIVITY_TIMEOUT_US (5 * 1000 * 1000)
#define MOTION_CHECK_PERIOD_MS 200

static SemaphoreHandle_t adxl_interrupt_sem = NULL;
static volatile int64_t last_motion_timestamp = 0;
static volatile bool is_currently_inactive = true;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    xSemaphoreGiveFromISR(adxl_interrupt_sem, NULL);
}

static void adxl_interrupt_handler_task(void *arg)
{
    ESP_LOGI(TAG_MOTION, "ADXL Interrupt Handler Task started.");
    for (;;)
    {
        if (xSemaphoreTake(adxl_interrupt_sem, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGD(TAG_MOTION, "Semaphore taken, checking ADXL INT source...");
            adxl345_check_activity_interrupt_source();
        }
    }
}

static void motion_inactivity_task(void *arg)
{
    ESP_LOGI(TAG_MOTION, "Motion/Inactivity Logic Task started.");
    last_motion_timestamp = esp_timer_get_time();
    is_currently_inactive = true;
    ESP_LOGI(TAG_MOTION, "System initialized as inactive.");

    for (;;)
    {
        bool motion_detected = adxl345_has_motion_occurred(true);

        if (motion_detected)
        {
            last_motion_timestamp = esp_timer_get_time();
            if (is_currently_inactive)
            {
                ESP_LOGW(TAG_MOTION, "Motion Detected! (System was inactive)");
                is_currently_inactive = false;
            }
        }
        else
        {
            if (!is_currently_inactive)
            {
                int64_t current_time = esp_timer_get_time();
                int64_t time_since_last_motion = current_time - last_motion_timestamp;

                if (time_since_last_motion > INACTIVITY_TIMEOUT_US)
                {
                    ESP_LOGW(TAG_MOTION, "Inactive for 5 seconds.");
                    is_currently_inactive = true;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MOTION_CHECK_PERIOD_MS));
    }
}

// --- I2C Scanner Function ---
static void i2c_scanner(void)
{
    ESP_LOGI(TAG_MAIN, "Starting I2C scan...");
    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    for (int i = 0; i < 128; i += 16)
    {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++)
        {
            address = i + j;
            // Skip reserved addresses
            if (address < 0x08 || address > 0x77)
            {
                printf("   ");
                continue;
            }

            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true); // Check for ACK
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS); // 50ms timeout
            i2c_cmd_link_delete(cmd);

            if (ret == ESP_OK)
            {
                printf("%02x ", address);
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                printf("UU "); // Timeout (shouldn't happen often)
            }
            else
            {
                printf("-- "); // No ACK received
            }
        }
        printf("\n");
    }
    ESP_LOGI(TAG_MAIN, "I2C scan finished.");
}
// --- End I2C Scanner ---

void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "System Startup...");

    // --- Initialize I2C ---
    ESP_LOGI(TAG_MAIN, "Initializing I2C Master...");
    ESP_LOGI(TAG_MAIN, "I2C SDA: %d, SCL: %d", SDA_IO_PIN, SCL_IO_PIN);
    adxl345_i2c_master_init(SDA_IO_PIN, SCL_IO_PIN);

    // --- Scan I2C Bus ---
    // i2c_scanner(); // Run the scanner FIRST

    // --- Initialize ADXL345 ---
    ESP_LOGI(TAG_MAIN, "Setting ADXL345 to measure mode...");
    // adxl345_set_measure_mode(); // Includes device ID check

    ESP_LOGI(TAG_MAIN, "Configuring ADXL345 activity interrupt...");
    esp_err_t config_ret = adxl345_config_activity_int(ACTIVITY_THRESHOLD_VALUE, ADXL345_INT1_PIN);
    if (config_ret != ESP_OK)
    {
        ESP_LOGE(TAG_MAIN, "Failed to configure ADXL345 activity interrupt! Error: 0x%x", config_ret);
        return;
    }
    ESP_LOGI(TAG_MAIN, "ADXL345 Activity Interrupt Configured (Threshold: %d, Pin: INT1)", ACTIVITY_THRESHOLD_VALUE);

    adxl_interrupt_sem = xSemaphoreCreateBinary();
    if (adxl_interrupt_sem == NULL)
    {
        ESP_LOGE(TAG_MAIN, "Failed to create interrupt semaphore!");
        return;
    }
    ESP_LOGI(TAG_MAIN, "Interrupt Semaphore created.");

    ESP_LOGI(TAG_MAIN, "Configuring GPIO interrupt for pin %d...", ADXL345_INT1_GPIO);
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << ADXL345_INT1_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    esp_err_t isr_service_ret = gpio_install_isr_service(0);
    if (isr_service_ret != ESP_OK && isr_service_ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG_MAIN, "Failed to install GPIO ISR service! Error: 0x%x", isr_service_ret);
        return;
    }

    esp_err_t isr_hook_ret = gpio_isr_handler_add(ADXL345_INT1_GPIO, gpio_isr_handler, (void *)ADXL345_INT1_GPIO);
    if (isr_hook_ret != ESP_OK)
    {
        ESP_LOGE(TAG_MAIN, "Failed to add GPIO ISR handler! Error: 0x%x", isr_hook_ret);
        return;
    }
    ESP_LOGI(TAG_MAIN, "GPIO Interrupt configured.");

    ESP_LOGI(TAG_MAIN, "Creating Tasks...");
    xTaskCreate(adxl_interrupt_handler_task, "adxl_int_handler", 2048, NULL, 10, NULL);
    xTaskCreate(motion_inactivity_task, "motion_logic", 2560, NULL, 5, NULL);

    ESP_LOGI(TAG_MAIN, "Initialization Complete. Tasks running.");
}