#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "adxl345.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uI2C.h"

static const char *TAG_MAIN = "APP_MAIN";
static const char *TAG_MOTION = "MOTION_CTRL";

#define ADXL345_INT1_GPIO GPIO_NUM_4
#define ACTIVITY_THRESHOLD_VALUE 80
#define INACTIVITY_THRESHOLD_VALUE 6
#define INACTIVITY_TIME_S 5
#define DEBUG_PIN_CHECK_PERIOD_S 5
#define INTERRUPT_DEBOUNCE_MS 200  // Debounce time in milliseconds

static volatile bool is_currently_inactive = true;
static TaskHandle_t adxl_task_handle = NULL;
static int64_t last_interrupt_time = 0;  // To track last interrupt time for debounce

static void interrupt_pin_debug_task(void *arg) {
    ESP_LOGI(TAG_MAIN, "Interrupt pin debug task started.");
    for (;;) {
        // Read the current state of the interrupt pin
        int pin_level = gpio_get_level(ADXL345_INT1_GPIO);
        ESP_LOGI(TAG_MAIN, "DEBUG: INT1 pin (GPIO%d) state: %s", 
                 ADXL345_INT1_GPIO, 
                 pin_level ? "HIGH" : "LOW");
        
        // Also read the INT_SOURCE register to see if there are pending interrupts
        uint8_t int_source;
        if (adxl345_read_reg(ADXL345_REG_INT_SOURCE, &int_source) == ESP_OK) {
            ESP_LOGI(TAG_MAIN, "DEBUG: INT_SOURCE register: 0x%02x", int_source);
            if (int_source & ADXL345_INT_SOURCE_ACTIVITY)
                ESP_LOGI(TAG_MAIN, "DEBUG: Activity interrupt flag is SET");
            if (int_source & ADXL345_INT_SOURCE_INACTIVITY)
                ESP_LOGI(TAG_MAIN, "DEBUG: Inactivity interrupt flag is SET");
        }
        
        vTaskDelay(pdMS_TO_TICKS(DEBUG_PIN_CHECK_PERIOD_S * 1000));
    }
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    // Get current time for debouncing
    int64_t current_time = esp_timer_get_time() / 1000;  // Convert to milliseconds
    
    // Simple debounce - ignore interrupts that come too quickly after the previous one
    if ((current_time - last_interrupt_time) < INTERRUPT_DEBOUNCE_MS) {
        return;  // Ignore this interrupt - it's too soon after the last one
    }
    
    // Update the last interrupt time
    last_interrupt_time = current_time;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (adxl_task_handle != NULL) {
        // Send a notification to the adxl_interrupt_handler_task
        vTaskNotifyGiveFromISR(adxl_task_handle, &xHigherPriorityTaskWoken);
    }
    // If xHigherPriorityTaskWoken is now set to pdTRUE, then a context switch should be requested.
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void adxl_interrupt_handler_task(void *arg) {
    ESP_LOGI(TAG_MOTION, "ADXL Interrupt Handler Task started.");
    for (;;) {
        // Wait indefinitely for a notification.
        // pdTRUE as the first arg means it acts like a binary semaphore (clears notification count on take).
        uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (ulNotificationValue > 0) // A notification was received
        {
            ESP_LOGI(TAG_MOTION, "Interrupt received, processing ADXL INT source...");
            
            // Process both activity and inactivity interrupts
            uint8_t int_source = adxl345_process_interrupts();
            
            // Handle Activity interrupt
            if (int_source & ADXL345_INT_SOURCE_ACTIVITY) {
                if (is_currently_inactive) {
                    ESP_LOGW(TAG_MOTION, "Motion detected! (System was inactive)");
                    is_currently_inactive = false;
                } else {
                    ESP_LOGI(TAG_MOTION, "Additional motion detected");
                }
            }
            
            // Handle Inactivity interrupt
            if (int_source & ADXL345_INT_SOURCE_INACTIVITY) {
                if (!is_currently_inactive) {
                    ESP_LOGW(TAG_MOTION, "No motion detected for %d seconds - now inactive", INACTIVITY_TIME_S);
                    is_currently_inactive = true;
                }
            }
            
            // Check if interrupt pin has been pulled low
            int pin_level = gpio_get_level(ADXL345_INT1_GPIO);
            ESP_LOGI(TAG_MOTION, "After interrupt processing, INT1 pin is: %s", 
                     pin_level ? "STILL HIGH" : "LOW (correctly cleared)");
            
            // Add a short delay to allow the interrupt pin to settle
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG_MAIN, "System Startup...");

    // --- Initialize I2C ---
    ESP_LOGI(TAG_MAIN, "Initializing I2C Master...");
    ESP_LOGI(TAG_MAIN, "I2C SDA: %d, SCL: %d", SDA_IO_PIN, SCL_IO_PIN);
    adxl345_i2c_master_init(SDA_IO_PIN, SCL_IO_PIN);
    // --- Initialize ADXL345 ---
    ESP_LOGI(TAG_MAIN, "Setting ADXL345 to measure mode...");
    adxl345_set_measure_mode();  // Includes device ID check
    
    // Simple read test for ADXL345
    uint8_t buf[8];
    uint8_t addrs[6] = {ADXL345_REG_DATAX0, ADXL345_REG_DATAX1,
                        ADXL345_REG_DATAY0, ADXL345_REG_DATAY1,
                        ADXL345_REG_DATAZ0, ADXL345_REG_DATAZ1};
    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_NUM, ADXL345_DEFAULT_ADDRESS, &addrs[0], 1, &buf[0], 1,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG_MAIN, "ADXL345 Device ID: 0x%02x", ret);
    } else {
        ESP_LOGE(TAG_MAIN, "Failed to read ADXL345 Device ID! Error: 0x%x",
                 ret);
    }

    // --- Configure Activity Interrupt ---
    ESP_LOGI(TAG_MAIN, "Configuring ADXL345 activity interrupt...");
    esp_err_t config_ret =
        adxl345_config_activity_int(ACTIVITY_THRESHOLD_VALUE, ADXL345_INT1_PIN);
    if (config_ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN,
                 "Failed to configure ADXL345 activity interrupt! Error: 0x%x",
                 config_ret);
        return;
    }
    ESP_LOGI(TAG_MAIN,
             "ADXL345 Activity Interrupt Configured (Threshold: %d, Pin: INT1)",
             ACTIVITY_THRESHOLD_VALUE);

    // --- Configure Inactivity Interrupt ---
    ESP_LOGI(TAG_MAIN, "Configuring ADXL345 inactivity interrupt...");
    config_ret = adxl345_config_inactivity_int(INACTIVITY_THRESHOLD_VALUE, INACTIVITY_TIME_S, ADXL345_INT1_PIN);
    if (config_ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN,
                 "Failed to configure ADXL345 inactivity interrupt! Error: 0x%x",
                 config_ret);
        return;
    }
    ESP_LOGI(TAG_MAIN,
             "ADXL345 Inactivity Interrupt Configured (Threshold: %d, Time: %ds, Pin: INT1)",
             INACTIVITY_THRESHOLD_VALUE, INACTIVITY_TIME_S);

    // --- Verify Configuration by Reading Back Registers ---
    ESP_LOGI(TAG_MAIN, "Verifying ADXL345 configuration...");
    uint8_t read_val;
    esp_err_t read_ret;

    read_ret = adxl345_read_reg(ADXL345_REG_THRESH_ACT, &read_val);
    if (read_ret == ESP_OK)
        ESP_LOGI(TAG_MAIN,
                 "  Read THRESH_ACT    (0x24): 0x%02x (Expected: 0x%02x)",
                 read_val, ACTIVITY_THRESHOLD_VALUE);
    else
        ESP_LOGE(TAG_MAIN, "  Failed to read THRESH_ACT (err=0x%x)", read_ret);

    read_ret = adxl345_read_reg(ADXL345_REG_THRESH_INACT, &read_val);
    if (read_ret == ESP_OK)
        ESP_LOGI(TAG_MAIN,
                 "  Read THRESH_INACT  (0x25): 0x%02x (Expected: 0x%02x)",
                 read_val, INACTIVITY_THRESHOLD_VALUE);
    else
        ESP_LOGE(TAG_MAIN, "  Failed to read THRESH_INACT (err=0x%x)", read_ret);

    read_ret = adxl345_read_reg(ADXL345_REG_TIME_INACT, &read_val);
    if (read_ret == ESP_OK)
        ESP_LOGI(TAG_MAIN,
                 "  Read TIME_INACT    (0x26): 0x%02x (Expected: 0x%02x)",
                 read_val, INACTIVITY_TIME_S);
    else
        ESP_LOGE(TAG_MAIN, "  Failed to read TIME_INACT (err=0x%x)", read_ret);

    read_ret = adxl345_read_reg(ADXL345_REG_ACT_INACT_CTL, &read_val);
    uint8_t expected_act_ctl =
        ADXL345_ACT_X_EN | ADXL345_ACT_Y_EN | ADXL345_ACT_Z_EN |
        ADXL345_INACT_X_EN | ADXL345_INACT_Y_EN | ADXL345_INACT_Z_EN;
    if (read_ret == ESP_OK)
        ESP_LOGI(TAG_MAIN,
                 "  Read ACT_INACT_CTL (0x27): 0x%02x (Expected: 0x%02x)",
                 read_val, expected_act_ctl);
    else
        ESP_LOGE(TAG_MAIN, "  Failed to read ACT_INACT_CTL (err=0x%x)",
                 read_ret);

    read_ret = adxl345_read_reg(ADXL345_REG_INT_MAP, &read_val);
    uint8_t expected_int_map = 0x00;  // Both Activity and Inactivity mapped to INT1
    if (read_ret == ESP_OK)
        ESP_LOGI(TAG_MAIN,
                 "  Read INT_MAP       (0x2F): 0x%02x (Expected: 0x%02x)",
                 read_val, expected_int_map);
    else
        ESP_LOGE(TAG_MAIN, "  Failed to read INT_MAP (err=0x%x)", read_ret);

    read_ret = adxl345_read_reg(ADXL345_REG_INT_ENABLE, &read_val);
    uint8_t expected_int_enable = ADXL345_INT_ENABLE_ACTIVITY | ADXL345_INT_ENABLE_INACTIVITY;
    if (read_ret == ESP_OK)
        ESP_LOGI(TAG_MAIN,
                 "  Read INT_ENABLE    (0x2E): 0x%02x (Expected: 0x%02x)",
                 read_val, expected_int_enable);
    else
        ESP_LOGE(TAG_MAIN, "  Failed to read INT_ENABLE (err=0x%x)", read_ret);
    // --- End Verification ---

    uint8_t clear_int;
    adxl345_read_reg(ADXL345_REG_INT_SOURCE, &clear_int);
    ESP_LOGI(TAG_MAIN, "Cleared initial interrupt states: 0x%02x", clear_int);

    ESP_LOGI(TAG_MAIN, "Configuring GPIO interrupt for pin %d...",
             ADXL345_INT1_GPIO);
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;  // Just to be explicit, this is rising edge only
    io_conf.pin_bit_mask = (1ULL << ADXL345_INT1_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;  // Enable pull-down resistor to help keep pin LOW when not triggered
    io_conf.pull_up_en = 0;    // Disable pull-up resistor
    gpio_config(&io_conf);

    esp_err_t isr_service_ret = gpio_install_isr_service(0);
    if (isr_service_ret != ESP_OK && isr_service_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG_MAIN, "Failed to install GPIO ISR service! Error: 0x%x",
                 isr_service_ret);
        return;
    }

    esp_err_t isr_hook_ret = gpio_isr_handler_add(
        ADXL345_INT1_GPIO, gpio_isr_handler, (void *)ADXL345_INT1_GPIO);
    if (isr_hook_ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to add GPIO ISR handler! Error: 0x%x",
                 isr_hook_ret);
        return;
    }
    ESP_LOGI(TAG_MAIN, "GPIO Interrupt configured.");

    ESP_LOGI(TAG_MAIN, "Creating Interrupt Handler Task...");
    xTaskCreate(adxl_interrupt_handler_task, "adxl_int_handler", 2048, NULL, 10, &adxl_task_handle);
    
    // Create the debug task with lower priority
    ESP_LOGI(TAG_MAIN, "Creating Debug Task...");
    xTaskCreate(interrupt_pin_debug_task, "int_pin_debug", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG_MAIN, "Initialization Complete. System starting in inactive state.");
    is_currently_inactive = true;
}