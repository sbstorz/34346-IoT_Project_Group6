#include <string.h>
#include <stdbool.h> // Include for bool type

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
// driver should be included in CMakeLists.txt
#include "driver/i2c.h"

#include "adxl345.h"

#define TAG "ADXL345"

// Flag to store motion detection status (volatile because it's accessed by ISR context indirectly)
static volatile bool g_motion_detected_flag = false;
static volatile bool g_inactivity_detected_flag = false;

/**
 * @brief Writes a single byte to a specific register on the ADXL345
 */
static esp_err_t adxl345_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM, ADXL345_DEFAULT_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Reads a single byte from a specific register on the ADXL345
 */
esp_err_t adxl345_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, ADXL345_DEFAULT_ADDRESS, &reg, 1, value, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void adxl345_i2c_master_init(int16_t sda, int16_t scl)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ};
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
    // master mode only, no buffer RX/TX needed
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
}

void adxl345_set_measure_mode()
{
    uint8_t buf[2];
    esp_err_t ret;
    // check device id
    uint8_t ADXL345_DEVID[2];
    ADXL345_DEVID[0] = ADXL345_REG_DEVID & 0xFF;
    ADXL345_DEVID[1] = (ADXL345_REG_DEVID >> 8) & 0xFF;
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, ADXL345_DEFAULT_ADDRESS, &ADXL345_DEVID[0], 2, &buf[0], 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "devid=%x", buf[0]);
    }
    else
    {
        ESP_LOGE(TAG, "%s", "ESP_FAIL while reading devid");
    }
    // set ADXL345 to measure mode
    buf[0] = ADXL345_REG_POWER_CTL;
    // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable
    buf[1] = 0x08;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, ADXL345_DEFAULT_ADDRESS, &buf[0], sizeof(buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "%s", "ADXL345 set to measure mode");
    }
    else
    {
        ESP_LOGE(TAG, "%s", "ESP_FAIL while setting ADXL345 to measure mode");
    }
}

esp_err_t adxl345_config_activity_int(uint8_t threshold, adxl345_int_pin_t int_pin)
{
    esp_err_t ret;

    // 1. Set Activity Threshold
    ret = adxl345_write_reg(ADXL345_REG_THRESH_ACT, threshold);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set activity threshold (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Activity threshold set to 0x%x (%d mg)", threshold, threshold * 63); // Approx 62.5 mg/LSB

    // 2. Configure Activity/Inactivity Control
    //    - Activity: DC-coupled, X, Y, Z enabled
    //    - Preserve inactivity settings.
    uint8_t current_act_inact_ctl = 0;
    ret = adxl345_read_reg(ADXL345_REG_ACT_INACT_CTL, &current_act_inact_ctl);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read current ACT_INACT_CTL register (err=0x%x)", ret);
        return ret;
    }
    // Enable activity on X, Y, Z. Assumes DC-coupled (Bit 7 ACT_ACDC = 0, Bit 3 INACT_ACDC = 0, which is default or set by inactivity config)
    uint8_t act_inact_ctl_val = current_act_inact_ctl | ADXL345_ACT_X_EN | ADXL345_ACT_Y_EN | ADXL345_ACT_Z_EN;
    ret = adxl345_write_reg(ADXL345_REG_ACT_INACT_CTL, act_inact_ctl_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set activity control (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Activity control set (DC-coupled, XYZ enabled)");

    // 3. Map Activity Interrupt to INT1 or INT2
    //    Preserve other interrupt mappings.
    uint8_t current_int_map = 0;
    ret = adxl345_read_reg(ADXL345_REG_INT_MAP, &current_int_map);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read current INT_MAP register (err=0x%x)", ret);
        return ret;
    }

    uint8_t int_map_val = current_int_map;
    if (int_pin == ADXL345_INT2_PIN) {
        int_map_val |= ADXL345_INT_MAP_ACTIVITY;  // Route Activity to INT2
    } else {
        int_map_val &= ~ADXL345_INT_MAP_ACTIVITY; // Route Activity to INT1
    }
    ret = adxl345_write_reg(ADXL345_REG_INT_MAP, int_map_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set interrupt map (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Activity interrupt mapped to INT%d", (int_pin == ADXL345_INT1_PIN) ? 1 : 2);

    // 4. Enable Activity Interrupt
    //    Preserve other enabled interrupts.
    uint8_t current_int_enable = 0;
    ret = adxl345_read_reg(ADXL345_REG_INT_ENABLE, &current_int_enable);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read current INT_ENABLE register (err=0x%x)", ret);
        return ret;
    }
    uint8_t int_enable_val = current_int_enable | ADXL345_INT_ENABLE_ACTIVITY;
    ret = adxl345_write_reg(ADXL345_REG_INT_ENABLE, int_enable_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable activity interrupt (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Activity interrupt enabled");

    return ESP_OK;
}

esp_err_t adxl345_config_inactivity_int(uint8_t threshold, uint8_t time_s, adxl345_int_pin_t int_pin)
{
    esp_err_t ret;

    // 1. Set Inactivity Threshold
    ret = adxl345_write_reg(ADXL345_REG_THRESH_INACT, threshold);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set inactivity threshold (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inactivity threshold set to 0x%x (%d mg)", threshold, threshold * 63); // Approx 62.5 mg/LSB

    // 2. Set Inactivity Time
    ret = adxl345_write_reg(ADXL345_REG_TIME_INACT, time_s);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set inactivity time (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inactivity time set to %d seconds", time_s);

    // 3. Configure Activity/Inactivity Control to enable inactivity detection
    // Read current value to preserve activity settings
    uint8_t current_act_inact_ctl = 1;
    ret = adxl345_read_reg(ADXL345_REG_ACT_INACT_CTL, &current_act_inact_ctl);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read current ACT_INACT_CTL register (err=0x%x)", ret);
        return ret;
    }
    
    // Set Inactivity settings (keeping current activity settings)
    uint8_t act_inact_ctl_val = current_act_inact_ctl | 
                               ADXL345_INACT_X_EN | ADXL345_INACT_Y_EN | ADXL345_INACT_Z_EN;
    ret = adxl345_write_reg(ADXL345_REG_ACT_INACT_CTL, act_inact_ctl_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set inactivity control (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inactivity control set (DC-coupled, XYZ enabled)");

    // 4. Map Inactivity Interrupt to INT1 or INT2
    uint8_t current_int_map = 0;
    ret = adxl345_read_reg(ADXL345_REG_INT_MAP, &current_int_map);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read current INT_MAP register (err=0x%x)", ret);
        return ret;
    }
    
    // Update INT_MAP with inactivity mapping while preserving other mappings
    uint8_t int_map_val = current_int_map;
    if (int_pin == ADXL345_INT2_PIN) {
        int_map_val |= ADXL345_INT_MAP_INACTIVITY;  // Set bit for INT2
    } else {
        int_map_val &= ~ADXL345_INT_MAP_INACTIVITY; // Clear bit for INT1
    }
    
    ret = adxl345_write_reg(ADXL345_REG_INT_MAP, int_map_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set interrupt map (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inactivity interrupt mapped to INT%d", (int_pin == ADXL345_INT1_PIN) ? 1 : 2);

    // 5. Enable Inactivity Interrupt
    uint8_t current_int_enable = 0;
    ret = adxl345_read_reg(ADXL345_REG_INT_ENABLE, &current_int_enable);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read current INT_ENABLE register (err=0x%x)", ret);
        return ret;
    }
    
    // Update INT_ENABLE with inactivity bit while preserving other interrupts
    uint8_t int_enable_val = current_int_enable | ADXL345_INT_ENABLE_INACTIVITY;
    ret = adxl345_write_reg(ADXL345_REG_INT_ENABLE, int_enable_val);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable inactivity interrupt (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inactivity interrupt enabled");

    return ESP_OK;
}

uint8_t adxl345_process_interrupts(void)
{
    uint8_t int_source;
    esp_err_t ret = adxl345_read_reg(ADXL345_REG_INT_SOURCE, &int_source);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read interrupt source (err=0x%x)", ret);
        return 0; // Cannot confirm source if read failed
    }

    // Check for Activity interrupt
    if (int_source & ADXL345_INT_SOURCE_ACTIVITY)
    {
        ESP_LOGI(TAG, "Activity interrupt detected (INT_SOURCE: 0x%x)", int_source);
        g_motion_detected_flag = true;
        g_inactivity_detected_flag = false;
    }

    // Check for Inactivity interrupt
    if (int_source & ADXL345_INT_SOURCE_INACTIVITY)
    {
        ESP_LOGI(TAG, "Inactivity interrupt detected (INT_SOURCE: 0x%x)", int_source);
        g_inactivity_detected_flag = true;
    }

    return int_source;
}

bool adxl345_check_activity_interrupt_source(void)
{
    return adxl345_process_interrupts() & ADXL345_INT_SOURCE_ACTIVITY;
}

bool adxl345_has_motion_occurred(bool clear_flag)
{
    bool current_status = g_motion_detected_flag;
    if (clear_flag)
    {
        g_motion_detected_flag = false;
    }
    return current_status;
}

bool adxl345_is_inactive(bool clear_flag)
{
    bool current_status = g_inactivity_detected_flag;
    if (clear_flag)
    {
        g_inactivity_detected_flag = false;
    }
    return current_status;
}