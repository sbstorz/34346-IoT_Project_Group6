#include <stdbool.h>

#include "esp_log.h"
// driver should be included in CMakeLists.txt
#include "adxl345.h"
#include "driver/i2c_master.h" // New I2C master driver

#define TAG "ADXL345"

static i2c_master_dev_handle_t adxl345_dev_handle = NULL; // Handle for the ADXL345 I2C device
static adxl_device_config_t _config;

/**
 * @brief Writes a single byte to a specific register on the ADXL345
 */
static esp_err_t adxl345_write_reg(uint8_t reg, uint8_t value)
{
    if (adxl345_dev_handle == NULL)
    {
        ESP_LOGE(TAG, "ADXL345 device handle not initialized");
        return ESP_FAIL;
    }
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(adxl345_dev_handle, write_buf, sizeof(write_buf),
                               I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief Reads a single byte from a specific register on the ADXL345
 */
esp_err_t adxl345_read_reg(uint8_t reg, uint8_t *value)
{
    if (adxl345_dev_handle == NULL)
    {
        ESP_LOGE(TAG, "ADXL345 device handle not initialized");
        return ESP_FAIL;
    }

    for (size_t i = 0; i < 3; i++)
    {
        if (i2c_master_transmit_receive(adxl345_dev_handle, &reg, 1, value, 1,
                                        I2C_MASTER_TIMEOUT_MS) == ESP_OK)
        {
            return ESP_OK;
        }
    }

    return ESP_FAIL;
}

// Helper function to perform a read-modify-write operation on a register
static esp_err_t adxl345_update_reg_bits(uint8_t reg, uint8_t bits_to_set,
                                         uint8_t bits_to_clear)
{
    uint8_t current_value;
    esp_err_t ret = adxl345_read_reg(reg, &current_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read register 0x%02x for update (err=0x%x)",
                 reg, ret);
        return ret;
    }
    uint8_t new_value = (current_value & ~bits_to_clear) | bits_to_set;
    if (new_value != current_value)
    { // Only write if value changed
        ret = adxl345_write_reg(reg, new_value);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG,
                     "Failed to write register 0x%02x during update (err=0x%x)",
                     reg, ret);
        }
    }
    else
    {
        // ESP_LOGD(TAG, "Register 0x%02x update: No change needed
        // (value=0x%02x)", reg, current_value);
    }
    return ret;
}

// Reads and returns the interrupt source register, clearing interrupts on the
// device. Returns the value read, or 0xFF on error (as 0x00 is a valid source
// value).
uint8_t adxl345_get_and_clear_int_source(void)
{
    uint8_t int_source_val = 0;
    esp_err_t ret = adxl345_read_reg(ADXL345_REG_INT_SOURCE, &int_source_val);
    if (ret == ESP_OK)
    {
        return int_source_val;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read INT_SOURCE register. Error: 0x%x", ret);
        return 0xFF; // Return error indicator
    }
}

esp_err_t adxl345_init(const adxl_device_config_t *config)
{
    _config = *config;
    esp_err_t rc = adxl345_i2c_master_init(config->sda_pin, config->scl_pin);
    if (rc != ESP_OK)
    {
        return rc;
    }

    // Make sure measurement mode is enabled, also serves to check if we can communicate
    rc = adxl345_set_measure_mode();
    if (rc != ESP_OK)
    {
        return rc;
    }

    return ESP_OK;
}

esp_err_t adxl345_i2c_master_init(gpio_num_t sda, gpio_num_t scl)
{
    if (adxl345_dev_handle != NULL)
    {
        ESP_LOGW(TAG, "ADXL345 I2C already initialized.");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing I2C master for ADXL345...");

    i2c_master_bus_config_t i2c_mst_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM, // ESP-IDF I2C port number (0 or 1)
        .sda_io_num = sda,
        .scl_io_num = scl,
        .glitch_ignore_cnt = 7, // Default from examples, can be tuned
        .flags.enable_internal_pullup =
            true, // To enable internal pullups for SDA and SCL
    };
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&i2c_mst_bus_config, &bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus (err=0x%x)", ret);
        return ESP_FAIL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADXL345_DEFAULT_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &adxl345_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add ADXL345 device to I2C bus (err=0x%x)",
                 ret);
        // Clean up bus if device add fails
        i2c_del_master_bus(bus_handle);
        adxl345_dev_handle = NULL;
        return ESP_FAIL;
        ;
    }
    ESP_LOGI(TAG, "ADXL345 I2C master and device initialized successfully.");

    return ESP_OK;
}

esp_err_t adxl345_set_measure_mode()
{
    if (adxl345_dev_handle == NULL)
    {
        ESP_LOGE(TAG,
                 "ADXL345 device handle not initialized for set_measure_mode");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t dev_id;
    esp_err_t ret;

    // Check device id
    ret = adxl345_read_reg(ADXL345_REG_DEVID, &dev_id);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "ADXL345 Device ID: 0x%02x", dev_id);
        if (dev_id != 0xE5)
        { // Expected device ID for ADXL345
            ESP_LOGW(TAG, "Unexpected Device ID: 0x%02x (Expected 0xE5)",
                     dev_id);
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read ADXL345 Device ID (err=0x%x)", ret);
        return ESP_FAIL;
        // It might be problematic to continue if we can't read the device ID
    }

    // Set ADXL345 to measure mode
    // Bit D3 High (0x08) in POWER_CTL register (0x2D) enables measurement
    ret = adxl345_write_reg(ADXL345_REG_POWER_CTL, 0x08);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "ADXL345 set to measure mode");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to set ADXL345 to measure mode (err=0x%x)", ret);
        return ESP_FAIL;
    }

    return ESP_OK;
}

// Refactored function
esp_err_t adxl345_enable_activity_int(void)
{
    esp_err_t ret;

    // 1. Set Activity Threshold
    ret = adxl345_write_reg(ADXL345_REG_THRESH_ACT, _config.activity_threshold);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set activity threshold (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Activity threshold set to 0x%x (%d mg)", _config.activity_threshold,
             _config.activity_threshold * 63);

    // 2. Map Activity Interrupt to INT1 or INT2
    uint8_t map_bits_to_set = 0;
    uint8_t map_bits_to_clear = 0;
    if (_config.activity_int_pin == ADXL345_INT2_PIN)
    {
        map_bits_to_set = ADXL345_INT_MAP_ACTIVITY; // Route Activity to INT2
    }
    else
    {
        map_bits_to_clear =
            ADXL345_INT_MAP_ACTIVITY; // Route Activity to INT1 (clear bit)
    }
    ret = adxl345_update_reg_bits(ADXL345_REG_INT_MAP, map_bits_to_set,
                                  map_bits_to_clear);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set interrupt map for activity (err=0x%x)",
                 ret);
        return ret;
    }
    ESP_LOGI(TAG, "Activity interrupt mapped to INT%d",
             (_config.activity_int_pin == ADXL345_INT1_PIN) ? 1 : 2);

    // 4. Configure Activity Control (DC-coupled, XYZ enabled)
    uint8_t act_ctl_bits_to_set =
        ADXL345_ACT_X_EN | ADXL345_ACT_Y_EN | ADXL345_ACT_Z_EN;
    uint8_t act_ctl_bits_to_clear = ADXL345_ACT_ACDC; // Ensure DC coupling
    ret = adxl345_update_reg_bits(ADXL345_REG_ACT_INACT_CTL,
                                  act_ctl_bits_to_set, act_ctl_bits_to_clear);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Failed to set activity control in ACT_INACT_CTL (err=0x%x)",
                 ret);
        return ret;
    }
    ESP_LOGI(TAG,
             "Activity control set in ACT_INACT_CTL (DC-coupled, XYZ enabled)");

    // 3. Enable Activity Interrupt
    ret = adxl345_update_reg_bits(ADXL345_REG_INT_ENABLE,
                                  ADXL345_INT_ENABLE_ACTIVITY, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable activity interrupt (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Activity interrupt enabled in INT_ENABLE register");

    return ESP_OK;
}

esp_err_t adxl345_enable_inactivity_int(void)
{
    esp_err_t ret;

    // 1. Set Inactivity Threshold
    ret = adxl345_write_reg(ADXL345_REG_THRESH_INACT, _config.inactivity_threshold);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set inactivity threshold (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inactivity threshold set to 0x%x (%d mg)", _config.inactivity_threshold,
             _config.inactivity_threshold * 63);

    // 2. Set Inactivity Time
    ret = adxl345_write_reg(ADXL345_REG_TIME_INACT, _config.inactivity_time_s);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set inactivity time (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inactivity time set to %d seconds", _config.inactivity_time_s);

    // 3. Map Inactivity Interrupt to INT1 or INT2
    uint8_t map_bits_to_set = 0;
    uint8_t map_bits_to_clear = 0;
    if (_config.inactivity_int_pin == ADXL345_INT2_PIN)
    {
        map_bits_to_set =
            ADXL345_INT_MAP_INACTIVITY; // Route Inactivity to INT2
    }
    else
    {
        map_bits_to_clear =
            ADXL345_INT_MAP_INACTIVITY; // Route Inactivity to INT1 (clear bit)
    }
    ret = adxl345_update_reg_bits(ADXL345_REG_INT_MAP,
                                  map_bits_to_set,
                                  map_bits_to_clear);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set interrupt map for inactivity (err=0x%x)",
                 ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inactivity interrupt mapped to INT%d",
             (_config.inactivity_int_pin == ADXL345_INT1_PIN) ? 1 : 2);

    // 5. Configure Inactivity Control (DC-coupled, XYZ enabled)
    uint8_t inact_ctl_bits_to_set =
        ADXL345_INACT_X_EN | ADXL345_INACT_Y_EN | ADXL345_INACT_Z_EN;
    uint8_t inact_ctl_bits_to_clear = ADXL345_INACT_ACDC; // Ensure DC coupling
    ret = adxl345_update_reg_bits(ADXL345_REG_ACT_INACT_CTL,
                                  inact_ctl_bits_to_set,
                                  inact_ctl_bits_to_clear);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Failed to set inactivity control in ACT_INACT_CTL (err=0x%x)",
                 ret);
        return ret;
    }
    ESP_LOGI(
        TAG,
        "Inactivity control set in ACT_INACT_CTL (DC-coupled, XYZ enabled)");

    // 4. Enable Inactivity Interrupt
    ret = adxl345_update_reg_bits(ADXL345_REG_INT_ENABLE,
                                  ADXL345_INT_ENABLE_INACTIVITY, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable inactivity interrupt (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inactivity interrupt enabled in INT_ENABLE register");

    return ESP_OK;
}

esp_err_t adxl345_disable_inactivity_int(void)
{
    esp_err_t ret;

    ret = adxl345_update_reg_bits(ADXL345_REG_INT_ENABLE,
                                  0, ADXL345_INT_ENABLE_INACTIVITY);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to disable inactivity interrupt (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inactivity interrupt disabled in INT_ENABLE register");

    return ESP_OK;
}

esp_err_t adxl345_disable_activity_int(void)
{
    esp_err_t ret;

    ret = adxl345_update_reg_bits(ADXL345_REG_INT_ENABLE,
                                  0, ADXL345_INT_ENABLE_ACTIVITY);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to disnable activity interrupt (err=0x%x)", ret);
        return ret;
    }
    ESP_LOGI(TAG, "activity interrupt disabled in INT_ENABLE register");

    return ESP_OK;
}