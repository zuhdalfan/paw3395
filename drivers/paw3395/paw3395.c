/*
 * PAW3395 Zephyr Driver - Complete Implementation
 *
 * Features:
 * - CPI (DPI) configuration (800, 1600, 2400, 3200, 5000, 10000, 26000)
 * - Power saving: Rest1=30s, Rest2=400s, Rest3=5000s
 * - Lift cutoff configurable
 * - Zephyr sensor API glue: sample_fetch, channel_get, attr_set, trigger_set
 * - Motion burst read
 * - No LED or unrelated peripheral code
 */
#define DT_DRV_COMPAT pixart_paw3395

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include "paw3395.h"
#include "pixart.h"
#include "paw3395_priv.h"

LOG_MODULE_REGISTER(paw3395);

#define PAW3395_REG_PRODUCT_ID 0x00
#define PAW3395_REG_POWER_UP_RESET 0x3A
#define PAW3395_REG_MOTION_BURST 0x16
#define PAW3395_REG_SET_RESOLUTION 0x47
#define PAW3395_REG_RESOLUTION_X_LOW 0x48
#define PAW3395_REG_RESOLUTION_X_HIGH 0x49
#define PAW3395_REG_RESOLUTION_Y_LOW 0x4A
#define PAW3395_REG_RESOLUTION_Y_HIGH 0x4B
#define PAW3395_REG_LIFT_CONFIG_H 0x0C
#define PAW3395_REG_LIFT_CONFIG_L 0x4E
#define PAW3395_REG_RUN_DOWNSHIFT 0x77
#define PAW3395_REG_REST1_DOWNSHIFT 0x79
#define PAW3395_REG_REST2_DOWNSHIFT 0x7B
#define PAW3395_REG_REST3_DOWNSHIFT 0x7C
#define PAW3395_PRODUCT_ID 0x51
#define SPI_WRITE_BIT 0x80
#define PAW3395_BURST_SIZE 6
#define PAW3395_MOTION 0 // motion byte
#define PAW3395_DX_POS 2 // dx byte
#define PAW3395_DY_POS 4 // dx byte

#define CPI_TO_REG(cpi) (((cpi) / 50) - 1)

// Power saving times (ms)
#define PAW3395_REST1_DOWNSHIFT_MS 30000
#define PAW3395_REST2_DOWNSHIFT_MS 400000
#define PAW3395_REST3_DOWNSHIFT_MS 5000000

static const uint32_t paw3395_cpi_choices[] = {
    800, 1600, 2400, 3200, 5000, 10000, 26000
};

// In pixart.h or paw3395.h
#define paw3395_config pixart_config

const struct device *paw3395 = DEVICE_DT_GET_ONE(pixart_paw3395);

struct paw3395_data {
    struct pixart_data base;
    int16_t x;
    int16_t y;
    bool ready;
};

static int paw3395_spi_write(const struct device *dev, uint8_t reg, uint8_t val) {
    const struct pixart_config *cfg = dev->config;
    uint8_t buf[2] = {reg | SPI_WRITE_BIT, val};
    struct spi_buf tx_buf = {.buf = buf, .len = 2};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
    return spi_write_dt(&cfg->bus, &tx);
}

static int paw3395_spi_read(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct pixart_config *cfg = dev->config;
    struct spi_buf tx_buf = {.buf = &reg, .len = 1};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
    int err = spi_write_dt(&cfg->bus, &tx);
    if (err) return err;
    struct spi_buf rx_buf = {.buf = val, .len = 1};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};
    return spi_read_dt(&cfg->bus, &rx);
}

// paw3395 spi transcieve function - use spi_transceive_dt()
static int paw3395_spi_transceive(const struct device *dev, const uint8_t *tx_buf, size_t tx_len, uint8_t *rx_buf, size_t rx_len) {
    const struct pixart_config *cfg = dev->config;
    struct spi_buf tx_spi_buf = { .buf = (void *)tx_buf, .len = tx_len };
    struct spi_buf_set tx_set = { .buffers = &tx_spi_buf, .count = 1 };
    struct spi_buf rx_spi_buf = { .buf = rx_buf, .len = rx_len };
    struct spi_buf_set rx_set = { .buffers = &rx_spi_buf, .count = 1 };
    return spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
}

// Set the rest period for a given rest mode (1, 2, or 3)
// period_ms: desired period in ms (see datasheet for valid range per mode)
static int paw3395_set_rest_period(const struct device *dev, uint8_t rest_mode, uint16_t period_ms) {
    uint8_t reg = 0;
    uint8_t val = 0;

    switch (rest_mode) {
        case 1:
            reg = 0x78; // REST1_PERIOD
            val = period_ms; // 1ms units, min 1, max 255
            if (val == 0) val = 1;
            break;
        case 2:
            reg = 0x7A; // REST2_PERIOD
            val = period_ms / 4; // 4ms units, min 1, max 255
            if (val == 0) val = 1;
            break;
        case 3:
            reg = 0x7C; // REST3_PERIOD
            val = period_ms / 8; // 8ms units, min 1, max 255
            if (val == 0) val = 1;
            break;
        default:
            return -EINVAL;
    }
    return paw3395_spi_write(dev, reg, val);
}

// Get the rest period (in ms) for a given rest mode (1, 2, or 3)
static int paw3395_get_rest_period(const struct device *dev, uint8_t rest_mode, uint16_t *period_ms) {
    uint8_t reg = 0;
    uint8_t val = 0;
    int err = 0;

    switch (rest_mode) {
        case 1:
            reg = 0x78; // REST1_PERIOD
            err = paw3395_spi_read(dev, reg, &val);
            if (err) return err;
            *period_ms = val * 1; // 1ms units
            break;
        case 2:
            reg = 0x7A; // REST2_PERIOD
            err = paw3395_spi_read(dev, reg, &val);
            if (err) return err;
            *period_ms = val * 4; // 4ms units
            break;
        case 3:
            reg = 0x7C; // REST3_PERIOD
            err = paw3395_spi_read(dev, reg, &val);
            if (err) return err;
            *period_ms = val * 8; // 8ms units
            break;
        default:
            return -EINVAL;
    }
    return 0;
}

// Helper functions to calculate register values for desired rest downshift times

// Calculate REST1_DOWNSHIFT register value
static uint8_t paw3395_calc_rest1_downshift(uint32_t time_ms, uint8_t rest1_period_ms) {
    // Formula: value = time_ms / (64 * rest1_period_ms)
    uint32_t val = time_ms / (64 * rest1_period_ms);
    if (val == 0) val = 1;
    if (val > 255) val = 255;
    return (uint8_t)val;
}

// Calculate REST2_DOWNSHIFT register value
static uint8_t paw3395_calc_rest2_downshift(uint32_t time_ms, uint16_t rest2_period_ms) {
    // Formula: value = time_ms / (64 * rest2_period_ms)
    uint32_t val = time_ms / (64 * rest2_period_ms);
    if (val == 0) val = 1;
    if (val > 255) val = 255;
    return (uint8_t)val;
}

// Calculate REST3_DOWNSHIFT register value
static uint8_t paw3395_calc_rest3_downshift(uint32_t time_ms, uint16_t rest3_period_ms) {
    // Formula: value = time_ms / (64 * rest3_period_ms)
    uint32_t val = time_ms / (64 * rest3_period_ms);
    if (val == 0) val = 1;
    if (val > 255) val = 255;
    return (uint8_t)val;
}

static int paw3395_set_cpi(const struct device *dev, uint32_t cpi, bool axis_x) {
    if (cpi < 50 || cpi > 26000) return -EINVAL;
    uint16_t regval = CPI_TO_REG(cpi);
    uint8_t buf[2];
    sys_put_le16(regval, buf);
    uint8_t addr[2] = {axis_x ? PAW3395_REG_RESOLUTION_X_LOW : PAW3395_REG_RESOLUTION_Y_LOW,
                       axis_x ? PAW3395_REG_RESOLUTION_X_HIGH : PAW3395_REG_RESOLUTION_Y_HIGH};
    for (int i = 0; i < 2; ++i) {
        int err = paw3395_spi_write(dev, addr[i], buf[i]);
        if (err) return err;
    }
    return paw3395_spi_write(dev, PAW3395_REG_SET_RESOLUTION, 0x01);
}

static int paw3395_set_cpi_enum(const struct device *dev, paw3395_cpi_enum_t cpi_choice, bool axis_x) {
    if (cpi_choice < 0 || cpi_choice >= PAW3395_CPI_COUNT)
        return -EINVAL;
    return paw3395_set_cpi(dev, paw3395_cpi_choices[cpi_choice], axis_x);
}

static int paw3395_set_cpi_all(const struct device *dev, paw3395_cpi_enum_t cpi) {
    int err = paw3395_set_cpi_enum(dev, cpi, true);
    if (err) return err;
    return paw3395_set_cpi_enum(dev, cpi, false);
}

static int paw3395_set_lift_cutoff(const struct device *dev, uint8_t value) {
    int err = paw3395_spi_write(dev, PAW3395_REG_LIFT_CONFIG_H, value);
    if (err) return err;
    return paw3395_spi_write(dev, PAW3395_REG_LIFT_CONFIG_L, value);
}

static int paw3395_set_power_saving(const struct device *dev) {
    int err = 0;

    // set rest period 1 to 5ms
    err |= paw3395_set_rest_period(dev, 1, 5);

    // get all rest periods to calculate downshift values
    uint16_t rest1_period_ms = 1;
    uint16_t rest2_period_ms = 1;
    uint16_t rest3_period_ms = 1;

    err |= paw3395_get_rest_period(dev, 1, &rest1_period_ms);
    err |= paw3395_get_rest_period(dev, 2, &rest2_period_ms);
    err |= paw3395_get_rest_period(dev, 3, &rest3_period_ms);
    if (err) return err;

    // handle rest period division by zero
    if (rest1_period_ms == 0 || rest2_period_ms == 0 ||
        rest3_period_ms == 0) {
        LOG_ERR("Invalid rest period: %d, %d, %d", rest1_period_ms,
                rest2_period_ms, rest3_period_ms);
        return -EINVAL;
    }

    // set rest1 downshift time to 30s
    uint8_t rest1_downshift = paw3395_calc_rest1_downshift(PAW3395_REST1_DOWNSHIFT_MS, rest1_period_ms);
    err = paw3395_spi_write(dev, PAW3395_REG_REST1_DOWNSHIFT, rest1_downshift);
    if (err) return err;

    // set rest2 downshift time to 400s
    uint8_t rest2_downshift = paw3395_calc_rest2_downshift(PAW3395_REST2_DOWNSHIFT_MS, rest2_period_ms);
    err = paw3395_spi_write(dev, PAW3395_REG_REST2_DOWNSHIFT, rest2_downshift);
    if (err) return err;

    // set rest3 downshift time to 5000s
    uint8_t rest3_downshift = paw3395_calc_rest3_downshift(PAW3395_REST3_DOWNSHIFT_MS, rest3_period_ms);
    err = paw3395_spi_write(dev, PAW3395_REG_REST3_DOWNSHIFT, rest3_downshift);
    if (err) return err;

    return err;
}

static int paw3395_motion_burst(const struct device *dev, uint8_t *buf, size_t len) {
    uint8_t reg = PAW3395_REG_MOTION_BURST;
    // Use the transceive function: send reg, receive burst data
    return paw3395_spi_transceive(dev, &reg, 1, buf, len);
}

static int paw3395_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct paw3395_data *data = dev->data;
    uint8_t buf[PAW3395_BURST_SIZE];
    if (chan != SENSOR_CHAN_ALL) return -ENOTSUP;
    if (!data->ready) return -EBUSY;
    int err = paw3395_motion_burst(dev, buf, sizeof(buf));
    if (err) return err;
    // check if motion byte 1, if 0 then set the dx and dx into 0
    if (buf[PAW3395_MOTION] == 0x01){
        data->x = (int16_t)sys_get_le16(&buf[PAW3395_DX_POS]);
        data->y = (int16_t)sys_get_le16(&buf[PAW3395_DY_POS]);
    }else{
        data->x = 0;
        data->y = 0;
    }
    return 0;
}

static int paw3395_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
    struct paw3395_data *data = dev->data;
    if (!data->ready) return -EBUSY;
    switch (chan) {
        case SENSOR_CHAN_POS_DX:
            val->val1 = data->x;
            val->val2 = 0;
            break;
        case SENSOR_CHAN_POS_DY:
            val->val1 = data->y;
            val->val2 = 0;
            break;
        default:
            return -ENOTSUP;
    }
    return 0;
}

static int paw3395_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val) {
    if (chan != SENSOR_CHAN_ALL) return -ENOTSUP;
    switch ((uint32_t)attr) {
        case PAW3395_ATTR_X_CPI:
            return paw3395_set_cpi(dev, val->val1, true);
        case PAW3395_ATTR_Y_CPI:
            return paw3395_set_cpi(dev, val->val1, false);
        case PAW3395_ATTR_CPI_ALL:
            return paw3395_set_cpi_all(dev, (paw3395_cpi_enum_t)val->val1);
        case PAW3395_ATTR_REST1_DOWNSHIFT_TIME:
            return paw3395_spi_write(dev, PAW3395_REG_REST1_DOWNSHIFT, val->val1 / 1000);
        case PAW3395_ATTR_REST2_DOWNSHIFT_TIME:
            return paw3395_spi_write(dev, PAW3395_REG_REST2_DOWNSHIFT, val->val1 / 1000);
        case PAW3395_ATTR_REST3_SAMPLE_TIME:
            return paw3395_spi_write(dev, PAW3395_REG_REST3_DOWNSHIFT, val->val1 / 1000);
        case PAW3395_ATTR_RUN_DOWNSHIFT_TIME:
            return paw3395_spi_write(dev, PAW3395_REG_RUN_DOWNSHIFT, val->val1 / 1000);
        case PAW3395_ATTR_LIFT_CUTOFF:
            return paw3395_set_lift_cutoff(dev, val->val1);
        default:
            return -ENOTSUP;
    }
}

// IRQ handler and trigger support for high-performance, low-latency operation
static void paw3395_irq_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct paw3395_data *data = CONTAINER_OF(cb, struct paw3395_data, base.irq_gpio_cb);
    if (data->base.data_ready_handler) {
        data->base.data_ready_handler(data->base.dev, data->base.trigger);
    }
}

static int paw3395_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler) {
    struct paw3395_data *data = dev->data;
    const struct pixart_config *cfg = dev->config;
    int err;
    if (trig->type != SENSOR_TRIG_DATA_READY) return -ENOTSUP;
    if (!device_is_ready(cfg->irq_gpio.port)) return -ENODEV;
    gpio_remove_callback(cfg->irq_gpio.port, &data->base.irq_gpio_cb);
    if (handler) {
        data->base.data_ready_handler = handler;
        data->base.trigger = trig;
        gpio_add_callback(cfg->irq_gpio.port, &data->base.irq_gpio_cb);
        err = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (err) return err;
    } else {
        data->base.data_ready_handler = NULL;
        data->base.trigger = NULL;
        err = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
        if (err) return err;
    }
    return 0;
}

static int paw3395_init(const struct device *dev) {
    struct paw3395_data *data = dev->data;
    const struct pixart_config *cfg = dev->config;
    data->ready = false;
    data->base.dev = dev;
    LOG_DBG("Initializing PAW3395 driver");
    // Configure IRQ pin
    if (!device_is_ready(cfg->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO port not ready");
        return -ENODEV;
    }
    int err = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Failed to configure IRQ GPIO pin: %d", err);
        return err;
    }
    gpio_init_callback(&data->base.irq_gpio_cb, paw3395_irq_callback, BIT(cfg->irq_gpio.pin));
    // Drive NCS high, and then low to reset the SPI port.
    k_msleep(50);
    // Power-up reset
    k_msleep(5);
    paw3395_spi_write(dev, PAW3395_REG_POWER_UP_RESET, 0x5A);
    k_msleep(5);
    // Upload power-up register settings
    LOG_DBG("Uploading power-up register settings (part 1)");
    for (size_t i = 0; i < paw3395_pwrup_registers_length1; ++i){
        paw3395_spi_write(dev, paw3395_pwrup_registers_addr1[i], paw3395_pwrup_registers_data1[i]);
    }
    k_msleep(1);

    uint8_t reg_6c = 0;
    for (int i = 0; i < 60; ++i) {
        paw3395_spi_read(dev, 0x6C, &reg_6c);
        if (reg_6c == 0x80) break;
        k_msleep(1);
    }
    if (reg_6c != 0x80) {
        LOG_WRN("Register 0x6C did not reach 0x80, performing workaround");
        paw3395_spi_write(dev, 0x7F, 0x14);
        paw3395_spi_write(dev, 0x6C, 0x00);
        paw3395_spi_write(dev, 0x7F, 0x00);
    }
    paw3395_spi_write(dev, 0x22, 0x00);
    paw3395_spi_write(dev, 0x55, 0x00);
    paw3395_spi_write(dev, 0x7F, 0x07);
    paw3395_spi_write(dev, 0x40, 0x40);
    paw3395_spi_write(dev, 0x7F, 0x00);
    paw3395_spi_write(dev, 0x68, 0x01);

    LOG_DBG("Uploading power-up register settings (part 2)");
    for (size_t i = 0; i < paw3395_pwrup_registers_length2; ++i){
        paw3395_spi_write(dev, paw3395_pwrup_registers_addr2[i], paw3395_pwrup_registers_data2[i]);
    }
    // Check product ID
    uint8_t prod_id = 0;
    paw3395_spi_read(dev, PAW3395_REG_PRODUCT_ID, &prod_id);
    LOG_DBG("PAW3395 product ID: 0x%02X", prod_id);
    if (prod_id != PAW3395_PRODUCT_ID) {
        LOG_ERR("Product ID mismatch: expected 0x%02X, got 0x%02X", PAW3395_PRODUCT_ID, prod_id);
        // return -ENODEV;
    }
    // Set default CPI, power saving, etc.
    LOG_DBG("Setting default CPI and power saving");
    paw3395_set_cpi_all(dev, PAW3395_CPI_1600); // Default CPI
    paw3395_set_power_saving(dev);
    data->ready = true;
    LOG_INF("PAW3395 initialization complete");
    return 0;
}

static const struct sensor_driver_api paw3395_api = {
    .sample_fetch = paw3395_sample_fetch,
    .channel_get = paw3395_channel_get,
    .attr_set = paw3395_attr_set,
    .trigger_set = paw3395_trigger_set,
};

const struct device *paw3395_get_device()
{
    return paw3395;
}

// Expansion macro to define driver instances
#define PAW3395_DEFINE(inst)                                                 \
    static const struct pixart_config paw3395_config_##inst = {             \
        .bus = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0),           \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                 \
    };                                                                       \
                                                                             \
    static struct paw3395_data paw3395_data_##inst;                          \
                                                                             \
    DEVICE_DT_INST_DEFINE(inst,                                              \
                          paw3395_init,                                      \
                          NULL,                                              \
                          &paw3395_data_##inst,                              \
                          &paw3395_config_##inst,                            \
                          POST_KERNEL,                                       \
                          CONFIG_PAW3395_INIT_PRIORITY,                      \
                          &paw3395_api);

// Instantiate for all devices in the Devicetree with compatible = "pixart,paw3395"
DT_INST_FOREACH_STATUS_OKAY(PAW3395_DEFINE)



