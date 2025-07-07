#ifndef ZEPHYR_INCLUDE_PIXART_H_
#define ZEPHYR_INCLUDE_PIXART_H_

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pixart_data {
    const struct device *dev;
    int16_t x;
    int16_t y;
    struct k_spinlock lock;
    struct gpio_callback irq_gpio_cb;
    sensor_trigger_handler_t data_ready_handler;
    const struct sensor_trigger *trigger;
    struct k_work trigger_handler_work;
    struct k_work_delayable init_work;
    int async_init_step;
    bool ready;
    bool last_read_burst;
    int err;
    struct k_work poll_work;
    struct k_timer poll_timer;
    bool sw_smart_flag;
};

struct pixart_config {
    struct gpio_dt_spec irq_gpio;
    struct spi_dt_spec bus;
    struct gpio_dt_spec cs_gpio;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_PIXART_H_ */
