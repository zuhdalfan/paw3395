#ifndef ZEPHYR_INCLUDE_PAW3395_H_
#define ZEPHYR_INCLUDE_PAW3395_H_

#include "pixart.h"
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file paw3395.h
 * @brief Header file for the paw3395 driver.
 */

enum paw3395_run_mode {
    HP_MODE,      // high performance mode
    LP_MODE,      // low power mode
    OFFICE_MODE,  // office mode
    GAME_MODE,    // game mode
    RUN_MODE_COUNT
};

enum paw3395_attribute {
    PAW3395_ATTR_X_CPI = SENSOR_ATTR_PRIV_START,
    PAW3395_ATTR_Y_CPI,
    PAW3395_ATTR_CPI_ALL,
    PAW3395_ATTR_REST_ENABLE,
    PAW3395_ATTR_RUN_DOWNSHIFT_TIME,
    PAW3395_ATTR_REST1_DOWNSHIFT_TIME,
    PAW3395_ATTR_REST2_DOWNSHIFT_TIME,
    PAW3395_ATTR_REST1_SAMPLE_TIME,
    PAW3395_ATTR_REST2_SAMPLE_TIME,
    PAW3395_ATTR_REST3_SAMPLE_TIME,
    PAW3395_ATTR_RUN_MODE,
    PAW3395_ATTR_LIFT_CUTOFF,
};

typedef enum {
    PAW3395_CPI_800 = 0,
    PAW3395_CPI_1600,
    PAW3395_CPI_2400,
    PAW3395_CPI_3200,
    PAW3395_CPI_5000,
    PAW3395_CPI_10000,
    PAW3395_CPI_26000,
    PAW3395_CPI_COUNT
} paw3395_cpi_enum_t;

struct device *paw3395_get_device();

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_PAW3395_H_ */
