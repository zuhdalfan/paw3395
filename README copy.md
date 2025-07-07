# PAW3395 Zephyr Driver (High-Performance Mouse Edition)

This driver provides a robust, high-performance interface for the PixArt PAW3395 optical mouse sensor, fully integrated with Zephyr RTOS. It is designed for use in demanding mouse applications, including gaming and wireless mice, with a focus on low latency, high responsiveness, and power efficiency.

## Features
- **Interrupt-driven (IRQ) motion detection** for lowest latency and best power efficiency
- **Full Zephyr sensor API support**: fetch, channel_get, attr_set, trigger_set
- **Motion burst read** for atomic, high-speed data fetch
- **Configurable CPI (DPI), lift cutoff, and power saving** at runtime
- **Automatic power-up register upload and product ID check**
- **Device tree integration** for easy hardware configuration
- **No LED or unrelated peripheral code** (use your own LED driver)

## How It Works
- On boot, the driver resets the sensor, uploads all required power-up registers, and checks the product ID.
- Default CPI (DPI) and power saving settings are applied.
- The driver configures the IRQ pin for motion interrupts.
- When motion is detected, the IRQ handler triggers a Zephyr sensor event, allowing your application to fetch new data immediately.
- All configuration (CPI, lift cutoff, power saving) can be changed at runtime using the Zephyr sensor API.

## Usage Example

### Device Tree (DTS)
```dts
&spi1 {
    status = "okay";
    paw3395@0 {
        compatible = "pixart,paw3395";
        reg = <0>;
        spi-max-frequency = <2000000>;
        cs-gpios = <&gpio0 15 GPIO_ACTIVE_LOW>;
        irq-gpios = <&gpio0 14 GPIO_ACTIVE_HIGH>;
        label = "PAW3395";
    };
};
```

### Application Code
```c
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

const struct device *paw3395 = DEVICE_DT_GET_ONE(pixart_paw3395);

static void motion_handler(const struct device *dev, const struct sensor_trigger *trig) {
    struct sensor_value dx, dy;
    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_POS_DX, &dx);
    sensor_channel_get(dev, SENSOR_CHAN_POS_DY, &dy);
    printk("Motion: dx=%d, dy=%d\n", dx.val1, dy.val1);
}

void main(void) {
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };
    sensor_trigger_set(paw3395, &trig, motion_handler);
    while (1) {
        k_sleep(K_SECONDS(1)); // Main thread can sleep, motion is handled by IRQ
    }
}
```

### Changing CPI or Lift Cutoff at Runtime
```c
struct sensor_value cpi = { .val1 = 3200 };
sensor_attr_set(paw3395, SENSOR_CHAN_ALL, PAW3395_ATTR_X_CPI, &cpi);

struct sensor_value lift = { .val1 = 5 };
sensor_attr_set(paw3395, SENSOR_CHAN_ALL, PAW3395_ATTR_LIFT_CUTOFF, &lift);
```

## Performance Tips
- Use the highest SPI frequency supported by your hardware (2MHz recommended).
- For wireless/BLE, let the MCU sleep between motion events; the driver will wake it on IRQ.
- For USB, match your HID report rate to the sensor's IRQ rate for best responsiveness.
- All register tables and power-up sequences are handled automatically by the driver.

## Advanced Configuration
- You can tune power saving, CPI, and lift cutoff at runtime using the Zephyr sensor API.
- The driver supports all four run modes (HP, LP, Office, Game) via register tables.
- For custom features (debounce, smoothing, orientation), extend the driver or use Zephyr's sensor API hooks.

## Limitations
- This driver does not handle LED indication or button input—use separate drivers for those features.
- Orientation and advanced filtering are not included by default but can be added as needed.

## License
MIT

## Credits
Based on [Krillleeee/paw3395-module](https://github.com/Krillleeee/paw3395-module) and Zephyr RTOS best practices.