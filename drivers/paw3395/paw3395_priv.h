#ifndef PAW3395_PRIV_H_
#define PAW3395_PRIV_H_

#include <zephyr/types.h>
#include <stddef.h>

// Power-up register settings (to be filled as per datasheet or reference)
extern const size_t paw3395_pwrup_registers_length1;
extern const uint8_t paw3395_pwrup_registers_addr1[];
extern const uint8_t paw3395_pwrup_registers_data1[];
extern const size_t paw3395_pwrup_registers_length2;
extern const uint8_t paw3395_pwrup_registers_addr2[];
extern const uint8_t paw3395_pwrup_registers_data2[];

// Run mode register settings
extern const size_t paw3395_mode_registers_length[];
extern const uint8_t* paw3395_mode_registers_addr[];
extern const uint8_t* paw3395_mode_registers_data[];

#endif // PAW3395_PRIV_H_
