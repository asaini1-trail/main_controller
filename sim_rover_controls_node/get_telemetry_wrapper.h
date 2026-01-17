#ifndef GET_TELEMETRY_WRAPPER_H
#define GET_TELEMETRY_WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Telemetry wrapper function for Simulink */
void get_telemetry_wrapper(
    float voltages[4],
    float current_motor[4],
    float current_battery[4],
    float rpm[4],
    uint8_t fault_codes[4],
    int32_t *status
);

#ifdef __cplusplus
}
#endif

#endif /* GET_TELEMETRY_WRAPPER_H */
