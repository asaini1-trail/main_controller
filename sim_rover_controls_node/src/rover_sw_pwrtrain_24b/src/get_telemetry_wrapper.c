#include "get_telemetry_wrapper.h"
#include "vesc_uart.h"

/*
 * get_telemetry_wrapper
 *
 * IMPORTANT DESIGN DECISION:
 * -------------------------
 * - We NEVER zero outputs on partial failure.
 * - Stale data is safer than lying zeros.
 * - status reports success/failure of the telemetry fetch.
 */

void get_telemetry_wrapper(float *voltages,
                           float *current_motor,
                           float *current_battery,
                           float *rpm,
                           uint8_t *fault_codes,
                           int32_t *status)
{
    vesc_values_t values[4];
    int i;

    /* Fetch telemetry from all VESCs */
    int ret = vesc_get_values_all(values);

    /* Return status to MATLAB/Simulink */
    *status = (int32_t)ret;

    /* Always copy values (even if ret != 0) */
    for (i = 0; i < 4; i++) {
        voltages[i]        = values[i].v_in;
        current_motor[i]   = values[i].current_motor;
        current_battery[i] = values[i].current_in;
        rpm[i]             = values[i].rpm;
        fault_codes[i]     = values[i].fault_code;
    }
}