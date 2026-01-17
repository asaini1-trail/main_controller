#include "customcode_XK4i8iwN9zHZcP7laN2zMH.h"
#ifdef __cplusplus
extern "C" {
#endif


/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
DLL_EXPORT_CC extern const char_T *get_dll_checksum_XK4i8iwN9zHZcP7laN2zMH(void);
DLL_EXPORT_CC extern int32_T vesc_open_noargs_XK4i8iwN9zHZcP7laN2zMH(void);
DLL_EXPORT_CC extern void vesc_close_XK4i8iwN9zHZcP7laN2zMH(void);
DLL_EXPORT_CC extern int32_T vesc_send_duty_left_XK4i8iwN9zHZcP7laN2zMH(real_T duty);
DLL_EXPORT_CC extern int32_T vesc_send_duty_right_XK4i8iwN9zHZcP7laN2zMH(real_T duty);
DLL_EXPORT_CC extern int32_T vesc_get_values_left_XK4i8iwN9zHZcP7laN2zMH(uint8_T can_id, vesc_values_t *out);
DLL_EXPORT_CC extern int32_T vesc_get_values_right_XK4i8iwN9zHZcP7laN2zMH(uint8_T can_id, vesc_values_t *out);
DLL_EXPORT_CC extern int32_T vesc_get_values_all_XK4i8iwN9zHZcP7laN2zMH(vesc_values_t values[4]);
DLL_EXPORT_CC extern int32_T vesc_poll_XK4i8iwN9zHZcP7laN2zMH(real32_T *v_in, real32_T *t_mos, real32_T *t_motor);
DLL_EXPORT_CC extern void get_telemetry_wrapper_XK4i8iwN9zHZcP7laN2zMH(real32_T voltages[4], real32_T current_motor[4], real32_T current_battery[4], real32_T rpm[4], uint8_T fault_codes[4], int32_T *status);

/* Function Definitions */
DLL_EXPORT_CC const uint8_T *get_checksum_source_info(int32_T *size);
#ifdef __cplusplus
}
#endif

