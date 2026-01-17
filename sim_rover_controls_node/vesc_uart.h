#ifndef VESC_UART_H
#define VESC_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float   current_motor;   // A
    float   current_in;      // A (battery side)
    float   v_in;            // V
    float   rpm;             // erpm
    uint8_t fault_code;      // VESC fault code enum
} vesc_values_t;

/* Open both LEFT and RIGHT VESC UART ports with default settings.
 * Returns 0 on success, <0 on error.
 */
int vesc_open_noargs(void);

/* Close both UART ports. */
void vesc_close(void);

/* Send duty cycle to LEFT master and all LEFT CAN slaves.
 * duty in [-1.0, 1.0], internally clamped to [-0.95, 0.95].
 * Returns 0 on success, <0 on error.
 */
int vesc_send_duty_left(double duty);

/* Send duty cycle to RIGHT master and all RIGHT CAN slaves.
 * duty in [-1.0, 1.0], internally clamped to [-0.95, 0.95].
 * Returns 0 on success, <0 on error.
 */
int vesc_send_duty_right(double duty);

/* Get values from a LEFT-side VESC.
 *  can_id == 0   → LEFT master (no CAN forwarding)
 *  can_id != 0   → query that CAN ID via COMM_FORWARD_CAN on LEFT bus
 *
 * Fills vesc_values_t with:
 *   current_motor, current_in, v_in, rpm, fault_code.
 * Returns 0 on success, <0 on error.
 */
int vesc_get_values_left(uint8_t can_id, vesc_values_t *out);

/* Get values from a RIGHT-side VESC.
 *  can_id == 0   → RIGHT master
 *  can_id != 0   → query that CAN ID via COMM_FORWARD_CAN on RIGHT bus
 *
 * Returns 0 on success, <0 on error.
 */
int vesc_get_values_right(uint8_t can_id, vesc_values_t *out);

/* Convenience: get values from 4 motors in fixed order:
 *   0: LEFT master
 *   1: first LEFT slave  (VESC_CAN_LEFT_SLAVES[0])
 *   2: RIGHT master
 *   3: first RIGHT slave (VESC_CAN_RIGHT_SLAVES[0])
 *
 * Returns 0 on success, <0 on error for any failure.
 */
int vesc_get_values_all(vesc_values_t values[4]);

/* Legacy stub, safe to ignore if unused. */
int vesc_poll(float* v_in, float* t_mos, float* t_motor);

#ifdef __cplusplus
}
#endif

#endif /* VESC_UART_H */