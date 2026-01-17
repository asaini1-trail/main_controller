#include "vesc_uart.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <stdlib.h>

/* ---------- GLOBAL FDs FOR LEFT & RIGHT ---------- */

static int g_fd_left  = -1;
static int g_fd_right = -1;

/* ---------- CONFIGURABLE CONSTANTS ---------- */

// Use your actual stable paths:
#define VESC_LEFT_UART_DEVICE   "/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0"
#define VESC_RIGHT_UART_DEVICE  "/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0"

/* UART baud rate */
#define VESC_UART_BAUD          115200

/* CAN IDs of slave VESCs on the LEFT side */
static const uint8_t VESC_CAN_LEFT_SLAVES[] = {
    85,  /* example: left rear VESC CAN ID */
    /* add more left-side CAN IDs here if needed */
};

/* CAN IDs of slave VESCs on the RIGHT side */
static const uint8_t VESC_CAN_RIGHT_SLAVES[] = {
    104,  /* example: right rear VESC CAN ID */
    /* add more right-side CAN IDs here if needed */
};

/* VESC command IDs (from VESC firmware) */
#define COMM_GET_VALUES    4
#define COMM_SET_DUTY      5
#define COMM_FORWARD_CAN   34

/* ---------- INTERNAL HELPERS ---------- */

static uint16_t crc16(const uint8_t *data, int len) {
    /* Standard CRC16 implementation used by VESC UART protocol */
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

static int open_config_port(const char *dev, int baud) {
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        return -1;
    }

    struct termios tio;
    memset(&tio, 0, sizeof(tio));

    cfmakeraw(&tio);

    speed_t spd = B115200;
    switch (baud) {
        case 9600:   spd = B9600;   break;
        case 19200:  spd = B19200;  break;
        case 38400:  spd = B38400;  break;
        case 57600:  spd = B57600;  break;
        case 115200: spd = B115200; break;
        default:     spd = B115200; break;
    }

    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tio.c_cflag |= CS8;

    tio.c_cc[VTIME] = 1; /* 0.1s read timeout */
    tio.c_cc[VMIN]  = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    usleep(100000); /* Let things settle a bit */

    return fd;
}

/* Read exactly len bytes or fail lightly. */
static int read_all(int fd, uint8_t *buf, int len) {
    int got = 0;
    while (got < len) {
        ssize_t n = read(fd, buf + got, len - got);
        if (n < 0) {
            return -1;
        }
        if (n == 0) {
            /* timeout-ish */
            usleep(1000);
            continue;
        }
        got += (int)n;
    }
    return 0;
}

/* Build and send a raw VESC UART frame with given payload on a specific fd */
static int vesc_send_payload_fd(int fd, const uint8_t *payload, uint8_t len) {
    if (fd < 0) {
        return -1;
    }

    /* frame: [start][len][payload...][crc_hi][crc_lo][stop] */
    uint8_t frame[3 + 256 + 2 + 1]; /* more than enough for small payloads */
    if (len > 256) {
        return -2;
    }

    frame[0] = 2;      /* short frame start byte */
    frame[1] = len;

    memcpy(&frame[2], payload, len);

    uint16_t c = crc16(payload, len);
    frame[2 + len] = (uint8_t)(c >> 8);
    frame[3 + len] = (uint8_t)(c & 0xFF);
    frame[4 + len] = 3;  /* stop byte */

    ssize_t total_len = 2 + len + 2 + 1;
    ssize_t n = write(fd, frame, total_len);

    return (n == total_len) ? 0 : -3;
}

/* Read one VESC UART frame (short frame) and return payload length.
   Returns:
      >=0  -> payload length
      <0   -> error
*/
static int vesc_read_payload_fd(int fd, uint8_t *payload, int maxlen) {
    if (fd < 0) {
        return -1;
    }

    uint8_t start;
    uint8_t len;

    /* Read start byte */
    if (read_all(fd, &start, 1) < 0) {
        return -2;
    }
    if (start != 2) {  /* short frame */
        return -3;
    }

    /* Read length */
    if (read_all(fd, &len, 1) < 0) {
        return -4;
    }
    if (len > maxlen) {
        /* drain and fail */
        uint8_t dump[512];
        read(fd, dump, sizeof(dump));
        return -5;
    }

    /* Read payload */
    if (read_all(fd, payload, len) < 0) {
        return -6;
    }

    /* Read CRC + stop */
    uint8_t tail[3];
    if (read_all(fd, tail, 3) < 0) {
        return -7;
    }

    uint8_t crc_hi = tail[0];
    uint8_t crc_lo = tail[1];
    uint8_t stop   = tail[2];

    if (stop != 3) {
        return -8;
    }

    uint16_t c = crc16(payload, len);
    if (((c >> 8) & 0xFF) != crc_hi || (c & 0xFF) != crc_lo) {
        return -9;
    }

    return (int)len;
}

/* Send duty to the local VESC (no CAN) over a specific fd */
static int vesc_send_duty_local_fd(int fd, double duty) {
    if (fd < 0) {
        return -1;
    }

    if (duty > 0.95)  duty = 0.95;
    if (duty < -0.95) duty = -0.95;

    int32_t val = (int32_t)(duty * 100000.0);

    uint8_t payload[1 + 4];
    payload[0] = COMM_SET_DUTY;
    payload[1] = (uint8_t)((val >> 24) & 0xFF);
    payload[2] = (uint8_t)((val >> 16) & 0xFF);
    payload[3] = (uint8_t)((val >> 8)  & 0xFF);
    payload[4] = (uint8_t)(val & 0xFF);

    return vesc_send_payload_fd(fd, payload, sizeof(payload));
}

/* Send duty to a VESC on CAN bus through COMM_FORWARD_CAN on a specific fd */
static int vesc_send_duty_can_fd(int fd, double duty, uint8_t can_id) {
    if (fd < 0) {
        return -1;
    }

    if (duty > 0.95)  duty = 0.95;
    if (duty < -0.95) duty = -0.95;

    int32_t val = (int32_t)(duty * 100000.0);

    /* inner command: SET_DUTY */
    uint8_t inner[1 + 4];
    inner[0] = COMM_SET_DUTY;
    inner[1] = (uint8_t)((val >> 24) & 0xFF);
    inner[2] = (uint8_t)((val >> 16) & 0xFF);
    inner[3] = (uint8_t)((val >> 8)  & 0xFF);
    inner[4] = (uint8_t)(val & 0xFF);

    /* outer payload: FORWARD_CAN, CAN_ID, inner... */
    uint8_t payload[1 + 1 + sizeof(inner)];
    payload[0] = COMM_FORWARD_CAN;
    payload[1] = can_id;
    memcpy(&payload[2], inner, sizeof(inner));

    return vesc_send_payload_fd(fd, payload, sizeof(payload));
}

/* Send COMM_GET_VALUES or COMM_FORWARD_CAN+COMM_GET_VALUES */
static int vesc_send_get_values_fd(int fd, uint8_t can_id) {
    if (fd < 0) return -1;

    if (can_id == 0) {
        /* Direct to master */
        uint8_t req[1] = { COMM_GET_VALUES };
        return vesc_send_payload_fd(fd, req, sizeof(req));
    } else {
        /* Forward over CAN */
        uint8_t inner[1] = { COMM_GET_VALUES };
        uint8_t payload[1 + 1 + sizeof(inner)];
        payload[0] = COMM_FORWARD_CAN;
        payload[1] = can_id;
        memcpy(&payload[2], inner, sizeof(inner));
        return vesc_send_payload_fd(fd, payload, sizeof(payload));
    }
}

/* Parse COMM_GET_VALUES payload into vesc_values_t (subset fields only). */
static int vesc_get_values_parse(const uint8_t *buf, int len, vesc_values_t *out) {
    if (!buf || !out || len <= 0) {
        return -1;
    }
    if (buf[0] != COMM_GET_VALUES) {
        return -2;
    }

    int idx = 1;

    /* temp_mos (int16, degC*10) -> skip */
    idx += 2;

    /* temp_motor (int16, degC*10) -> skip */
    idx += 2;

    /* current_motor (int32, A*100) */
    if (idx + 4 > len) return -3;
    int32_t current_motor_raw =
        ((int32_t)buf[idx] << 24) |
        ((int32_t)buf[idx+1] << 16) |
        ((int32_t)buf[idx+2] << 8)  |
        ((int32_t)buf[idx+3]);
    idx += 4;

    /* current_in (int32, A*100) */
    if (idx + 4 > len) return -3;
    int32_t current_in_raw =
        ((int32_t)buf[idx] << 24) |
        ((int32_t)buf[idx+1] << 16) |
        ((int32_t)buf[idx+2] << 8)  |
        ((int32_t)buf[idx+3]);
    idx += 4;

    /* duty_now (int32, *100000) -> skip */
    idx += 4;

    /* rpm (int32) */
    if (idx + 4 > len) return -3;
    int32_t rpm_raw =
        ((int32_t)buf[idx] << 24) |
        ((int32_t)buf[idx+1] << 16) |
        ((int32_t)buf[idx+2] << 8)  |
        ((int32_t)buf[idx+3]);
    idx += 4;

    /* v_in (int16, V*10) */
    if (idx + 2 > len) return -3;
    int16_t v_in_raw =
        (int16_t)(((int16_t)buf[idx] << 8) | (int16_t)buf[idx+1]);
    idx += 2;

    /* WORKAROUND for VESC FW 6.05: voltage at byte 28 */
    if (v_in_raw == 0 && len > 28) {
        v_in_raw = (int16_t)buf[28];
    }

    /* amp_hours (int32, Ah*10000) -> skip */
    idx += 4;

    /* amp_hours_charged (int32, Ah*10000) -> skip */
    idx += 4;

    /* watt_hours (int32, Wh*10000) -> skip */
    idx += 4;

    /* watt_hours_charged (int32, Wh*10000) -> skip */
    idx += 4;

    /* tachometer (int32) -> skip */
    idx += 4;

    /* tachometer_abs (int32) -> skip */
    idx += 4;

    /* fault code (uint8) */
    if (idx >= len) return -3;
    uint8_t fault = buf[idx];

    out->current_motor = (float)current_motor_raw / 100.0f;
    out->current_in    = (float)current_in_raw    / 100.0f;
    out->rpm           = (float)rpm_raw;
    out->v_in          = (float)v_in_raw / 10.0f;
    out->fault_code    = fault;

    return 0;
}

/* Core function: send GET_VALUES (optional CAN) over fd and parse into out. */
static int vesc_get_values_fd(int fd, uint8_t can_id, vesc_values_t *out) {
    if (fd < 0 || !out) {
        return -1;
    }

    int s = vesc_send_get_values_fd(fd, can_id);
    if (s < 0) {
        return -2;
    }

    uint8_t buf[256];
    int len = vesc_read_payload_fd(fd, buf, sizeof(buf));
    if (len <= 0) {
        return -3;
    }

    return vesc_get_values_parse(buf, len, out);
}

/* ---------- PUBLIC API ---------- */

int vesc_open_noargs(void) {
    int status = 0;

    g_fd_left  = open_config_port(VESC_LEFT_UART_DEVICE,  VESC_UART_BAUD);
    if (g_fd_left < 0) {
        status = -1;
    }

    g_fd_right = open_config_port(VESC_RIGHT_UART_DEVICE, VESC_UART_BAUD);
    if (g_fd_right < 0 && status == 0) {
        status = -2;
    }

    return status;
}

void vesc_close(void) {
    if (g_fd_left >= 0) {
        close(g_fd_left);
        g_fd_left = -1;
    }
    if (g_fd_right >= 0) {
        close(g_fd_right);
        g_fd_right = -1;
    }
}

/* LEFT SIDE: send duty to left master + left CAN slaves */
int vesc_send_duty_left(double duty) {
    int status = 0;

    /* 1) Send to LEFT master / local VESC */
    int s_local = vesc_send_duty_local_fd(g_fd_left, duty);
    if (s_local < 0) {
        status = s_local;
    }

    /* 2) Forward same duty to all LEFT CAN slaves */
    for (size_t i = 0; i < (sizeof(VESC_CAN_LEFT_SLAVES) / sizeof(VESC_CAN_LEFT_SLAVES[0])); i++) {
        int s_can = vesc_send_duty_can_fd(g_fd_left, duty, VESC_CAN_LEFT_SLAVES[i]);
        if (s_can < 0) {
            status = s_can;
        }
    }

    return status;
}

/* RIGHT SIDE: send duty to right master + right CAN slaves */
int vesc_send_duty_right(double duty) {
    int status = 0;

    /* 1) Send to RIGHT master / local VESC */
    int s_local = vesc_send_duty_local_fd(g_fd_right, duty);
    if (s_local < 0) {
        status = s_local;
    }

    /* 2) Forward same duty to all RIGHT CAN slaves */
    for (size_t i = 0; i < (sizeof(VESC_CAN_RIGHT_SLAVES) / sizeof(VESC_CAN_RIGHT_SLAVES[0])); i++) {
        int s_can = vesc_send_duty_can_fd(g_fd_right, duty, VESC_CAN_RIGHT_SLAVES[i]);
        if (s_can < 0) {
            status = s_can;
        }
    }

    return status;
}

/* LEFT: can_id == 0 -> master, else CAN slave on left bus */
int vesc_get_values_left(uint8_t can_id, vesc_values_t *out) {
    return vesc_get_values_fd(g_fd_left, can_id, out);
}

/* RIGHT: can_id == 0 -> master, else CAN slave on right bus */
int vesc_get_values_right(uint8_t can_id, vesc_values_t *out) {
    return vesc_get_values_fd(g_fd_right, can_id, out);
}

/* Convenience: get values from 4 motors in fixed order:
   0: left master
   1: first left slave  (VESC_CAN_LEFT_SLAVES[0])
   2: right master
   3: first right slave (VESC_CAN_RIGHT_SLAVES[0])
*/
int vesc_get_values_all(vesc_values_t values[4]) {
    int status = 0;
    
    if (vesc_get_values_left(0, &values[0]) < 0) {
        status = -1;
    }
    if (sizeof(VESC_CAN_LEFT_SLAVES) > 0 &&
        vesc_get_values_left(VESC_CAN_LEFT_SLAVES[0], &values[1]) < 0) {
        status = -2;
    }
    if (vesc_get_values_right(0, &values[2]) < 0) {
        status = -3;
    }
    if (sizeof(VESC_CAN_RIGHT_SLAVES) > 0 &&
        vesc_get_values_right(VESC_CAN_RIGHT_SLAVES[0], &values[3]) < 0) {
        status = -4;
    }

    return status;
}

/* Legacy stub for compatibility; unused if you're using vesc_get_values_* */
int vesc_poll(float* v_in, float* t_mos, float* t_motor) {
    (void)v_in;
    (void)t_mos;
    (void)t_motor;
    return -1;
}
