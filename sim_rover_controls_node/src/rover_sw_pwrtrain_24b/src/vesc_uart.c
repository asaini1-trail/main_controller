#include "vesc_uart.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* ============================================================
 * PLATFORM SWITCH
 * ============================================================ */

#if defined(__linux__)

/* ====================== LINUX IMPLEMENTATION ====================== */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>

/* ---------- GLOBAL FDs ---------- */
static int g_fd_left  = -1;
static int g_fd_right = -1;

/* ---------- CONFIG ---------- */
#define VESC_LEFT_UART_DEVICE   "/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0"
#define VESC_RIGHT_UART_DEVICE  "/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0"
#define VESC_UART_BAUD          115200

static const uint8_t VESC_CAN_LEFT_SLAVES[]  = { 85 };
static const uint8_t VESC_CAN_RIGHT_SLAVES[] = { 104 };

#define COMM_GET_VALUES    4
#define COMM_SET_DUTY      5
#define COMM_FORWARD_CAN   34

/* ---------- HELPERS ---------- */

static uint16_t crc16(const uint8_t *data, int len) {
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

static int open_config_port(const char *dev, int baud) {
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    cfmakeraw(&tio);

    speed_t spd = B115200;
    if (baud == 9600) spd = B9600;
    else if (baud == 19200) spd = B19200;
    else if (baud == 38400) spd = B38400;
    else if (baud == 57600) spd = B57600;

    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tio.c_cflag |= CS8;

    tio.c_cc[VTIME] = 1;
    tio.c_cc[VMIN]  = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    usleep(100000);
    return fd;
}

static int read_all(int fd, uint8_t *buf, int len) {
    int got = 0;
    while (got < len) {
        ssize_t n = read(fd, buf + got, len - got);
        if (n < 0) return -1;
        if (n == 0) { usleep(1000); continue; }
        got += (int)n;
    }
    return 0;
}

static int vesc_send_payload_fd(int fd, const uint8_t *payload, uint8_t len) {
    if (fd < 0 || len > 256) return -1;

    uint8_t frame[260];
    frame[0] = 2;
    frame[1] = len;
    memcpy(&frame[2], payload, len);

    uint16_t c = crc16(payload, len);
    frame[2 + len] = (uint8_t)(c >> 8);
    frame[3 + len] = (uint8_t)(c & 0xFF);
    frame[4 + len] = 3;

    ssize_t total = 2 + len + 3;
    return (write(fd, frame, total) == total) ? 0 : -1;
}

/* ---------- PUBLIC API (LINUX) ---------- */

int vesc_open_noargs(void) {
    g_fd_left  = open_config_port(VESC_LEFT_UART_DEVICE,  VESC_UART_BAUD);
    g_fd_right = open_config_port(VESC_RIGHT_UART_DEVICE, VESC_UART_BAUD);
    return (g_fd_left < 0 || g_fd_right < 0) ? -1 : 0;
}

void vesc_close(void) {
    if (g_fd_left  >= 0) close(g_fd_left);
    if (g_fd_right >= 0) close(g_fd_right);
    g_fd_left = g_fd_right = -1;
}

int vesc_send_duty_left(double duty) {
    int32_t v = (int32_t)(duty * 100000.0);
    uint8_t p[5] = {
        COMM_SET_DUTY,
        (uint8_t)(v >> 24), (uint8_t)(v >> 16),
        (uint8_t)(v >> 8),  (uint8_t)v
    };
    return vesc_send_payload_fd(g_fd_left, p, sizeof(p));
}

int vesc_send_duty_right(double duty) {
    int32_t v = (int32_t)(duty * 100000.0);
    uint8_t p[5] = {
        COMM_SET_DUTY,
        (uint8_t)(v >> 24), (uint8_t)(v >> 16),
        (uint8_t)(v >> 8),  (uint8_t)v
    };
    return vesc_send_payload_fd(g_fd_right, p, sizeof(p));
}

int vesc_get_values_left(uint8_t can_id, vesc_values_t *out) {
    (void)can_id; (void)out;
    return -1; /* unused in your current Simulink flow */
}

int vesc_get_values_right(uint8_t can_id, vesc_values_t *out) {
    (void)can_id; (void)out;
    return -1;
}

int vesc_get_values_all(vesc_values_t values[4]) {
    (void)values;
    return -1;
}

int vesc_poll(float* a, float* b, float* c) {
    (void)a; (void)b; (void)c;
    return -1;
}

/* ====================== HOST (WINDOWS) STUBS ====================== */

#else

int vesc_open_noargs(void) { return 0; }
void vesc_close(void) {}

int vesc_send_duty_left(double duty)  { (void)duty; return 0; }
int vesc_send_duty_right(double duty) { (void)duty; return 0; }

int vesc_get_values_left(uint8_t id, vesc_values_t *o)
{ (void)id; (void)o; return 0; }

int vesc_get_values_right(uint8_t id, vesc_values_t *o)
{ (void)id; (void)o; return 0; }

int vesc_get_values_all(vesc_values_t v[4])
{ (void)v; return 0; }

int vesc_poll(float* a, float* b, float* c)
{ (void)a; (void)b; (void)c; return 0; }

#endif
