#ifndef CYW43_INCLUDED_CONFIGPORT_H
#define CYW43_INCLUDED_CONFIGPORT_H

#include <stdint.h>

#define MIN(a, b)                           ((a) <= (b) ? (a) : (b))
#define static_assert(expr, msg)            typedef int static_assert_##__LINE__[(expr) ? 1 : -1]
#define CYW43_ARRAY_SIZE(a)                 (sizeof(a) / sizeof((a)[0]))

#define CYW43_USE_SPI                       (0)
#define CYW43_LWIP                          (0)

#define CYW43_PIN_WL_REG_ON                 (1)
#define CYW43_PIN_WL_RFSW_VDD               (2)
#define CYW43_PIN_WL_SDIO_1                 (3)
#define CYW43_PIN_WL_HOST_WAKE              (4)

#define CYW43_EPERM                         (1)
#define CYW43_EIO                           (5)
#define CYW43_EINVAL                        (22)
#define CYW43_ETIMEDOUT                     (110)

#define CYW43_THREAD_ENTER                  do { } while (0)
#define CYW43_THREAD_EXIT                   do { } while (0)
#define CYW43_THREAD_LOCK_CHECK             do { } while (0)

#define CYW43_SDPCM_SEND_COMMON_WAIT        do { } while (0)
#define CYW43_DO_IOCTL_WAIT                 do { } while (0)

#define CYW43_HAL_PIN_MODE_INPUT            (0)
#define CYW43_HAL_PIN_MODE_OUTPUT           (1)
#define CYW43_HAL_PIN_PULL_NONE             (0)
#define CYW43_HAL_MAC_WLAN0                 (0)

static inline unsigned int cyw43_hal_ticks_us(void) {
    return 0;
}

static inline unsigned int cyw43_hal_ticks_ms(void) {
    return 0;
}

static inline void cyw43_delay_us(unsigned int us) {
}

static inline void cyw43_delay_ms(unsigned int ms) {
}

static inline void cyw43_hal_get_mac(int interface, uint8_t mac[6]) {
}

static inline void cyw43_hal_pin_config(int pin, int mode, int pull, int alt) {
}

static inline void cyw43_hal_pin_config_irq_falling(int pin, int enable) {
}

static inline int cyw43_hal_pin_read(int pin) {
    return 0;
}

static inline void cyw43_hal_pin_low(int pin) {
}

static inline void cyw43_hal_pin_high(int pin) {
}

static inline void cyw43_schedule_internal_poll_dispatch(void (*func)(void)) {
}

#endif // CYW43_INCLUDED_CONFIGPORT_H
