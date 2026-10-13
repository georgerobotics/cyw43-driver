#include "src/cyw43.h"

unsigned int cyw43_test_rejoin_count;
unsigned int cyw43_test_disassoc_count;
unsigned int cyw43_test_set_wpa_auth_count;
int cyw43_test_disassoc_result;
int cyw43_test_join_result;

void cyw43_ll_init(cyw43_ll_t *self, void *cb_data) {
    (void)self;
    (void)cb_data;
}

void cyw43_ll_deinit(cyw43_ll_t *self) {
    (void)self;
}

bool cyw43_ll_has_work(cyw43_ll_t *self) {
    (void)self;
    return false;
}

int cyw43_ll_bus_init(cyw43_ll_t *self, const uint8_t *mac) {
    (void)self;
    (void)mac;
    return 0;
}

void cyw43_ll_process_packets(cyw43_ll_t *self) {
    (void)self;
}

int cyw43_ll_send_ethernet(cyw43_ll_t *self, int itf, size_t len, const void *buf, bool is_pbuf) {
    (void)self;
    (void)itf;
    (void)len;
    (void)buf;
    (void)is_pbuf;
    return 0;
}

int cyw43_ll_ioctl(cyw43_ll_t *self, uint32_t cmd, size_t len, uint8_t *buf, uint32_t iface) {
    (void)self;
    (void)len;
    (void)buf;
    (void)iface;
    if (cmd == CYW43_IOCTL_SET_DISASSOC) {
        ++cyw43_test_disassoc_count;
        return cyw43_test_disassoc_result;
    }
    return 0;
}

void cyw43_ll_wifi_set_wpa_auth(cyw43_ll_t *self) {
    (void)self;
    ++cyw43_test_set_wpa_auth_count;
}

int cyw43_ll_wifi_on(cyw43_ll_t *self, uint32_t country) {
    (void)self;
    (void)country;
    return 0;
}

int cyw43_ll_wifi_pm(cyw43_ll_t *self, uint32_t pm, uint32_t pm_sleep_ret, uint32_t li_bcn, uint32_t li_dtim, uint32_t li_assoc) {
    (void)self;
    (void)pm;
    (void)pm_sleep_ret;
    (void)li_bcn;
    (void)li_dtim;
    (void)li_assoc;
    return 0;
}

int cyw43_ll_wifi_get_pm(cyw43_ll_t *self, uint32_t *pm, uint32_t *pm_sleep_ret, uint32_t *li_bcn, uint32_t *li_dtim, uint32_t *li_assoc) {
    (void)self;
    (void)pm;
    (void)pm_sleep_ret;
    (void)li_bcn;
    (void)li_dtim;
    (void)li_assoc;
    return 0;
}

int cyw43_ll_wifi_scan(cyw43_ll_t *self, cyw43_wifi_scan_options_t *opts) {
    (void)self;
    (void)opts;
    return 0;
}

int cyw43_ll_wifi_get_bssid(cyw43_ll_t *self, uint8_t *bssid) {
    (void)self;
    (void)bssid;
    return 0;
}

int cyw43_ll_wifi_join(cyw43_ll_t *self, size_t ssid_len, const uint8_t *ssid, size_t key_len, const uint8_t *key, uint32_t auth_type, const uint8_t *bssid, uint32_t channel) {
    (void)self;
    (void)ssid_len;
    (void)ssid;
    (void)key_len;
    (void)key;
    (void)auth_type;
    (void)bssid;
    (void)channel;
    return cyw43_test_join_result;
}

void cyw43_ll_wifi_rejoin(cyw43_ll_t *self) {
    (void)self;
    ++cyw43_test_rejoin_count;
}

int cyw43_ll_wifi_ap_init(cyw43_ll_t *self, size_t ssid_len, const uint8_t *ssid, uint32_t auth, size_t key_len, const uint8_t *key, uint32_t channel) {
    (void)self;
    (void)ssid_len;
    (void)ssid;
    (void)auth;
    (void)key_len;
    (void)key;
    (void)channel;
    return 0;
}

int cyw43_ll_wifi_ap_set_up(cyw43_ll_t *self, bool up) {
    (void)self;
    (void)up;
    return 0;
}

int cyw43_ll_wifi_ap_get_stas(cyw43_ll_t *self, int *num_stas, uint8_t *macs) {
    (void)self;
    (void)num_stas;
    (void)macs;
    return 0;
}

int cyw43_ll_wifi_update_multicast_filter(cyw43_ll_t *self, uint8_t *addr, bool add) {
    (void)self;
    (void)addr;
    (void)add;
    return 0;
}

void cyw43_ll_bus_sleep(cyw43_ll_t *self, bool can_sleep) {
    (void)self;
    (void)can_sleep;
}

void cyw43_sdio_deinit(void) {
}

void cyw43_sdio_init(void) {
}

void cyw43_sdio_reinit(void) {
}
