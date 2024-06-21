#include "src/cyw43.h"
#include "src/cyw43_country.h"

struct pbuf;

uint16_t pbuf_copy_partial(const struct pbuf *p, void *dataptr, uint16_t len, uint16_t offset) {
    (void)p;
    (void)dataptr;
    (void)len;
    (void)offset;
    return 0;
}

void cyw43_cb_tcpip_init(cyw43_t *self, int itf) {
    (void)self;
    (void)itf;
}

void cyw43_cb_tcpip_deinit(cyw43_t *self, int itf) {
    (void)self;
    (void)itf;
}

void cyw43_cb_tcpip_set_link_up(cyw43_t *self, int itf) {
    (void)self;
    (void)itf;
}

void cyw43_cb_tcpip_set_link_down(cyw43_t *self, int itf) {
    (void)self;
    (void)itf;
}

void cyw43_cb_process_ethernet(void *cb_data, int itf, size_t len, const uint8_t *buf) {
    (void)cb_data;
    (void)itf;
    (void)len;
    (void)buf;
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    cyw43_init(&cyw43_state);
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_WORLDWIDE);
    cyw43_deinit(&cyw43_state);
    return 0;
}
