#include "src/cyw43.h"
#include "src/cyw43_country.h"

struct pbuf;

uint16_t pbuf_copy_partial(const struct pbuf *p, void *dataptr, uint16_t len, uint16_t offset) {
    return 0;
}

void cyw43_cb_tcpip_init(cyw43_t *self, int itf) {
}

void cyw43_cb_tcpip_deinit(cyw43_t *self, int itf) {
}

void cyw43_cb_tcpip_set_link_up(cyw43_t *self, int itf) {
}

void cyw43_cb_tcpip_set_link_down(cyw43_t *self, int itf) {
}

void cyw43_cb_process_ethernet(void *cb_data, int itf, size_t len, const uint8_t *buf) {
}

uint32_t storage_read_blocks(uint8_t *dest, uint32_t block_num, uint32_t num_blocks) {
    return 0;
}

int main(int argc, char **argv) {
    cyw43_init(&cyw43_state);
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_WORLDWIDE);
    cyw43_deinit(&cyw43_state);
    return 0;
}
