#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "src/cyw43.h"

#define TEST_WIFI_JOIN_STATE_ACTIVE (1)
#define TEST_WIFI_JOIN_STATE_BADAUTH (4)

struct pbuf;

extern void (*cyw43_poll)(void);
extern uint32_t cyw43_sleep;
extern unsigned int cyw43_test_disassoc_count;
extern unsigned int cyw43_test_rejoin_count;
extern unsigned int cyw43_test_set_wpa_auth_count;
extern int cyw43_test_disassoc_result;
extern int cyw43_test_join_result;

void (*cyw43_test_scheduled_poll)(void);
unsigned int cyw43_test_schedule_count;

static unsigned int link_up_count;
static unsigned int link_down_count;

static void empty_poll(void) {
}

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
    ++link_up_count;
}

void cyw43_cb_tcpip_set_link_down(cyw43_t *self, int itf) {
    (void)self;
    (void)itf;
    ++link_down_count;
}

void cyw43_cb_process_ethernet(void *cb_data, int itf, size_t len, const uint8_t *buf) {
    (void)cb_data;
    (void)itf;
    (void)len;
    (void)buf;
}

static void reset_join_state(bool in_progress) {
    memset(&cyw43_state, 0, sizeof(cyw43_state));
    cyw43_state.wifi_join_state = TEST_WIFI_JOIN_STATE_ACTIVE;
    cyw43_state.wifi_join_in_progress = in_progress;
    cyw43_poll = NULL;
    cyw43_sleep = 1;
    cyw43_test_scheduled_poll = NULL;
    cyw43_test_schedule_count = 0;
    cyw43_test_disassoc_count = 0;
    cyw43_test_rejoin_count = 0;
    cyw43_test_set_wpa_auth_count = 0;
    cyw43_test_disassoc_result = 0;
    cyw43_test_join_result = 0;
    link_up_count = 0;
    link_down_count = 0;
}

static void prepare_active_interface(void) {
    cyw43_state.initted = true;
    cyw43_state.itf_state = 1 << CYW43_ITF_STA;
    cyw43_poll = empty_poll;
}

static void process_event(uint32_t event_type, uint32_t status, uint32_t reason, uint16_t flags) {
    cyw43_async_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.event_type = event_type;
    ev.status = status;
    ev.reason = reason;
    ev.flags = flags;
    ev.interface = CYW43_ITF_STA;
    cyw43_cb_process_async_event(&cyw43_state, &ev);
}

static void run_scheduled_poll(void) {
    assert(cyw43_test_scheduled_poll != NULL);
    cyw43_poll = cyw43_test_scheduled_poll;
    cyw43_test_scheduled_poll = NULL;
    cyw43_poll();
}

static void complete_secure_join(void) {
    process_event(CYW43_EV_AUTH, CYW43_STATUS_SUCCESS, 0, 0);
    process_event(CYW43_EV_LINK, CYW43_STATUS_SUCCESS, 0, 1);
    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYED, 0, 0);
}

static void test_secure_join_succeeds(void) {
    reset_join_state(true);

    process_event(CYW43_EV_AUTH, CYW43_STATUS_SUCCESS, 0, 0);
    assert(link_up_count == 0);
    process_event(CYW43_EV_LINK, CYW43_STATUS_SUCCESS, 0, 1);
    assert(link_up_count == 0);
    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYED, 0, 0);

    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_JOIN);
    assert(!cyw43_state.wifi_join_in_progress);
    assert(cyw43_state.wifi_join_established);
    assert(!cyw43_state.pend_disassoc);
    assert(!cyw43_state.pend_rejoin);
    assert(link_up_count == 1);
}

static void test_secure_join_requires_all_events(void) {
    reset_join_state(true);
    process_event(CYW43_EV_LINK, CYW43_STATUS_SUCCESS, 0, 1);
    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYED, 0, 0);
    assert(link_up_count == 0);
    assert(cyw43_state.wifi_join_in_progress);

    reset_join_state(true);
    process_event(CYW43_EV_AUTH, CYW43_STATUS_SUCCESS, 0, 0);
    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYED, 0, 0);
    assert(link_up_count == 0);
    assert(cyw43_state.wifi_join_in_progress);

    reset_join_state(true);
    process_event(CYW43_EV_AUTH, CYW43_STATUS_SUCCESS, 0, 0);
    process_event(CYW43_EV_LINK, CYW43_STATUS_SUCCESS, 0, 1);
    assert(link_up_count == 0);
    assert(cyw43_state.wifi_join_in_progress);
}

static void test_open_join_does_not_require_psk_event(void) {
    static const uint8_t ssid[] = "open";

    reset_join_state(false);
    prepare_active_interface();

    assert(cyw43_wifi_join(&cyw43_state, sizeof(ssid) - 1, ssid, 0, NULL,
        CYW43_AUTH_OPEN, NULL, 0) == 0);
    process_event(CYW43_EV_AUTH, CYW43_STATUS_SUCCESS, 0, 0);
    assert(link_up_count == 0);
    process_event(CYW43_EV_LINK, CYW43_STATUS_SUCCESS, 0, 1);

    assert(link_up_count == 1);
    assert(!cyw43_state.wifi_join_in_progress);
    assert(cyw43_state.wifi_join_established);
}

static void test_m3_timeout_is_terminal_bad_auth(void) {
    reset_join_state(true);

    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYXCHANGE_WAIT_M3, CYW43_REASON_SUP_WPA_PSK_TMO, 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_BADAUTH);
    assert(!cyw43_state.wifi_join_in_progress);
    assert(!cyw43_state.pend_rejoin);
    assert(cyw43_state.pend_disassoc);

    run_scheduled_poll();
    assert(cyw43_test_disassoc_count == 1);

    unsigned int schedule_count = cyw43_test_schedule_count;
    process_event(CYW43_EV_ICV_ERROR, CYW43_STATUS_SUCCESS, 0, 0);
    process_event(CYW43_EV_PRUNE, CYW43_STATUS_SUCCESS, CYW43_REASON_RSN_MISMATCH, 0);
    process_event(CYW43_EV_DISASSOC_IND, CYW43_STATUS_SUCCESS, CYW43_REASON_DEAUTH, 0);
    assert(!cyw43_state.pend_rejoin);
    assert(cyw43_test_scheduled_poll == NULL);
    assert(cyw43_test_schedule_count == schedule_count);

    complete_secure_join();
    process_event(CYW43_EV_DISASSOC, CYW43_STATUS_SUCCESS, 0, 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_BADAUTH);
    assert(link_up_count == 0);
}

static void test_psk_deauth_is_terminal_bad_auth(void) {
    reset_join_state(true);

    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYXCHANGE_WAIT_M3, CYW43_REASON_SUP_DEAUTH, 0);
    process_event(CYW43_EV_DEAUTH_IND, CYW43_STATUS_SUCCESS, CYW43_REASON_DEAUTH, 0);

    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_BADAUTH);
    assert(!cyw43_state.wifi_join_in_progress);
    assert(!cyw43_state.pend_rejoin);
    assert(cyw43_state.pend_disassoc);
    assert(cyw43_test_schedule_count == 1);

    run_scheduled_poll();
    assert(cyw43_test_disassoc_count == 1);
}

static void test_authenticated_unkeyed_disassoc_is_terminal_bad_auth(void) {
    reset_join_state(true);

    process_event(CYW43_EV_AUTH, CYW43_STATUS_SUCCESS, 0, 0);
    process_event(CYW43_EV_LINK, CYW43_STATUS_SUCCESS, 0, 1);
    process_event(CYW43_EV_SET_SSID, CYW43_STATUS_SUCCESS, 0, 0);
    process_event(CYW43_EV_DISASSOC_IND, CYW43_STATUS_SUCCESS, CYW43_REASON_DEAUTH, 0);

    assert(cyw43_test_rejoin_count == 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_BADAUTH);
    assert(!cyw43_state.wifi_join_in_progress);
    assert(!cyw43_state.pend_rejoin);
    assert(cyw43_state.pend_disassoc);
}

static void test_early_disassoc_is_terminal_failure(void) {
    reset_join_state(true);

    process_event(CYW43_EV_DISASSOC_IND, CYW43_STATUS_SUCCESS, CYW43_REASON_DEAUTH, 0);

    assert(cyw43_test_rejoin_count == 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_FAIL);
    assert(!cyw43_state.wifi_join_in_progress);
    assert(!cyw43_state.pend_rejoin);
    assert(cyw43_state.pend_disassoc);
}

static void test_m1_and_g1_timeouts_are_terminal_failures(void) {
    reset_join_state(true);

    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYXCHANGE_WAIT_M1, CYW43_REASON_SUP_WPA_PSK_TMO, 0);
    assert(cyw43_test_rejoin_count == 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_FAIL);
    assert(!cyw43_state.pend_rejoin);
    assert(cyw43_state.pend_disassoc);

    reset_join_state(true);
    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYXCHANGE_WAIT_G1, CYW43_REASON_SUP_WPA_PSK_TMO, 0);
    assert(cyw43_test_rejoin_count == 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_FAIL);
    assert(!cyw43_state.pend_rejoin);
    assert(cyw43_state.pend_disassoc);
}

static void test_rsn_mismatch_and_icv_are_terminal_failures(void) {
    reset_join_state(true);

    process_event(CYW43_EV_PRUNE, CYW43_STATUS_SUCCESS, CYW43_REASON_RSN_MISMATCH, 0);
    assert(cyw43_test_rejoin_count == 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_FAIL);
    assert(!cyw43_state.pend_rejoin);
    assert(!cyw43_state.pend_rejoin_wpa);
    assert(cyw43_state.pend_disassoc);

    reset_join_state(true);
    process_event(CYW43_EV_ICV_ERROR, CYW43_STATUS_SUCCESS, 0, 0);
    assert(cyw43_test_rejoin_count == 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_FAIL);
    assert(!cyw43_state.pend_rejoin);
    assert(cyw43_state.pend_disassoc);
}

static void test_disassoc_is_terminal_failure(void) {
    reset_join_state(true);

    process_event(CYW43_EV_DISASSOC, CYW43_STATUS_SUCCESS, 0, 0);

    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_FAIL);
    assert(!cyw43_state.wifi_join_in_progress);
    assert(!cyw43_state.pend_rejoin);
    assert(cyw43_state.pend_disassoc);
}

static void test_encryption_mismatch_is_terminal(void) {
    reset_join_state(true);

    process_event(CYW43_EV_PRUNE, CYW43_STATUS_SUCCESS, CYW43_REASON_PRUNE_ENCR_MISMATCH, 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_JOIN);

    process_event(CYW43_EV_SET_SSID, CYW43_STATUS_FAIL, 0, 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_ENCR_MISMATCH);
    assert(!cyw43_state.wifi_join_in_progress);
    assert(cyw43_state.pend_disassoc);

    process_event(CYW43_EV_DISASSOC, CYW43_STATUS_SUCCESS, 0, 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_ENCR_MISMATCH);
}

static void test_mismatch_does_not_poison_matching_candidate(void) {
    reset_join_state(true);

    process_event(CYW43_EV_PRUNE, CYW43_STATUS_SUCCESS, CYW43_REASON_PRUNE_ENCR_MISMATCH, 0);
    complete_secure_join();

    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_JOIN);
    assert(!cyw43_state.wifi_join_in_progress);
    assert(link_up_count == 1);

    reset_join_state(true);
    process_event(CYW43_EV_PRUNE, CYW43_STATUS_SUCCESS, CYW43_REASON_PRUNE_ENCR_MISMATCH, 0);
    process_event(CYW43_EV_AUTH, CYW43_STATUS_SUCCESS, 0, 0);
    process_event(CYW43_EV_SET_SSID, CYW43_STATUS_FAIL, 0, 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_FAIL);
}

static void test_missing_network_is_terminal(void) {
    reset_join_state(true);

    process_event(CYW43_EV_SET_SSID, 3, 0, 0);

    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_NONET);
    assert(!cyw43_state.wifi_join_in_progress);
    assert(cyw43_state.pend_disassoc);
}

static void test_new_join_clears_result_and_can_connect(void) {
    static const uint8_t ssid[] = "test";
    static const uint8_t key[] = "password";

    reset_join_state(false);
    prepare_active_interface();
    cyw43_state.wifi_join_state = TEST_WIFI_JOIN_STATE_BADAUTH;
    cyw43_state.wifi_join_result = TEST_WIFI_JOIN_STATE_BADAUTH;
    cyw43_state.pend_disassoc = true;
    cyw43_state.pend_rejoin = true;
    cyw43_state.pend_rejoin_wpa = true;
    assert(cyw43_wifi_join(&cyw43_state, sizeof(ssid) - 1, ssid, sizeof(key) - 1, key,
        CYW43_AUTH_WPA2_AES_PSK, NULL, 0) == 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_JOIN);
    assert(cyw43_state.wifi_join_result == 0);
    assert(cyw43_state.wifi_join_in_progress);
    assert(!cyw43_state.pend_disassoc);
    assert(!cyw43_state.pend_rejoin);

    complete_secure_join();
    assert(!cyw43_state.wifi_join_in_progress);
    assert(link_up_count == 1);
}

static void test_rejected_join_preserves_terminal_result(void) {
    static const uint8_t ssid[] = "test";
    static const uint8_t key[] = "password";

    reset_join_state(false);
    prepare_active_interface();
    cyw43_state.wifi_join_state = TEST_WIFI_JOIN_STATE_BADAUTH;
    cyw43_state.wifi_join_result = TEST_WIFI_JOIN_STATE_BADAUTH;
    cyw43_state.pend_disassoc = true;
    cyw43_test_join_result = -CYW43_EIO;

    assert(cyw43_wifi_join(&cyw43_state, sizeof(ssid) - 1, ssid, sizeof(key) - 1, key,
        CYW43_AUTH_WPA2_AES_PSK, NULL, 0) == -CYW43_EIO);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_BADAUTH);
    assert(cyw43_state.wifi_join_result == TEST_WIFI_JOIN_STATE_BADAUTH);
    assert(cyw43_state.pend_disassoc);
}

static void test_leave_clears_join_state(void) {
    reset_join_state(false);
    prepare_active_interface();
    cyw43_state.wifi_join_state = TEST_WIFI_JOIN_STATE_BADAUTH;
    cyw43_state.wifi_join_result = TEST_WIFI_JOIN_STATE_BADAUTH;
    cyw43_state.pend_disassoc = true;
    cyw43_state.pend_rejoin = true;
    cyw43_state.pend_rejoin_wpa = true;
    assert(cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA) == 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_DOWN);
    assert(cyw43_test_disassoc_count == 1);
    assert(cyw43_state.wifi_join_result == 0);
    assert(!cyw43_state.wifi_join_in_progress);
    assert(!cyw43_state.pend_disassoc);
    assert(!cyw43_state.pend_rejoin);
    assert(!cyw43_state.pend_rejoin_wpa);
}

static void test_delayed_recovery_events_after_leave_do_not_rejoin(void) {
    reset_join_state(false);
    prepare_active_interface();
    cyw43_state.wifi_join_established = true;
    assert(cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA) == 0);

    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYXCHANGE_WAIT_M3, CYW43_REASON_SUP_WPA_PSK_TMO, 0);
    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYXCHANGE_WAIT_M1, CYW43_REASON_SUP_WPA_PSK_TMO, 0);
    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYXCHANGE_WAIT_G1, CYW43_REASON_SUP_WPA_PSK_TMO, 0);
    process_event(CYW43_EV_PRUNE, CYW43_STATUS_SUCCESS, CYW43_REASON_RSN_MISMATCH, 0);
    process_event(CYW43_EV_ICV_ERROR, CYW43_STATUS_SUCCESS, 0, 0);
    process_event(CYW43_EV_SET_SSID, CYW43_STATUS_FAIL, 0, 0);
    process_event(CYW43_EV_AUTH, CYW43_STATUS_FAIL, 0, 0);
    process_event(CYW43_EV_DEAUTH_IND, CYW43_STATUS_SUCCESS, CYW43_REASON_DEAUTH, 0);
    process_event(CYW43_EV_LINK, CYW43_STATUS_SUCCESS, 0, 1);
    process_event(CYW43_EV_PSK_SUP, CYW43_STATUS_FAIL, 0, 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_DOWN);
    assert(!cyw43_state.wifi_join_established);
    assert(!cyw43_state.pend_rejoin);
    assert(!cyw43_state.pend_rejoin_wpa);
    assert(cyw43_test_scheduled_poll == NULL);
    assert(cyw43_test_schedule_count == 0);
}

static void test_failed_leave_preserves_terminal_result(void) {
    reset_join_state(false);
    prepare_active_interface();
    cyw43_state.wifi_join_state = TEST_WIFI_JOIN_STATE_BADAUTH;
    cyw43_state.wifi_join_result = TEST_WIFI_JOIN_STATE_BADAUTH;
    cyw43_state.pend_disassoc = true;
    cyw43_test_disassoc_result = -CYW43_EIO;

    assert(cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA) == -CYW43_EIO);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_BADAUTH);
    assert(cyw43_state.wifi_join_result == TEST_WIFI_JOIN_STATE_BADAUTH);
    assert(cyw43_state.pend_disassoc);
}

static void test_post_connection_recovery_is_unchanged(void) {
    reset_join_state(false);
    cyw43_state.wifi_join_established = true;

    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYXCHANGE_WAIT_M3, CYW43_REASON_SUP_WPA_PSK_TMO, 0);
    run_scheduled_poll();
    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYXCHANGE_WAIT_M1, CYW43_REASON_SUP_WPA_PSK_TMO, 0);
    run_scheduled_poll();
    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYXCHANGE_WAIT_G1, CYW43_REASON_SUP_WPA_PSK_TMO, 0);
    run_scheduled_poll();
    process_event(CYW43_EV_PRUNE, CYW43_STATUS_SUCCESS, CYW43_REASON_RSN_MISMATCH, 0);
    run_scheduled_poll();
    process_event(CYW43_EV_ICV_ERROR, CYW43_STATUS_SUCCESS, 0, 0);
    run_scheduled_poll();

    assert(cyw43_test_rejoin_count == 5);
    assert(cyw43_test_set_wpa_auth_count == 1);
    assert(cyw43_state.wifi_join_result == 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_JOIN);
}

static void test_post_connection_recovery_uses_established_state(void) {
    reset_join_state(false);
    cyw43_state.wifi_join_established = true;

    process_event(CYW43_EV_AUTH, CYW43_STATUS_FAIL, 0, 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_BADAUTH);
    process_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYXCHANGE_WAIT_M3, CYW43_REASON_SUP_WPA_PSK_TMO, 0);
    run_scheduled_poll();

    process_event(CYW43_EV_SET_SSID, CYW43_STATUS_FAIL, 0, 0);
    assert(cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_FAIL);
    process_event(CYW43_EV_ICV_ERROR, CYW43_STATUS_SUCCESS, 0, 0);
    run_scheduled_poll();

    assert(cyw43_test_rejoin_count == 2);
    assert(cyw43_state.wifi_join_established);
}

int main(void) {
    test_secure_join_succeeds();
    test_secure_join_requires_all_events();
    test_open_join_does_not_require_psk_event();
    test_m3_timeout_is_terminal_bad_auth();
    test_psk_deauth_is_terminal_bad_auth();
    test_authenticated_unkeyed_disassoc_is_terminal_bad_auth();
    test_early_disassoc_is_terminal_failure();
    test_m1_and_g1_timeouts_are_terminal_failures();
    test_rsn_mismatch_and_icv_are_terminal_failures();
    test_disassoc_is_terminal_failure();
    test_encryption_mismatch_is_terminal();
    test_mismatch_does_not_poison_matching_candidate();
    test_missing_network_is_terminal();
    test_new_join_clears_result_and_can_connect();
    test_rejected_join_preserves_terminal_result();
    test_leave_clears_join_state();
    test_delayed_recovery_events_after_leave_do_not_rejoin();
    test_failed_leave_preserves_terminal_result();
    test_post_connection_recovery_is_unchanged();
    test_post_connection_recovery_uses_established_state();

    puts("join state tests passed");
    return 0;
}
