/**
 * Copyright (c) 2023 Beechwoods Software, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/wifi.h>
#include <zephyr/drivers/spi.h>

#include "zephyr_cyw43_drv.h"

#include "georgerobotics/cyw43.h"
#include "georgerobotics/cyw43_country.h"
#include "zephyr_cyw43_log.h"

static void cyw43_set_irq_enabled(bool);

#define DT_DRV_COMPAT infineon_cyw43

LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL);

#define ZEPHYR_CYW43_WORKQUEUE_STACK_SIZE 1024
K_KERNEL_STACK_DEFINE(zephyr_cyw43_work_q_stack, ZEPHYR_CYW43_WORKQUEUE_STACK_SIZE);

static const struct zephyr_cyw43_cfg zephyr_cyw43_cfg = {
        .irq_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(infineon_cyw43_module), host_wake_gpios),
        .wl_on_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(infineon_cyw43_module), wl_on_gpios),
};

static zephyr_cyw43_dev_t zephyr_cyw43_0; /* static instance */
zephyr_cyw43_dev_t *zephyr_cyw43_dev = &zephyr_cyw43_0;

#define EVENT_POLL_THREAD_STACK_SIZE 1024
#define EVENT_POLL_THREAD_PRIO 2
K_KERNEL_STACK_MEMBER(zephyr_cyw43_event_poll_stack, EVENT_POLL_THREAD_STACK_SIZE);

struct k_thread event_thread;
static void zephyr_cyw43_event_poll_thread(void *p1)
{
        int rv;

        LOG_DBG("Starting zephyr_cyw43_event_poll_thread\n");

        while (1) {
                rv = k_sem_take(&zephyr_cyw43_dev->event_sem, K_MSEC(5000));

                cyw43_set_irq_enabled(false);

                if (rv != 0) {
                        if (rv != -EAGAIN) {
                                LOG_DBG("k_sem_take returned nonzero %d\n", rv);
                        }
                }

                if (cyw43_poll) {
                        zephyr_cyw43_lock(zephyr_cyw43_dev);
                        cyw43_poll();
                        zephyr_cyw43_unlock(zephyr_cyw43_dev);
                }
                else {
                        LOG_DBG("Not calling cyw43_poll() from poll_thread, because it doesn't exist.");
                }
        }
}

/* Aa NOP version of this never-called LWIP function needs to exist to avoid link error.
   TODO: remove the requirement for this from georgerobotics code when LWIP is disabled. */
struct pbuf {};
uint16_t pbuf_copy_partial(const struct pbuf *p, void *dataptr, uint16_t len, uint16_t offset) {
        return 0;
}

static int process_cyw43_scan_result(void *env, const cyw43_ev_scan_result_t *result) {

        struct wifi_scan_result zephyr_scan_result;

        memset(&zephyr_scan_result, 0, sizeof(struct wifi_scan_result));

        if (result) {

                LOG_DBG("Setting up zephyr_scan_result\n");
                zephyr_scan_result.ssid_length = strnlen(result->ssid, WIFI_SSID_MAX_LEN);
                strncpy(zephyr_scan_result.ssid, result->ssid, zephyr_scan_result.ssid_length);
                zephyr_scan_result.channel = result->channel;
                zephyr_scan_result.rssi = result->rssi;
                zephyr_scan_result.mac_length = WIFI_MAC_ADDR_LEN;
                memcpy(zephyr_scan_result.mac, result->bssid, WIFI_MAC_ADDR_LEN);

                /* Had to reverse engineer the bit trickery below by looking at/
                   cyw43_ll_wifi_parse_scan_result() */
                zephyr_scan_result.security =
                        CYW43_SECURITY_TO_ZEPHYR_SECURITY(((result->auth_mode | 0x00400000) & 0xFFFFFFFE));

                LOG_DBG("Finished setting up zephyr_scan_result\n");

                zephyr_cyw43_lock(zephyr_cyw43_dev);
                LOG_DBG("calling scan_cb with interface=%s\n", zephyr_cyw43_dev->iface->if_dev->dev->name);
                zephyr_cyw43_dev->scan_cb(zephyr_cyw43_dev->iface, 0, &zephyr_scan_result);
                zephyr_cyw43_unlock(zephyr_cyw43_dev);
        }
        return 0;
}

static int zephyr_cyw43_scan(zephyr_cyw43_dev_t *zephyr_cyw43_device, bool active)
{

        LOG_DBG("Scannings (%s).\n", active ? "active" : "passive");

        int err=0;
        static cyw43_wifi_scan_options_t scan_options;


        if (!cyw43_wifi_scan_active(&cyw43_state)) {
                memset(&scan_options, 0, sizeof(cyw43_wifi_scan_options_t));
                scan_options.scan_type = (active ? 0 : 1);
                err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, process_cyw43_scan_result);

                if (err == 0) {
                        LOG_DBG("Performing wifi scan");
                } else {
                        LOG_ERR("Failed to start scan: %d", err);
                        goto error;
                }
        }
        else {
                LOG_DBG("Wifi_scan already active.");
                goto error;
        }

        for (int i=0; i<50; i++) {
                if (!cyw43_wifi_scan_active(&cyw43_state)) {
                        break;
                }
                k_sleep(K_MSEC(50));
        }

        LOG_DBG("scan is finished.");

        zephyr_cyw43_lock(zephyr_cyw43_device);
        zephyr_cyw43_dev->scan_cb(zephyr_cyw43_device->iface, 0, NULL);
        zephyr_cyw43_unlock(zephyr_cyw43_device);

error:
        return err;
}


static int zephyr_cyw43_connect(zephyr_cyw43_dev_t *zephyr_cyw43_device)
{
        const uint8_t *cyw43_ssid = (const uint8_t *)zephyr_cyw43_device->connect_params.ssid;
        const uint8_t *cyw43_key = (const uint8_t *)zephyr_cyw43_device->connect_params.psk;

        LOG_DBG("Connecting to %s (pass=%s)\n", cyw43_ssid, cyw43_key);

        uint32_t cyw43_auth_type;
        uint32_t cyw43_channel;
        uint32_t cyw43_ssid_len;
        uint32_t cyw43_key_len;
        const uint8_t *cyw43_bssid = NULL;
        int rv;

        LOG_DBG("connect_params.security = %d\n", zephyr_cyw43_device->connect_params.security);
        switch (zephyr_cyw43_device->connect_params.security) {
        case WIFI_SECURITY_TYPE_NONE:
                cyw43_ssid_len = strlen(cyw43_ssid);;
                cyw43_key_len = 0;
                cyw43_auth_type = CYW43_AUTH_OPEN;
                cyw43_channel = (zephyr_cyw43_device->connect_params.channel == 0
                                 ? CYW43_CHANNEL_NONE
                                 : zephyr_cyw43_device->connect_params.channel);
                break;
        case WIFI_SECURITY_TYPE_WPA_PSK: /* WPA-PSK security */
                cyw43_ssid_len = strlen(cyw43_ssid);
                cyw43_key_len = strlen(cyw43_key);
                cyw43_auth_type = CYW43_AUTH_WPA_TKIP_PSK;
                cyw43_channel = (zephyr_cyw43_device->connect_params.channel == 0
                                 ? CYW43_CHANNEL_NONE
                                 : zephyr_cyw43_device->connect_params.channel); 
                break;
        case WIFI_SECURITY_TYPE_PSK: /* WPA2-PSK security */
                cyw43_ssid_len = strlen(cyw43_ssid);
                cyw43_key_len = strlen(cyw43_key);
                cyw43_auth_type = CYW43_AUTH_WPA2_AES_PSK;
                cyw43_channel = (zephyr_cyw43_device->connect_params.channel == 0
                                 ? CYW43_CHANNEL_NONE
                                 : zephyr_cyw43_device->connect_params.channel); 
                break;
        default:
                cyw43_ssid_len = 0;
                cyw43_key_len = 0;
                cyw43_auth_type = CYW43_AUTH_OPEN;
                cyw43_channel = CYW43_CHANNEL_NONE;
                break;
        }

        LOG_DBG(" calling cyw43_wifi_join() with cyw43_ssid_len=%d, cyw43_ssid=%s, cyw43_key_len=%d,\n"
                "cyw43_key=%s, cyw43_auth_type=%d, cyw43_channel=%d\n",
                cyw43_ssid_len, cyw43_ssid, cyw43_key_len,
                cyw43_key, cyw43_auth_type, cyw43_channel);

        /* This looks unbelievably goofy, but the comments in cyw43.h say:
           "After success is returned, periodically call \ref cyw43_wifi_link_status or cyw43_tcpip_link_status,
           to query the status of the link. It can take a many seconds to connect to fully join a network."
        */
        for (int i=0; i<5; i++) {
                rv = 0;

                rv = cyw43_wifi_join(&cyw43_state, cyw43_ssid_len,
                                     cyw43_ssid, cyw43_key_len,
                                     cyw43_key, cyw43_auth_type,
                                     cyw43_bssid, cyw43_channel);
                if (rv) {
                        LOG_ERR("failed to connect.\n");
                        continue;
                }

                /* In my experience, full connection was always achieved before 4 seconds (j==8), but
                   increasing it may ben necessary on some wifi networks. Unfortunately increasing it
                   also raises the /minimum/ connection time.
                */
                for (int j=0; j<8; j++) {
                        int link_status=cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);

                        if (link_status != CYW43_LINK_JOIN) {
                                rv = 1;
                                break;
                        }

                        k_sleep(K_MSEC(500));
                }
                if (rv == 0) {
                        break;
                }
        }

        if (rv != 0) {
                net_if_dormant_on(zephyr_cyw43_device->iface);
                LOG_DBG("zephyr_cyw43_connect failed to connect.\n");
        }
        else {
                net_if_dormant_off(zephyr_cyw43_device->iface);

#if defined(CONFIG_NET_DHCPV4)
                net_dhcpv4_restart(zephyr_cyw43_device->iface);
#endif
                wifi_mgmt_raise_connect_result_event(zephyr_cyw43_device->iface, rv);

                LOG_DBG("zephyr_cyw43_connect connected.\n");
        }

        LOG_DBG("Done Connecting to %s (pass=%s)\n", zephyr_cyw43_device->connect_params.ssid, zephyr_cyw43_device->connect_params.psk);
        return rv;
}

static int zephyr_cyw43_disconnect(zephyr_cyw43_dev_t *zephyr_cyw43_device)
{
        LOG_DBG("Disconnecting from %s", zephyr_cyw43_device->connect_params.ssid);
        int rv;

        rv = cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        if (rv) { goto error; }

        LOG_DBG("Disconnected!");

#if defined(CONFIG_NET_DHCPV4)
        net_dhcpv4_stop(zephyr_cyw43_device->iface);
#endif
        net_if_dormant_on(zephyr_cyw43_device->iface);

        wifi_mgmt_raise_disconnect_result_event(zephyr_cyw43_device->iface, rv);
        return rv;

error:
        return -EIO;
}

static int zephyr_cyw43_enable_ap(zephyr_cyw43_dev_t *zephyr_cyw43_device)
{
        int rv=0;

        const uint8_t *cyw43_ssid = (const uint8_t *)zephyr_cyw43_device->ap_params.ssid;
        const uint8_t *cyw43_password = (const uint8_t *)zephyr_cyw43_device->ap_params.psk;
 
        uint32_t cyw43_auth;
        uint32_t cyw43_channel;
        uint32_t cyw43_ssid_len;
        uint32_t cyw43_password_len;

        LOG_DBG("ap_params.security = %d\n", zephyr_cyw43_device->ap_params.security);
        switch (zephyr_cyw43_device->ap_params.security) {
        case WIFI_SECURITY_TYPE_NONE:
                cyw43_ssid_len = strlen(cyw43_ssid);
                cyw43_password_len = 0;
                cyw43_auth = CYW43_AUTH_OPEN;
                cyw43_channel = (zephyr_cyw43_device->ap_params.channel == 0
                                 ? CYW43_CHANNEL_NONE
                                 : zephyr_cyw43_device->ap_params.channel);
                break;
        case WIFI_SECURITY_TYPE_WPA_PSK: /* WPA-PSK security */
                cyw43_ssid_len = strlen(cyw43_ssid);
                cyw43_password_len = strlen(cyw43_password);
                cyw43_auth = CYW43_AUTH_WPA_TKIP_PSK;
                cyw43_channel = (zephyr_cyw43_device->ap_params.channel == 0
                                 ? CYW43_CHANNEL_NONE
                                 : zephyr_cyw43_device->ap_params.channel);
                break;
        case WIFI_SECURITY_TYPE_PSK: /* WPA2-PSK security */
                cyw43_ssid_len = strlen(cyw43_ssid);
                cyw43_password_len = strlen(cyw43_password);
                cyw43_auth = CYW43_AUTH_WPA2_AES_PSK;
                cyw43_channel = (zephyr_cyw43_device->ap_params.channel == 0
                                 ? CYW43_CHANNEL_NONE
                                 : zephyr_cyw43_device->ap_params.channel);
                break;
        default:
                cyw43_ssid_len = 0;
                cyw43_password_len = 0;
                cyw43_auth = CYW43_AUTH_OPEN;
                cyw43_channel = CYW43_CHANNEL_NONE;
                break;
        }

        LOG_DBG(" Setting up AP with: ssid_len=%d, ssid=%s, password_len=%d, password=%s\n"
                "                     auth=%d, channel=%d\n",
                cyw43_ssid_len, cyw43_ssid, cyw43_password_len, cyw43_password,
                cyw43_auth, cyw43_channel);

        zephyr_cyw43_lock(zephyr_cyw43_device);
        cyw43_wifi_ap_set_ssid(&cyw43_state, cyw43_ssid_len, cyw43_ssid);

        if (cyw43_auth != CYW43_AUTH_OPEN) {
                cyw43_wifi_ap_set_password(&cyw43_state, cyw43_password_len, cyw43_password);
        }

        cyw43_wifi_ap_set_channel(&cyw43_state, cyw43_channel);
        cyw43_wifi_ap_set_auth(&cyw43_state, cyw43_auth);
        zephyr_cyw43_unlock(zephyr_cyw43_device);

        cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_AP, true, CYW43_COUNTRY_WORLDWIDE);

        return rv;
}

static int zephyr_cyw43_disable_ap(zephyr_cyw43_dev_t *zephyr_cyw43_device)
{
        cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_AP, false, CYW43_COUNTRY_WORLDWIDE);
        cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_WORLDWIDE);
        cyw43_state.itf_state &= ~(1 << CYW43_ITF_AP); /* cyw43_ctrl doesn't handle reset the itf state bit. */
        return 0;
}

static int zephyr_cyw43_set_pm(zephyr_cyw43_dev_t *zephyr_cyw43_device)
{
        int rv=0;
        rv = cyw43_wifi_pm(&cyw43_state, zephyr_cyw43_device->pm_params.pm_out);
        return rv;
}

static void zephyr_cyw43_request_work(struct k_work *item)
{
        zephyr_cyw43_dev_t *zephyr_cyw43_device;
        int err;

        zephyr_cyw43_device = CONTAINER_OF(item, zephyr_cyw43_dev_t, request_work);

        LOG_DBG("(req=%d)", zephyr_cyw43_device->req);

        switch (zephyr_cyw43_device->req) {
        case ZEPHYR_CYW43_REQ_CONNECT:
                err = zephyr_cyw43_connect(zephyr_cyw43_device);
                wifi_mgmt_raise_connect_result_event(zephyr_cyw43_device->iface, err);
                break;
        case ZEPHYR_CYW43_REQ_DISCONNECT:
                err = zephyr_cyw43_disconnect(zephyr_cyw43_device);
                wifi_mgmt_raise_disconnect_result_event(zephyr_cyw43_device->iface, err);
                break;
        case ZEPHYR_CYW43_REQ_PASSIVE_SCAN:
                err = zephyr_cyw43_scan(zephyr_cyw43_device, false);
                if (err) { LOG_ERR("Scan request returned error %d\n", err); }
                break;
        case ZEPHYR_CYW43_REQ_ACTIVE_SCAN:
                err = zephyr_cyw43_scan(zephyr_cyw43_device, true);
                if (err) { LOG_ERR("Scan request returned error %d\n", err); }
                break;
        case ZEPHYR_CYW43_REQ_ENABLE_AP:
                err = zephyr_cyw43_enable_ap(zephyr_cyw43_device);
                if (err) { LOG_ERR("Enable AP returned error %d\n", err); }
                break;
        case ZEPHYR_CYW43_REQ_DISABLE_AP:
                err = zephyr_cyw43_disable_ap(zephyr_cyw43_device);
                if (err) { LOG_ERR("Disable AP returned error %d\n", err); }
                break;
        case ZEPHYR_CYW43_REQ_SET_PM:
                err = zephyr_cyw43_set_pm(zephyr_cyw43_device);
                if (err) { LOG_ERR("Set PM returned error %d\n", err); }
                break;
        case ZEPHYR_CYW43_REQ_NONE:
        default:
                break;
        }
}

static int zephyr_cyw43_send(const struct device *dev, struct net_pkt *pkt)
{
        int rv;
        zephyr_cyw43_dev_t *zephyr_cyw43_device = dev->data;

        LOG_DBG("Calling zephyr_cyw43_send()  -  packet length=%d)\n", net_pkt_get_len(pkt));

        const int pkt_len = net_pkt_get_len(pkt);

        /* Read the packet payload */
        if (net_pkt_read(pkt, zephyr_cyw43_device->frame_buf, pkt_len) < 0) {
                rv = -EIO;
#if defined(CONFIG_NET_STATISTICS_WIFI)
                zephyr_cyw43_device->stats.errors.tx++;
#endif

                goto out;
        }

        rv = cyw43_send_ethernet(&cyw43_state, CYW43_ITF_STA, pkt_len, (void *)(zephyr_cyw43_device->frame_buf), false);

#if defined(CONFIG_NET_STATISTICS_WIFI)
        zephyr_cyw43_device->stats.bytes.sent += pkt_len;
        zephyr_cyw43_device->stats.pkts.tx++;
#endif

out:
        return rv;
}

static void zephyr_cyw43_iface_init(struct net_if *iface)
{
        struct ethernet_context *eth_ctx = net_if_l2_data(iface);

        LOG_DBG("Calling zephyr_cyw43_iface_init()\n");

        zephyr_cyw43_lock(zephyr_cyw43_dev);

        eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;
        zephyr_cyw43_dev->iface = iface;

        net_if_set_link_addr(iface, cyw43_state.mac, 6, NET_LINK_ETHERNET);

        ethernet_init(iface);

        net_if_carrier_on(iface);

        net_if_dormant_on(iface);

        net_dhcpv4_stop(iface);

        zephyr_cyw43_unlock(zephyr_cyw43_dev);

        return;
}

int zephyr_cyw43_iface_status(const struct device *dev,
                              struct wifi_iface_status *status)
{
        LOG_DBG("Calling iface_status()\n");

        zephyr_cyw43_dev_t *zephyr_cyw43_device = dev->data;

        status->iface_mode = (((cyw43_state.itf_state >> CYW43_ITF_AP) & 1) ? WIFI_MODE_AP :
                              ((cyw43_state.itf_state >> CYW43_ITF_STA) & 1) ? WIFI_MODE_INFRA :
                              WIFI_MODE_UNKNOWN);

        /*TODO: look harder to see if there's a way to actually retrieve this... until then,
          hard code to 802.11n */
        status->link_mode = WIFI_4;

        if (status->iface_mode == WIFI_MODE_INFRA) {
                int link_status=cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
                switch (link_status) {
                case CYW43_LINK_JOIN:
                        status->state = WIFI_STATE_COMPLETED;
                        break;
                case CYW43_LINK_NONET:
                        status->state = WIFI_STATE_INACTIVE;
                        break;
                default:
                        status->state = WIFI_STATE_DISCONNECTED;
                }
        }
        else if (status->iface_mode == WIFI_MODE_AP) {
                status->state = WIFI_STATE_COMPLETED;
        }
        else {
                status->state = WIFI_STATE_DISCONNECTED;
        }

        cyw43_wifi_get_bssid(&cyw43_state, (char *) &(status->bssid[0]));

        status->band = WIFI_FREQ_BAND_2_4_GHZ;

        cyw43_ioctl(&cyw43_state, CYW43_IOCTL_GET_CHANNEL, sizeof(status->channel),
                    (uint8_t *)&status->channel, (status->iface_mode == WIFI_MODE_INFRA ? CYW43_ITF_STA : CYW43_ITF_AP)); 

        if (status->iface_mode == WIFI_MODE_INFRA) {
                cyw43_wifi_get_rssi(&cyw43_state, (int32_t *) &(status->rssi));

                strcpy(status->ssid, (status->state == WIFI_STATE_COMPLETED ? zephyr_cyw43_device->connect_params.ssid : ""));
                status->ssid_len = strlen(zephyr_cyw43_device->connect_params.ssid);

                status->security = zephyr_cyw43_device->connect_params.security;
        }
        else if (status->iface_mode == WIFI_MODE_AP) {
                strcpy(status->ssid, (status->state == WIFI_STATE_COMPLETED ? zephyr_cyw43_device->ap_params.ssid : ""));
                status->ssid_len = strlen(zephyr_cyw43_device->ap_params.ssid);

                status->security = zephyr_cyw43_device->ap_params.security;
        }

        if (!cyw43_wifi_get_pm(&cyw43_state, &(zephyr_cyw43_device->pm_params.pm_in))) {
                if ((zephyr_cyw43_device->pm_params.pm_in & 0x0000000f) != CYW43_NO_POWERSAVE_MODE) {
                        status->beacon_interval = (zephyr_cyw43_device->pm_params.pm_in & 0x0000F000) >> 12;
                        status->dtim_period = (zephyr_cyw43_device->pm_params.pm_in & 0x000F0000) >> 16;
                }
        }
        else {
                LOG_ERR("cyw43_wifi_get_pm() returned error.\n"); 
        }

        status->twt_capable = false;

        return 0;
}

static int zephyr_cyw43_mgmt_scan(const struct device *dev,
                                  struct wifi_scan_params *params,
                                  scan_result_cb_t cb)
{

        zephyr_cyw43_dev_t *zephyr_cyw43_device = dev->data;

        LOG_DBG("Calling mgmt_scan()");

        zephyr_cyw43_lock(zephyr_cyw43_device);

        zephyr_cyw43_device->scan_cb = cb;

        zephyr_cyw43_device->req = (params->scan_type == WIFI_SCAN_TYPE_ACTIVE ?
                                    ZEPHYR_CYW43_REQ_ACTIVE_SCAN :
                                    ZEPHYR_CYW43_REQ_PASSIVE_SCAN);

        k_work_submit_to_queue(&zephyr_cyw43_device->work_q, &zephyr_cyw43_device->request_work);

        zephyr_cyw43_unlock(zephyr_cyw43_device);

        return 0;
}

static int zephyr_cyw43_mgmt_connect(const struct device *dev,
                                     struct wifi_connect_req_params *params)
{
        zephyr_cyw43_dev_t *zephyr_cyw43_device = dev->data;
        int rv=0;

        LOG_DBG("");

        if ((cyw43_state.itf_state >> CYW43_ITF_STA) & 1) {
                int link_status=cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);

                if (link_status == CYW43_LINK_JOIN || link_status == CYW43_LINK_NONET) {
                        LOG_ERR("Already connected.\n");
                        rv = -EAGAIN;
                        return rv;
                }
        }

        if ((cyw43_state.itf_state >> CYW43_ITF_AP) & 1) {
                LOG_ERR("Please disable access point mode before initiating a client connection.\n");
                rv = -EBUSY;
                return rv;
        }

        zephyr_cyw43_lock(zephyr_cyw43_device);

        /* Copy the relevant parameters into our own structure, because there's no
           guarantee (that I can see anyway) that the original one will not be deallocated. */
        strncpy(zephyr_cyw43_device->connect_params.ssid, params->ssid, params->ssid_length);
        zephyr_cyw43_device->connect_params.ssid[params->ssid_length] = '\0';
        strncpy(zephyr_cyw43_device->connect_params.psk, params->psk, params->psk_length);
        zephyr_cyw43_device->connect_params.psk[params->psk_length] = '\0';  
        zephyr_cyw43_device->connect_params.security = params->security;
        zephyr_cyw43_device->connect_params.channel = params->channel;

        zephyr_cyw43_device->req = ZEPHYR_CYW43_REQ_CONNECT;
        k_work_submit_to_queue(&zephyr_cyw43_device->work_q, &zephyr_cyw43_device->request_work);

        zephyr_cyw43_unlock(zephyr_cyw43_device);

        return rv;
}

static int zephyr_cyw43_mgmt_disconnect(const struct device *dev)
{
        zephyr_cyw43_dev_t *zephyr_cyw43_device = dev->data;
        LOG_DBG("");
        zephyr_cyw43_lock(zephyr_cyw43_device);
        zephyr_cyw43_device->req = ZEPHYR_CYW43_REQ_DISCONNECT;
        k_work_submit_to_queue(&zephyr_cyw43_device->work_q, &zephyr_cyw43_device->request_work);
        zephyr_cyw43_unlock(zephyr_cyw43_device);
        return 0;
}

static int zephyr_cyw43_mgmt_ap_enable(const struct device *dev,
                                       struct wifi_connect_req_params *params)
{
        zephyr_cyw43_dev_t *zephyr_cyw43_device = dev->data;
        int rv = 0;

        LOG_DBG("Calling mgmt_ap_enable()\n");

        if ((cyw43_state.itf_state >> CYW43_ITF_AP) & 1) {
                LOG_ERR("Already in AP mode..\n");
                rv = -EAGAIN;
                return rv;
        }

        if ((cyw43_state.itf_state >> CYW43_ITF_STA) & 1) {

                int link_status=cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);

                if (link_status == CYW43_LINK_JOIN || link_status == CYW43_LINK_NONET) {
                        LOG_ERR("Currently connected as a client. Please disconnect before enabling AP.\n");
                        rv = -EBUSY;
                        return rv;
                }
        }

        zephyr_cyw43_lock(zephyr_cyw43_device);

        strncpy(zephyr_cyw43_device->ap_params.ssid, params->ssid, params->ssid_length);
        zephyr_cyw43_device->ap_params.ssid[params->ssid_length] = '\0';
        strncpy(zephyr_cyw43_device->ap_params.psk, params->psk, params->psk_length);
        zephyr_cyw43_device->ap_params.psk[params->psk_length] = '\0';
        zephyr_cyw43_device->ap_params.security = params->security;
        zephyr_cyw43_device->ap_params.channel = params->channel;

        zephyr_cyw43_device->req = ZEPHYR_CYW43_REQ_ENABLE_AP;
        k_work_submit_to_queue(&zephyr_cyw43_device->work_q, &zephyr_cyw43_device->request_work);
        zephyr_cyw43_unlock(zephyr_cyw43_device);

        return rv;
}

static int zephyr_cyw43_mgmt_ap_disable(const struct device *dev)
{
        zephyr_cyw43_dev_t *zephyr_cyw43_device = dev->data;

        LOG_DBG("Calling mgmt_ap_disable()\n");
        zephyr_cyw43_lock(zephyr_cyw43_device);
        zephyr_cyw43_device->req = ZEPHYR_CYW43_REQ_DISABLE_AP;
        k_work_submit_to_queue(&zephyr_cyw43_device->work_q, &zephyr_cyw43_device->request_work);
        zephyr_cyw43_unlock(zephyr_cyw43_device);
        return 0;
}

static int zephyr_cyw43_mgmt_set_pm(const struct device *dev, struct wifi_ps_params *params)
{
        zephyr_cyw43_dev_t *zephyr_cyw43_device = dev->data;

        uint8_t pm_mode;
        uint16_t pm2_sleep_ret_ms = 0;
        uint8_t li_beacon_period = 0;
        uint8_t li_dtim_period = 0;
        uint8_t li_assoc = 0;

        LOG_DBG("");

        if (params->type == WIFI_PS_PARAM_STATE) {
                if (!params->enabled) {
                        pm_mode = CYW43_NO_POWERSAVE_MODE;
                }
                else {
                        if (CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_MODE == CYW43_NO_POWERSAVE_MODE) {
                                /* if we're dynamically going from off to on, then CYW43_PM2_POWERSAVE_MODE
                                   is the default. If you want to use CYW43_PM1_POWERSAVE_MODE, that has to be
                                   configured at build time with CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_MODE=1 */
                                pm_mode = CYW43_PM2_POWERSAVE_MODE;
                        }
                        else {
                                pm_mode = CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_MODE;
                        }
                }
                pm2_sleep_ret_ms = CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_PM2_SLEEP_RET_MS;
                li_beacon_period = CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_LI_BEACON_PERIOD;
                li_dtim_period = CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_LI_DTIM_PERIOD;
                li_assoc = CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_LI_ASSOC;
        }
        else {
                printk("set_power_save only used for enable/disable feature in this driver. Parameters\n"
                       "can be set using the CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_ build time\n"
                       "parameters.\n");
                return -1;
        }

        LOG_DBG("setting pm_out = cyw43_pm_value(pm_mode=%d, pm2_sleep_ret_ms=%d, li_beacon_period=%d, li_dtim_period=%d, li_assoc=%d)\n",
                pm_mode, pm2_sleep_ret_ms, li_beacon_period, li_dtim_period, li_assoc);
        zephyr_cyw43_lock(zephyr_cyw43_device);
        zephyr_cyw43_device->pm_params.pm_out = cyw43_pm_value(pm_mode, pm2_sleep_ret_ms, li_beacon_period, li_dtim_period, li_assoc);

        zephyr_cyw43_device->req = ZEPHYR_CYW43_REQ_SET_PM;
        k_work_submit_to_queue(&zephyr_cyw43_device->work_q, &zephyr_cyw43_device->request_work);
        zephyr_cyw43_unlock(zephyr_cyw43_device);

        return 0;
}

static int zephyr_cyw43_pm_status(const struct device *dev, struct wifi_ps_config *config)
{
        zephyr_cyw43_dev_t *zephyr_cyw43_device = dev->data;
        LOG_DBG("");

        uint8_t pm_mode;
        uint16_t pm2_sleep_ret_ms = 0;
        uint8_t li_beacon_period = 0;
        uint8_t li_dtim_period = 0;
        uint8_t li_assoc = 0;

        if (!cyw43_wifi_get_pm(&cyw43_state, &(zephyr_cyw43_device->pm_params.pm_in))) {
                LOG_DBG("zephyr_cyw43_device->pm_params.pm_in = %xu\n", zephyr_cyw43_device->pm_params.pm_in);
                zephyr_cyw43_lock(zephyr_cyw43_device);
                pm_mode = (uint8_t) (zephyr_cyw43_device->pm_params.pm_in & 0x0000000f);
                pm2_sleep_ret_ms = (uint16_t) ((zephyr_cyw43_device->pm_params.pm_in & 0x00000ff0) >> 4);
                li_beacon_period = (uint8_t) ((zephyr_cyw43_device->pm_params.pm_in & 0x00000ff0) >> 12);
                li_dtim_period = (uint8_t) ((zephyr_cyw43_device->pm_params.pm_in & 0x00000ff0) >> 16);
                li_assoc = (uint8_t) ((zephyr_cyw43_device->pm_params.pm_in & 0x00000ff0) >> 20);

                config->ps_params.enabled = (pm_mode == CYW43_NO_POWERSAVE_MODE ? false : true);
                if (config->ps_params.enabled) {
                        config->ps_params.wakeup_mode = (li_dtim_period ? WIFI_PS_WAKEUP_MODE_DTIM : WIFI_PS_WAKEUP_MODE_LISTEN_INTERVAL);
                        config->ps_params.timeout_ms = pm2_sleep_ret_ms * 10;
                }
                zephyr_cyw43_unlock(zephyr_cyw43_device);
        }
        else {
                LOG_ERR("cyw43_wifi_get_pm() returned error.\n");
        }

        return 0;
}

void cyw43_cb_tcpip_deinit(cyw43_t *self, int itf)
{
        LOG_DBG("Calling cyw43_cb_tcpip_deinit(itf=%d)\n", itf);
        return;
}

void cyw43_cb_process_ethernet(void *cb_data, int itf, size_t len, const uint8_t *buf)
{
        struct net_pkt *pkt;

        LOG_DBG("Calling cyw43_cb_process_ethernet(itf=%d, len=%d)\n", itf, len);

        if (zephyr_cyw43_dev->iface == NULL) {
                LOG_ERR("network interface unavailable");
                return;
        }

        pkt = net_pkt_rx_alloc_with_buffer(zephyr_cyw43_dev->iface, len, AF_UNSPEC, 0, K_MSEC(100));

        if (!pkt) {
                LOG_ERR("Failed to get net buffer");
                goto pkt_processing_succeeded;
        }

        if (net_pkt_write(pkt, buf, len) < 0) {
                LOG_ERR("Failed to write pkt");
                goto pkt_processing_failed;
        }

        if (net_recv_data(zephyr_cyw43_dev->iface, pkt) < 0) {
                LOG_ERR("Failed to push received data");
                goto pkt_processing_failed;
        }

#if defined(CONFIG_NET_STATISTICS_WIFI)
	zephyr_cyw43_dev->stats.bytes.received += len;
	zephyr_cyw43_dev->stats.pkts.rx++;
#endif

        goto pkt_processing_succeeded;

pkt_processing_failed:
#if defined(CONFIG_NET_STATISTICS_WIFI)
        zephyr_cyw43_dev->stats.errors.rx++;
#endif
        net_pkt_unref(pkt);

 pkt_processing_succeeded:
        return;
}

void cyw43_hal_generate_laa_mac(int idx, uint8_t buf[6])
{
        LOG_DBG("cyw43_hal_generate_laa_mac(idx=%d)", idx);
        return;
}

void cyw43_cb_tcpip_init(cyw43_t *self, int itf)
{
        LOG_DBG("cyw43_cb_tcpip_init(itf=%d)", itf);
        return;
}

void cyw43_cb_tcpip_set_link_up(cyw43_t *self, int itf)
{
        LOG_DBG("Calling cyw43_cb_tcpip_set_link_up(itf=%d)", itf);
        return;
}

void cyw43_cb_tcpip_set_link_down(cyw43_t *self, int itf)
{

        LOG_DBG("Calling cyw43_cb_tcpip_set_link_down(itf=%d)", itf);
        return;
}

void cyw43_schedule_internal_poll_dispatch(void (*func)(void)) {
        LOG_DBG("Calling cyw43_schedule_internal_poll_dispatch()");
        assert(func == cyw43_poll);
        return;
}

void cyw43_thread_enter(void)
{
        LOG_DBG("Calling cyw43_thread_enter()");
        zephyr_cyw43_lock(zephyr_cyw43_dev);
}

void cyw43_thread_exit(void)
{
        LOG_DBG("Calling cyw43_thread_exit()");
        zephyr_cyw43_unlock(zephyr_cyw43_dev);
}

void cyw43_delay_ms(uint32_t ms) {
        if (ms > 1000) {
                LOG_WRN("warning, calling cyw43_delay_ms() with value %d", ms); 
        }
        k_busy_wait(ms * 1000);
}

void sleep_ms(uint32_t ms) {
        if (ms > 1000) {
                LOG_WRN("warning, calling cyw43_delay_ms() with value %d\n", ms); 
        }

        k_sleep(K_MSEC(ms));
}

void cyw43_delay_us(uint32_t us) {
        if (us > 1000) {
                LOG_WRN("warning, calling cyw43_delay_ms() with value %d\n", us); 
        }

        k_busy_wait(us);
}

static void cyw43_set_irq_enabled(bool enabled)
{
        /* LOG_DBG("Calling cyw43_set_irq_enabled(%s)", (enabled ? "true" : "false")); */
        gpio_pin_interrupt_configure_dt(&zephyr_cyw43_cfg.irq_gpio, (enabled?(GPIO_INT_ENABLE|GPIO_INT_LEVEL_HIGH):GPIO_INT_DISABLE));
}

void cyw43_post_poll_hook(void)
{
        cyw43_set_irq_enabled(true);
}


struct gpio_callback zephyr_cyw43_gpio_cb;

static void zephyr_cyw43_isr(const struct device *port,
                             struct gpio_callback *cb,
                             gpio_port_pins_t pins)
{
        k_sem_give(&zephyr_cyw43_dev->event_sem);

        cyw43_set_irq_enabled(false);
}

static void zephyr_cyw43_register_cb()
{
        int rv;

        rv = gpio_pin_configure_dt(&zephyr_cyw43_cfg.irq_gpio, GPIO_INPUT);
        if (rv) {
                LOG_ERR("Unable to configure GPIO pin %u", zephyr_cyw43_cfg.irq_gpio.pin);
        }

        /* Note that this is technically a gpio_callback, not a true isr. */
        gpio_init_callback(&zephyr_cyw43_gpio_cb,
                           zephyr_cyw43_isr,
                           BIT(zephyr_cyw43_cfg.irq_gpio.pin));

        rv = gpio_add_callback(zephyr_cyw43_cfg.irq_gpio.port, &zephyr_cyw43_gpio_cb);
        if (rv) {
                LOG_ERR("gpio_add_callback() returned %d", rv);
        }

        cyw43_set_irq_enabled(true);
}

#if defined(CONFIG_NET_STATISTICS_WIFI)
static int zephyr_cyw43_wifi_stats(const struct device *dev, struct net_stats_wifi *stats)
{
        zephyr_cyw43_dev_t *zephyr_cyw43_device = dev->data;

        stats->bytes.received = zephyr_cyw43_device->stats.bytes.received;
        stats->bytes.sent = zephyr_cyw43_device->stats.bytes.sent;
        stats->pkts.rx = zephyr_cyw43_device->stats.pkts.rx;
        stats->pkts.tx = zephyr_cyw43_device->stats.pkts.tx;
        stats->errors.rx = zephyr_cyw43_device->stats.errors.rx;
        stats->errors.tx = zephyr_cyw43_device->stats.errors.tx;
        stats->broadcast.rx = zephyr_cyw43_device->stats.broadcast.rx;
        stats->broadcast.tx = zephyr_cyw43_device->stats.broadcast.tx;
        stats->multicast.rx = zephyr_cyw43_device->stats.multicast.rx;
        stats->multicast.tx = zephyr_cyw43_device->stats.multicast.tx;
        stats->sta_mgmt.beacons_rx = zephyr_cyw43_device->stats.sta_mgmt.beacons_rx;
        stats->sta_mgmt.beacons_miss = zephyr_cyw43_device->stats.sta_mgmt.beacons_miss;

        return 0;
}
#endif

static int zephyr_cyw43_init(const struct device *dev)
{
        zephyr_cyw43_dev_t *zephyr_cyw43_device = dev->data;

        LOG_DBG("");

        k_mutex_init(&zephyr_cyw43_device->mutex);

        zephyr_cyw43_device->role = ZEPHYR_CYW43_ROLE_CLIENT;

        k_sem_init(&zephyr_cyw43_device->event_sem, 0, 10);

        k_work_queue_start(&zephyr_cyw43_device->work_q, zephyr_cyw43_work_q_stack,
                           K_KERNEL_STACK_SIZEOF(zephyr_cyw43_work_q_stack),
                           CONFIG_SYSTEM_WORKQUEUE_PRIORITY - 1, NULL);

        k_work_init(&zephyr_cyw43_device->request_work, zephyr_cyw43_request_work);

        zephyr_cyw43_shell_register(zephyr_cyw43_device);

        cyw43_init(&cyw43_state);

        zephyr_cyw43_register_cb();

        k_tid_t thread_id = k_thread_create(&event_thread, zephyr_cyw43_event_poll_stack,
                                            EVENT_POLL_THREAD_STACK_SIZE,
                                            (k_thread_entry_t)zephyr_cyw43_event_poll_thread, (void *)dev, NULL,
                                            NULL, K_PRIO_COOP(EVENT_POLL_THREAD_PRIO), 0,
                                            K_NO_WAIT);
        k_thread_name_set(thread_id, "zephyr_cyw43_event_poll_thread");

        cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_WORLDWIDE);

        /* Set PM to our configurable parameters */
        cyw43_wifi_pm(&cyw43_state,
                      cyw43_pm_value(CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_MODE,
                                     CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_PM2_SLEEP_RET_MS,
                                     CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_LI_BEACON_PERIOD,
                                     CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_LI_DTIM_PERIOD,
                                     CONFIG_WIFI_ZEPHYR_CYW43_POWERSAVE_LI_ASSOC));

        return 0;

}

static const struct wifi_mgmt_ops zephyr_cyw43_mgmt_api = {
        .scan			   = zephyr_cyw43_mgmt_scan,
        .connect		   = zephyr_cyw43_mgmt_connect,
        .disconnect		   = zephyr_cyw43_mgmt_disconnect,
        .ap_enable		   = zephyr_cyw43_mgmt_ap_enable,
        .ap_disable		   = zephyr_cyw43_mgmt_ap_disable,
        .iface_status		   = zephyr_cyw43_iface_status,
#if defined(CONFIG_NET_STATISTICS_WIFI)
        .get_stats	           = zephyr_cyw43_wifi_stats,
#endif
        .set_power_save            = zephyr_cyw43_mgmt_set_pm,
        .get_power_save_config     = zephyr_cyw43_pm_status,
};

static const struct net_wifi_mgmt_offload zephyr_cyw43_callbacks = {
        .wifi_iface.iface_api.init = zephyr_cyw43_iface_init,
        .wifi_iface.send           = zephyr_cyw43_send,
        .wifi_mgmt_api             = &zephyr_cyw43_mgmt_api,
};

NET_DEVICE_DT_INST_DEFINE(0,
                          zephyr_cyw43_init, NULL,
                          &zephyr_cyw43_0, &zephyr_cyw43_cfg, CONFIG_WIFI_INIT_PRIORITY,
                          &zephyr_cyw43_callbacks, ETHERNET_L2,
                          NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);
