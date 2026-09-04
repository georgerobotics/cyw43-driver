/*
 * This file is part of the cyw43-driver
 *
 * Copyright (C) 2019-2022 George Robotics Pty Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Any redistribution, use, or modification in source or binary form is done
 *    solely for personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY THE LICENSOR AND COPYRIGHT OWNER "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE LICENSOR OR COPYRIGHT OWNER BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This software is also available for use with certain devices under different
 * terms, as set out in the top level LICENSE file.  For commercial licensing
 * options please email contact@georgerobotics.com.au.
 */

#ifndef CYW43_INCLUDED_CYW43_H
#define CYW43_INCLUDED_CYW43_H

#include "cyw43_config.h"

#if CYW43_LWIP
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#endif

#if CYW43_NETUTILS
#include "shared/netutils/dhcpserver.h"
#endif

#include "cyw43_ll.h"
#include <string.h>

/** \addtogroup cyw43_driver
 */
//!\{

/**
 * \file cyw43.h
 * \brief CYW43 driver interface
 */

/*!
 * \name CYW43 driver version as components
 * \brief Current version of the CYW43 driver as major/minor/micro components
 * \anchor CYW43_VERSION_
 */
//!\{
#define CYW43_VERSION_MAJOR 1
#define CYW43_VERSION_MINOR 1
#define CYW43_VERSION_MICRO 0
//!\}

/*!
 * \name CYW43 driver version
 * \brief Combined CYW43 driver version as a 32-bit number
 */
//!\{
#define CYW43_VERSION (CYW43_VERSION_MAJOR << 16 | CYW43_VERSION_MINOR << 8 | CYW43_VERSION_MICRO)
//!\}

/*!
 * \name Trace flags
 * \anchor CYW43_TRACE_
 */
//!\{
#define CYW43_TRACE_ASYNC_EV    (0x0001)
#define CYW43_TRACE_ETH_TX      (0x0002)
#define CYW43_TRACE_ETH_RX      (0x0004)
#define CYW43_TRACE_ETH_FULL    (0x0008)
#define CYW43_TRACE_MAC         (0x0010)
//!\}

/*!
 * \name Link status
 * \anchor CYW43_LINK_
 * \see cyw43_wifi_link_status() to get the wifi status
 * \see cyw43_tcpip_link_status() to get the overall link status
 */
//!\{
#define CYW43_LINK_DOWN         (0)     ///< link is down
#define CYW43_LINK_JOIN         (1)     ///< Connected to wifi
#define CYW43_LINK_NOIP         (2)     ///< Connected to wifi, but no IP address
#define CYW43_LINK_UP           (3)     ///< Connected to wifi with an IP address
#define CYW43_LINK_FAIL         (-1)    ///< Connection failed
#define CYW43_LINK_NONET        (-2)    ///< No matching SSID found (could be out of range, or down)
#define CYW43_LINK_BADAUTH      (-3)    ///< Authenticatation failure
//!\}

/*!
 * \name Interference mitigation mode
 * \anchor CYW43_INTERFERE_
 * The firmware default is \ref CYW43_INTERFERE_AUTO_NOISE.
 * \see cyw43_wifi_set_interference_mode() to set the mitigation mode
 * \see cyw43_wifi_get_interference_mode() to get the mitigation mode
 */
//!\{
#define CYW43_INTERFERE_NONE        (0)     ///< None/Disabled
#define CYW43_INTERFERE_NON_WLAN    (1)     ///< Non-WLAN
#define CYW43_INTERFERE_WLAN_MANUAL (2)     ///< Adjacent channel interference (ACI) mode
#define CYW43_INTERFERE_AUTO        (3)     ///< Automatic as determined by driver (ACI)
#define CYW43_INTERFERE_AUTO_NOISE  (4)     ///< Automatic as determined by driver with noise reduction (ACI)
//!\}

typedef struct _cyw43_t {
    cyw43_ll_t cyw43_ll;

    uint8_t itf_state;
    uint32_t trace_flags;

    // State for async events
    volatile uint32_t wifi_scan_state;
    uint32_t wifi_join_state;
    void *wifi_scan_env;
    int (*wifi_scan_cb)(void *, const cyw43_ev_scan_result_t *);

    bool initted;
    // Pending things to do
    bool pend_disassoc;
    bool pend_rejoin;
    bool pend_rejoin_wpa;

    // AP settings
    uint32_t ap_auth;
    uint8_t ap_channel;
    uint8_t ap_ssid_len;
    uint8_t ap_key_len;
    uint8_t ap_ssid[32];
    uint8_t ap_key[64];

    #if CYW43_LWIP
    // lwIP data
    struct netif netif[2];
    #if LWIP_IPV4 && LWIP_DHCP
    struct dhcp dhcp_client;
    #endif
    #endif

    #if CYW43_NETUTILS
    dhcp_server_t dhcp_server;
    #endif

    // mac from otp (or from cyw43_hal_generate_laa_mac if not set)
    uint8_t mac[6];

    #if CYW43_ENABLE_BLUETOOTH
    bool bt_loaded;
    #endif
    #if CYW43_ASSOC_CALLBACK
    void (*assoc_cb)(bool assoc);
    #endif
} cyw43_t;

extern cyw43_t cyw43_state;
extern void (*cyw43_poll)(void);
extern uint32_t cyw43_sleep;

/*!
 * \brief Initialize the driver
 *
 * This method must be called before using the driver
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 */
void cyw43_init(cyw43_t *self);

/*!
 * \brief Shut the driver down
 *
 * This method will close the network interfaces, and free up resources
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 */
void cyw43_deinit(cyw43_t *self);

/*!
 * \brief Send an ioctl command to cyw43
 *
 * This method sends a command to cyw43.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param cmd the command to send
 * \param len the amount of data to send with the command
 * \param buf a buffer containing the data to send
 * \param iface the interface to use, either CYW43_ITF_STA or CYW43_ITF_AP
 * \return 0 on success
 */
int cyw43_ioctl(cyw43_t *self, uint32_t cmd, size_t len, uint8_t *buf, uint32_t iface);

/*!
 * \brief Set an ioctl whose value is a single unsigned 32-bit integer
 *
 * Convenience wrapper around \ref cyw43_ioctl for the common case of an ioctl
 * that takes a 4-byte value, e.g. the various \c CYW43_IOCTL_SET_xxx commands.
 *
 * \param self  the driver state object. This should always be \c &cyw43_state
 * \param cmd   the ioctl command to send, e.g. one of the \c CYW43_IOCTL_SET_xxx defines
 * \param val   the value to set
 * \param iface the interface to use, either \ref CYW43_ITF_STA or \ref CYW43_ITF_AP
 * \return 0 on success, negative error code on failure
 */
int cyw43_ioctl_set_u32(cyw43_t *self, uint32_t cmd, uint32_t val, uint32_t iface);

/*!
 * \brief Get an ioctl whose value is a single unsigned 32-bit integer
 *
 * Convenience wrapper around \ref cyw43_ioctl for the common case of an ioctl
 * that returns a 4-byte value, e.g. the various \c CYW43_IOCTL_GET_xxx commands.
 *
 * \param self    the driver state object. This should always be \c &cyw43_state
 * \param cmd     the ioctl command to send, e.g. one of the \c CYW43_IOCTL_GET_xxx defines
 * \param out_val Output: the value that was read. Must not be NULL.
 * \param iface   the interface to use, either \ref CYW43_ITF_STA or \ref CYW43_ITF_AP
 * \return 0 on success, negative error code on failure
 */
int cyw43_ioctl_get_u32(cyw43_t *self, uint32_t cmd, uint32_t *out_val, uint32_t iface);

/*!
 * \brief Send a raw ethernet packet
 *
 * This method sends a raw ethernet packet.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param itf interface to use, either CYW43_ITF_STA or CYW43_ITF_AP
 * \param len the amount of data to send
 * \param buf the data to send
 * \param is_pbuf true if buf points to an lwip struct pbuf
 * \return 0 on success
 */
int cyw43_send_ethernet(cyw43_t *self, int itf, size_t len, const void *buf, bool is_pbuf);

/*!
 * \brief Set the WiFi interference mitigation mode
 *
 * Controls how aggressively the firmware attempts to mitigate interference
 * on the 2.4GHz band. Higher modes reduce interference at the cost of
 * RX sensitivity, which can be detrimental to long range operation.
 *
 * The firmware default is \ref CYW43_INTERFERE_AUTO_NOISE, the most
 * aggressive mode.
 *
 * \note This setting applies globally to the radio and affects both the
 * STA and AP interfaces in apsta mode.
 *
 * \param self  The driver state object, always \c &cyw43_state
 * \param mode  Interference mitigation mode, one of \ref CYW43_INTERFERE_
 * \return 0 on success, negative error code on failure
 */
static inline int cyw43_wifi_set_interference_mode(cyw43_t *self, uint32_t mode) {
    return cyw43_ioctl_set_u32(self, CYW43_IOCTL_SET_INTERFERENCE_MODE, mode, CYW43_ITF_STA);
}

/*!
 * \brief Get the current WiFi interference mitigation mode
 *
 * \param self  The driver state object, always \c &cyw43_state
 * \param mode  Output: current interference mitigation mode, one of \ref CYW43_INTERFERE_
 * \return 0 on success, negative error code on failure
 */
static inline int cyw43_wifi_get_interference_mode(cyw43_t *self, uint32_t *mode) {
    return cyw43_ioctl_get_u32(self, CYW43_IOCTL_GET_INTERFERENCE_MODE, mode, CYW43_ITF_STA);
}

/*!
 * \brief Set the wifi power management mode
 *
 * This method sets the power management mode used by cyw43.
 * This should be called after cyw43_wifi_set_up
 *
 * \see cyw43_pm_value
 * \see CYW43_DEFAULT_PM
 * \see CYW43_NONE_PM
 * \see CYW43_AGGRESSIVE_PM
 * \see CYW43_PERFORMANCE_PM
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param pm Power management value
 * \return 0 on success
 */
int cyw43_wifi_pm(cyw43_t *self, uint32_t pm);

/*!
 * \brief Get the wifi power management mode
 *
 * This method gets the power management mode used by cyw43.
 * The value is expressed as an unsigned integer 0x00adbrrm where,
 * m = pm_mode Power management mode
 * rr = pm2_sleep_ret (in units of 10ms)
 * b = li_beacon_period
 * d = li_dtim_period
 * a = li_assoc
 * \see cyw43_pm_value for an explanation of these values
 * This should be called after cyw43_wifi_set_up
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param pm Power management value
 * \return 0 on success
 */
int cyw43_wifi_get_pm(cyw43_t *self, uint32_t *pm);

/*!
 * \brief Get the wifi link status
 *
 * Returns the status of the wifi link.
 *
 * link status        | Meaning
 * -------------------|--------
 * CYW43_LINK_DOWN    | Wifi down
 * CYW43_LINK_JOIN    | Connected to wifi
 * CYW43_LINK_FAIL    | Connection failed
 * CYW43_LINK_NONET   | No matching SSID found (could be out of range, or down)
 * CYW43_LINK_BADAUTH | Authenticatation failure
 *
 * \note If the link status is negative it indicates an error
 * The wifi link status for the interface CYW43_ITF_AP is always CYW43_LINK_DOWN
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param itf the interface to use, should be CYW43_ITF_STA or CYW43_ITF_AP
 * \return A integer value representing the link status
 */
int cyw43_wifi_link_status(cyw43_t *self, int itf);

/*!
 * \brief Set up and initialise wifi
 *
 * This method turns on wifi and sets the country for regulation purposes.
 * The power management mode is initialised to \ref CYW43_DEFAULT_PM
 * For CYW43_ITF_AP, the access point is enabled.
 * For CYW43_ITF_STA, the TCP/IP stack is reinitialised
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param itf the interface to use either CYW43_ITF_STA or CYW43_ITF_AP
 * \param up true to enable the link. Set to false to disable AP mode.
 * Setting the \em up parameter to false for CYW43_ITF_STA is ignored.
 * \param country the country code, see \ref CYW43_COUNTRY_
 *
 */
void cyw43_wifi_set_up(cyw43_t *self, int itf, bool up, uint32_t country);

/*!
 * \brief Get the mac address of the device
 *
 * This method returns the mac address of the interface.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param itf the interface to use, either CYW43_ITF_STA or CYW43_ITF_AP
 * \param mac a buffer to receive the mac address
 * \return 0 on success
 */
int cyw43_wifi_get_mac(cyw43_t *self, int itf, uint8_t mac[6]);

/*!
 * \brief Add/remove multicast group address
 *
 * This method adds/removes an address from the multicast filter, allowing
 * frames sent to this group to be received
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param addr a buffer containing a group mac address
 * \param add true to add the address, false to remove it
 * \return 0 on success
 */
int cyw43_wifi_update_multicast_filter(cyw43_t *self, uint8_t *addr, bool add);

/*!
 * \brief Perform a wifi scan for wifi networks
 *
 * Start a scan for wifi networks. Results are returned via the callback.
 *
 * \note The scan is complete when \ref cyw43_wifi_scan_active return false
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param opts An instance of \ref cyw43_wifi_scan_options_t. Values in here are currently ignored.
 * \param env Pointer passed back in the callback
 * \param result_cb Callback for wifi scan results, see \ref cyw43_ev_scan_result_t
 * \return 0 on success
 */
int cyw43_wifi_scan(cyw43_t *self, cyw43_wifi_scan_options_t *opts, void *env, int (*result_cb)(void *, const cyw43_ev_scan_result_t *));

/*!
 * \brief Determine if a wifi scan is in progress
 *
 * This method tells you if the scan is still in progress
 *
 * \param self the driver state object. This should always be  \c &cyw43_state
 * \return true if a wifi scan is in progress
 */
static inline bool cyw43_wifi_scan_active(cyw43_t *self) {
    return self->wifi_scan_state == 1;
}

/*!
 * \brief Connect or \em join a wifi network
 *
 * Connect to a wifi network in STA (client) mode
 * After success is returned, periodically call \ref cyw43_wifi_link_status or \ref cyw43_tcpip_link_status,
 * to query the status of the link. It can take a many seconds to connect to fully join a network.
 *
 * \note Call \ref cyw43_wifi_leave to disassociate from a wifi network.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param ssid_len the length of the wifi network name
 * \param ssid A buffer containing the wifi network name
 * \param key_len The length of the wifi \em password
 * \param key A buffer containing the wifi \em password
 * \param auth_type Auth type, \see CYW43_AUTH_
 * \param bssid the mac address of the access point to connect to. This can be NULL.
 * \param channel Used to set the band of the connection. This is only used if bssid is non NULL.
 * \return 0 on success
 */
int cyw43_wifi_join(cyw43_t *self, size_t ssid_len, const uint8_t *ssid, size_t key_len, const uint8_t *key, uint32_t auth_type, const uint8_t *bssid, uint32_t channel);

/*!
 * \brief Disassociate from a wifi network
 *
 * This method disassociates from a wifi network.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param itf The interface to disconnect, either CYW43_ITF_STA or CYW43_ITF_AP
 * \return 0 on success
 */
int cyw43_wifi_leave(cyw43_t *self, int itf);

/*!
 * \brief Enable or disable WiFi roaming on the STA interface
 *
 * When disabled, the firmware will remain associated to its current AP
 * regardless of signal strength. When enabled, roaming behaviour is
 * governed by the parameters set in \c cyw43_wifi_set_roam_params.
 *
 * Roaming is enabled by default.
 *
 * \param self     The driver state object, always \c &cyw43_state
 * \param enabled  true to enable roaming, false to disable
 * \return 0 on success, negative error code on failure
 */
int cyw43_wifi_set_roam_enabled(cyw43_t *self, bool enabled);

/*!
 * \brief Query whether WiFi roaming is enabled on the STA interface
 *
 * Reads back the current state of the \c roam_off iovar.
 *
 * \param self     The driver state object, always \c &cyw43_state
 * \param enabled  Output: true if roaming is enabled, false if disabled
 * \return 0 on success, negative error code on failure
 */
int cyw43_wifi_get_roam_enabled(cyw43_t *self, bool *enabled);

/*!
 * \brief Configure WiFi roaming parameters for the STA interface
 *
 * Controls when the firmware will scan for and switch to a better AP.
 * Roaming is a STA-mode only feature - these parameters have no effect
 * when called on the AP interface.
 *
 * The firmware begins scanning for alternative APs when the current AP's
 * RSSI drops below \p trigger_dbm. A candidate AP must have an RSSI at
 * least \p candidate_delta_db higher than the current AP before the
 * firmware will roam to it. This prevents thrashing between APs of
 * similar signal strength.
 *
 * The firmware defaults are a trigger of -75dBm, a delta of 20dB and a scan
 * period of 10.
 *
 * \param self          The driver state object, always \c &cyw43_state
 * \param trigger_dbm   RSSI threshold in dBm below which roam scanning begins.
 *                      Must be negative. Default: -75.
 * \param candidate_delta_db  Minimum RSSI improvement in dB a candidate AP
 *                      must offer over the current AP to trigger a roam.
 *                      Default: 20.
 * \param scan_period   How often the firmware scans for better APs while
 *                      below the trigger threshold. Default: 10. The unit is
 *                      not documented by the vendor; it is believed to be
 *                      seconds, and the firmware range checks nothing.
 * \return 0 on success, negative error code on failure
 */
static inline int cyw43_wifi_set_roam_params(cyw43_t *self, int32_t trigger_dbm, int32_t candidate_delta_db, int32_t scan_period) {
    int ret = cyw43_ioctl_set_u32(self, CYW43_IOCTL_SET_ROAM_TRIGGER, (uint32_t)trigger_dbm, CYW43_ITF_STA);
    if (ret) {
        return ret;
    }
    ret = cyw43_ioctl_set_u32(self, CYW43_IOCTL_SET_ROAM_DELTA, (uint32_t)candidate_delta_db, CYW43_ITF_STA);
    if (ret) {
        return ret;
    }
    return cyw43_ioctl_set_u32(self, CYW43_IOCTL_SET_ROAM_SCAN_PERIOD, (uint32_t)scan_period, CYW43_ITF_STA);
}

/*!
 * \brief Retrieve the current WiFi roaming parameters for the STA interface
 *
 * Reads back the roaming parameters that are currently set.
 * Any pointer may be NULL if that parameter is not required.
 *
 * \param self                The driver state object, always \c &cyw43_state
 * \param trigger_dbm         Output: RSSI threshold in dBm below which roam
 *                            scanning begins. Will be a negative value.
 * \param candidate_delta_db  Output: Minimum RSSI improvement in dB a candidate
 *                            AP must offer over the current AP to trigger a roam.
 * \param scan_period         Output: How often the firmware scans for better
 *                            APs while below the trigger threshold.
 * \return 0 on success, negative error code on failure
 */
static inline int cyw43_wifi_get_roam_params(cyw43_t *self, int32_t *trigger_dbm, int32_t *candidate_delta_db, int32_t *scan_period) {
    int ret;
    if (trigger_dbm != NULL) {
        ret = cyw43_ioctl_get_u32(self, CYW43_IOCTL_GET_ROAM_TRIGGER, (uint32_t *)trigger_dbm, CYW43_ITF_STA);
        if (ret) {
            return ret;
        }
    }
    if (candidate_delta_db != NULL) {
        ret = cyw43_ioctl_get_u32(self, CYW43_IOCTL_GET_ROAM_DELTA, (uint32_t *)candidate_delta_db, CYW43_ITF_STA);
        if (ret) {
            return ret;
        }
    }
    if (scan_period != NULL) {
        ret = cyw43_ioctl_get_u32(self, CYW43_IOCTL_GET_ROAM_SCAN_PERIOD, (uint32_t *)scan_period, CYW43_ITF_STA);
        if (ret) {
            return ret;
        }
    }
    return 0;
}

/*!
 * \brief Get the signal strength (RSSI) of the wifi network
 *
 * For STA (client) mode, returns the signal strength or RSSI of the wifi network.
 * An RSSI value of zero is returned if you call this function before a network is connected.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param rssi a pointer to which the returned RSSI value is stored.
 * \return 0 on success
 */
int cyw43_wifi_get_rssi(cyw43_t *self, int32_t *rssi);

/*!
 * \brief Get the BSSID of the connected wifi network
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param bssid a buffer to receive the BSSID
 * \return 0 on success
 */
int cyw43_wifi_get_bssid(cyw43_t *self, uint8_t bssid[6]);

/*!
 * \brief Get the ssid for the access point
 *
 * For access point (AP) mode, this method can be used to get the SSID name of the wifi access point.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param len Returns the length of the AP SSID name
 * \param buf Returns a pointer to an internal buffer containing the AP SSID name
 */
static inline void cyw43_wifi_ap_get_ssid(cyw43_t *self, size_t *len, const uint8_t **buf) {
    *len = self->ap_ssid_len;
    *buf = self->ap_ssid;
}

/*!
 * \brief Get the security authorisation used in AP mode
 *
 * For access point (AP) mode, this method can be used to get the security authorisation mode.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \return the current security authorisation mode for the access point
 */
static inline uint32_t cyw43_wifi_ap_get_auth(cyw43_t *self) {
    return self->ap_auth;
}

/*!
 * \brief Set the the channel for the access point
 *
 * For access point (AP) mode, this method can be used to set the channel used for the wifi access point.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param channel Wifi channel to use for the wifi access point
 */
static inline void cyw43_wifi_ap_set_channel(cyw43_t *self, uint32_t channel) {
    self->ap_channel = (uint8_t)channel;
}

/*!
 * \brief Set the ssid for the access point
 *
 * For access point (AP) mode, this method can be used to set the SSID name of the wifi access point.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param len The length of the AP SSID name
 * \param buf A buffer containing the AP SSID name
 */
static inline void cyw43_wifi_ap_set_ssid(cyw43_t *self, size_t len, const uint8_t *buf) {
    self->ap_ssid_len = (uint8_t)MIN(len, sizeof(self->ap_ssid));
    memcpy(self->ap_ssid, buf, self->ap_ssid_len);
}

/*!
 * \brief Set the password for the wifi access point
 *
 * For access point (AP) mode, this method can be used to set the password for the wifi access point.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param len The length of the AP password
 * \param buf A buffer containing the AP password
 */
static inline void cyw43_wifi_ap_set_password(cyw43_t *self, size_t len, const uint8_t *buf) {
    self->ap_key_len = (uint8_t)MIN(len, sizeof(self->ap_key));
    memcpy(self->ap_key, buf, self->ap_key_len);
}

/*!
 * \brief Set the security authorisation used in AP mode
 *
 * For access point (AP) mode, this method can be used to set how access to the access point is authorised.
 *
 * Auth mode                 | Meaning
 * --------------------------|--------
 * CYW43_AUTH_OPEN           | Use an open access point with no authorisation required
 * CYW43_AUTH_WPA_TKIP_PSK   | Use WPA authorisation
 * CYW43_AUTH_WPA2_AES_PSK   | Use WPA2 (preferred)
 * CYW43_AUTH_WPA2_MIXED_PSK | Use WPA2/WPA mixed (currently treated the same as \ref CYW43_AUTH_WPA2_AES_PSK)
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param auth Auth mode for the access point
 */
static inline void cyw43_wifi_ap_set_auth(cyw43_t *self, uint32_t auth) {
    self->ap_auth = auth;
}

/*!
 * \brief Get the maximum number of devices (STAs) that can be associated with the wifi access point
 *
 * For access point (AP) mode, this method can be used to get the maximum number of devices that can be
 * connected to the wifi access point.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param max_stas Returns the maximum number of devices (STAs) that can be connected to the access point
 * (set to 0 on error)
 */
void cyw43_wifi_ap_get_max_stas(cyw43_t *self, int *max_stas);

/*!
 * \brief Get the number of devices (STAs) associated with the wifi access point
 *
 * For access point (AP) mode, this method can be used to get the number of devices and mac addresses of devices
 * connected to the wifi access point.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param num_stas Caller must provide the number of MACs that will fit in the macs buffer;
 * The supplied buffer should have enough room for 6 bytes per MAC address.
 * Returns the number of devices (STA) connected to the access point.
 * \param macs Returns up to num_stas MAC addresses of devices (STA) connected to the access point.
 * Call \ref cyw43_wifi_ap_get_max_stas to determine how many mac addresses can be returned.
 */
void cyw43_wifi_ap_get_stas(cyw43_t *self, int *num_stas, uint8_t *macs);

/*!
 * \brief Determines if the cyw43 driver been initialised
 *
 * Returns true if the cyw43 driver has been initialised with a call to \ref cyw43_init
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \return True if the cyw43 driver has been initialised
 */
static inline bool cyw43_is_initialized(cyw43_t *self) {
    return self->initted;
}

/*!
 * \brief Initialise the IP stack
 *
 * This method must be provided by the network stack interface
 * It is called to initialise the IP stack.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param itf the interface used, either CYW43_ITF_STA or CYW43_ITF_AP
 */
void cyw43_cb_tcpip_init(cyw43_t *self, int itf);

/*!
 * \brief Deinitialise the IP stack
 *
 * This method must be provided by the network stack interface
 * It is called to close the IP stack and free resources.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param itf the interface used, either CYW43_ITF_STA or CYW43_ITF_AP
 */
void cyw43_cb_tcpip_deinit(cyw43_t *self, int itf);

/*!
 * \brief Notify the IP stack that the link is up
 *
 * This method must be provided by the network stack interface
 * It is called to notify the IP stack that the link is up.
 * This can, for example be used to request an IP address via DHCP.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param itf the interface used, either CYW43_ITF_STA or CYW43_ITF_AP
 */
void cyw43_cb_tcpip_set_link_up(cyw43_t *self, int itf);

/*!
 * \brief Notify the IP stack that the link is down
 *
 * This method must be provided by the network stack interface
 * It is called to notify the IP stack that the link is down.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param itf the interface used, either CYW43_ITF_STA or CYW43_ITF_AP
 */
void cyw43_cb_tcpip_set_link_down(cyw43_t *self, int itf);

/*!
 * \brief Get the link status
 *
 * Returns the status of the link which is a superset of the wifi link status returned by \ref cyw43_wifi_link_status
 * \note If the link status is negative it indicates an error
 *
 * link status        | Meaning
 * -------------------|--------
 * CYW43_LINK_DOWN    | Wifi down
 * CYW43_LINK_JOIN    | Connected to wifi
 * CYW43_LINK_NOIP    | Connected to wifi, but no IP address
 * CYW43_LINK_UP      | Connect to wifi with an IP address
 * CYW43_LINK_FAIL    | Connection failed
 * CYW43_LINK_NONET   | No matching SSID found (could be out of range, or down)
 * CYW43_LINK_BADAUTH | Authenticatation failure
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param itf the interface for which to return the link status, should be CYW43_ITF_STA or CYW43_ITF_AP
 * \return A value representing the link status
 */
int cyw43_tcpip_link_status(cyw43_t *self, int itf);

#if CYW43_GPIO
/*!
 * \brief Set the value of the cyw43 gpio
 *
 * Set the value of a cyw43 gpio.
 * \note Check the datasheet for the number and purpose of the cyw43 gpios.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param gpio number of the gpio to set
 * \param val value for the gpio
 * \return 0 on success
 */
int cyw43_gpio_set(cyw43_t *self, int gpio, bool val);

/*!
 * \brief Get the value of the cyw43 gpio
 *
 * Get the value of a cyw43 gpio.
 * \note Check the datasheet for the number and purpose of the cyw43 gpios.
 *
 * \param self the driver state object. This should always be \c &cyw43_state
 * \param gpio number of the gpio to get
 * \param val Returns the value of the gpio
 * \return 0 on success
 */
int cyw43_gpio_get(cyw43_t *self, int gpio, bool *val);
#endif

/*!
 * \brief Return a power management value to pass to cyw43_wifi_pm
 *
 * Generate the power management (PM) value to pass to cyw43_wifi_pm
 *
 * pm_mode                  | Meaning
 * -------------------------|--------
 * CYW43_NO_POWERSAVE_MODE  | No power saving
 * CYW43_PM1_POWERSAVE_MODE | Aggressive power saving which reduces wifi throughput
 * CYW43_PM2_POWERSAVE_MODE | Power saving with High throughput (preferred). Saves power when there is no wifi activity for some time.
 *
 * \see \ref CYW43_DEFAULT_PM
 * \see \ref CYW43_NONE_PM
 * \see \ref CYW43_AGGRESSIVE_PM
 * \see \ref CYW43_PERFORMANCE_PM
 *
 * \param pm_mode Power management mode
 * \param pm2_sleep_ret_ms The maximum time to wait before going back to sleep for CYW43_PM2_POWERSAVE_MODE mode.
 * Value measured in milliseconds and must be between 10 and 2000ms and divisible by 10
 * \param li_beacon_period Wake period is measured in beacon periods
 * \param li_dtim_period Wake interval measured in DTIMs. If this is set to 0, the wake interval is measured in beacon periods
 * \param li_assoc Wake interval sent to the access point
 */
static inline uint32_t cyw43_pm_value(uint8_t pm_mode, uint16_t pm2_sleep_ret_ms, uint8_t li_beacon_period, uint8_t li_dtim_period, uint8_t li_assoc) {
    return li_assoc << 20 // listen interval sent to ap
           | li_dtim_period << 16
           | li_beacon_period << 12
           | (pm2_sleep_ret_ms / 10) << 4 // cyw43_ll_wifi_pm multiplies this by 10
           | pm_mode; // CYW43_PM2_POWERSAVE_MODE etc
}

/*!
 * \brief Default power management mode
 */
#define CYW43_DEFAULT_PM (CYW43_PERFORMANCE_PM)

/*!
 * \brief No power management
 */
#define CYW43_NONE_PM (cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 10, 0, 0, 0))

/*!
 * \brief Aggressive power management mode for optimal power usage at the cost of performance
 */
#define CYW43_AGGRESSIVE_PM (cyw43_pm_value(CYW43_PM1_POWERSAVE_MODE, 10, 0, 0, 0))

/*!
 * \brief Performance power management mode where more power is used to increase performance
 */
#define CYW43_PERFORMANCE_PM (cyw43_pm_value(CYW43_PM2_POWERSAVE_MODE, 200, 1, 1, 10))

#if CYW43_ENABLE_BLUETOOTH

/*!
 * \brief Initialise the Bluetooth HCI layer
 *
 * \return zero on success
 */
int cyw43_bluetooth_hci_init(void);

/*!
 * \brief Read data from the Bluetooth HCI layer
 *
 * \param buf Buffer to be filled with hci data
 * \param max_size The maximum size of the buffer
 * \param len Returns the length of the data in the buffer including an initial 4 byte header. The last byte of the header is the packet type
 * \return zero on success
 */
int cyw43_bluetooth_hci_read(uint8_t *buf, uint32_t max_size, uint32_t *len);

/*!
 * \brief Write data to the Bluetooth HCI layer
 *
 * \param buf Data to write
 * \param len Size of data to send. Must include a 4 byte header. The last byte of the header should be the packet type
 * \return zero on success
 */
int cyw43_bluetooth_hci_write(uint8_t *buf, size_t len);

/*!
 * \brief Callback for the Bluetooth HCI layer to do processing
 */
void cyw43_bluetooth_hci_process(void);

#endif // CYW43_ENABLE_BLUETOOTH

#if CYW43_ENABLE_BLUETOOTH_OVER_UART

/*!
 * \brief Initialise the cyw43's Bluetooth controller.
 *
 * The UART must be initialised and have flow control enabled before calling this function.
 *
 * \return zero on success
 */
int cyw43_bluetooth_controller_init(void);

/*!
 * \brief Deinitialise the cyw43's Bluetooth controller.
 *
 * \return zero on success
 */
int cyw43_bluetooth_controller_deinit(void);

/*!
 * \brief Tell the Bluetooth controller to go to sleep.
 *
 * \return zero on success
 */
int cyw43_bluetooth_controller_sleep_maybe(void);

/*!
 * \brief Query whether the controller woke up.
 *
 * \return true if the controller woke up, false otherwise
 */
bool cyw43_bluetooth_controller_woken(void);

/*!
 * \brief Wake up the Bluetooth controller.
 *
 * \return zero on success
 */
int cyw43_bluetooth_controller_wakeup(void);

#endif // CYW43_ENABLE_BLUETOOTH_OVER_UART

//!\} // cyw43_driver doxygen group

#endif // CYW43_INCLUDED_CYW43_H
