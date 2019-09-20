/**
 * SeCret Setting Service
 */

#ifndef BLE_SSS_H__
#define BLE_SSS_H__

#include <stdint.h>
#include <string.h>
#include "ble_cmds.h"
#include "ble.h"
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "ble_nus.h"


#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

uint32_t ble_sss_init(void);
uint32_t ble_sss_command_handler(const uint8_t *data, const uint16_t length);
void ble_sss_send_response_ok(const uint8_t cmd, const uint8_t seq_num);
void ble_sss_send_response_ok_with_param(const uint8_t cmd, const uint8_t seq_num, const uint8_t param);
void ble_sss_send_response_ng(const uint8_t cmd, const uint8_t seq_num, const uint8_t cmd_err_code);
void ble_sss_send_data(const uint8_t cmd, const uint8_t seq_num, void *p_data, uint32_t len);

/* File ID and Key used for the configuration record. */
#define CONFIG_FILE              (0xF010)
#define CONFIG_VIB_MUTE_KEY      (0x3000)
#define CONFIG_VIB_STRENGTH_KEY  (0x3001)
#define CONFIG_SLEEP_MODE_KEY    (0x3002)
//2019.09.20 Add Led & Vib setting option Start
#define CONFIG_NOTIFY_MODE_KEY   (0x3003)
//2019.09.20 Add Led & Vib setting option End
#define CONFIG_COMMON_TEL_KEY    (0x5001)
#define CONFIG_COMMON_EMAIL_KEY  (0x5002)
#define CONFIG_COMMON_LINE_KEY   (0x5003)
#define CONFIG_COMMON_TW_KEY     (0x5004)
#define CONFIG_COMMON_FB_KEY     (0x5005)
#define CONFIG_COMMON_INSTA_KEY  (0x5006)
#define CONFIG_COMMON_OTHER_KEY  (0x5099)
#define CONFIG_PTN_01_KEY        (0x7001)
#define CONFIG_PTN_02_KEY        (0x7002)
#define CONFIG_PTN_03_KEY        (0x7003)
#define CONFIG_PTN_04_KEY        (0x7004)
#define CONFIG_PTN_05_KEY        (0x7005)
#define CONFIG_PTN_06_KEY        (0x7006)
#define CONFIG_PTN_07_KEY        (0x7008)

typedef struct
{
    uint8_t pattern_id;
    uint8_t notif_category_id;
    char     device_name[64];
} pattern_conf_t;

ret_code_t record_write(uint32_t fid, uint32_t key, void const *p_data, uint32_t len);
ret_code_t record_delete(uint32_t fid, uint32_t key);
ret_code_t record_read(uint32_t fid, uint32_t key, uint8_t *p_data, uint32_t *p_data_len);

ret_code_t ble_sss_delete_bonds(void);
ret_code_t ble_sss_shutdown(void);

void fds_debug_error(const ret_code_t err_code);
void fds_debug_print(const uint8_t *data, const uint16_t data_len);

ret_code_t ble_sss_load_settings(void);

void ble_sss_set_ptn_setting_cache(uint32_t key, uint8_t *p_data, uint32_t p_data_len);
ret_code_t ble_sss_pattern_record_read(uint32_t fid, uint32_t key, uint8_t *p_data, uint32_t *p_data_len);

bool ble_sss_get_all_tel(void);
bool ble_sss_get_all_email(void);
bool ble_sss_get_all_line(void);
bool ble_sss_get_all_twitter(void);
bool ble_sss_get_all_facebook(void);
bool ble_sss_get_all_instagram(void);
bool ble_sss_get_all_others(void);
//2019.09.20 Add Led & Vib setting option Start
bool ble_sss_get_notif_Led_vib(void);
bool ble_sss_set_notif_Led_vib(bool notif_Led_vib_flg);
//2019.09.20 Add Led & Vib setting option End

#endif // BLE_SSS_H__
