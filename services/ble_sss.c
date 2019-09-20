#include "ble_sss.h"

#define NRF_LOG_MODULE_NAME ble_sss
#define NRF_LOG_LEVEL 4 // 4=DEBUG, 3=INFO
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "nrf_soc.h"
#include "peer_manager.h"
#include "ble_nus.h"
#include "fds.h"
// #include "fds_config.h"
#include "led.h"
#include "ble_dfu.h"

#include "vibration.h"

#define SSS_CLR_BUF(array)   for (uint16_t i=0; i < FDS_VIRTUAL_PAGE_SIZE * 4; i++) {array[i] = 0;}
#define SSS_COPY_BUF(target_array, src_array, offset, length)   for(uint16_t j=offset; j < length; j++) {target_array[j-offset] = src_array[j];}

BLE_NUS_DEF(m_nus);                                                                 /**< BLE NUS service instance. */

/* Defined in main.c */
extern char const * fds_err_str[];
static uint8_t m_payload_data[FDS_VIRTUAL_PAGE_SIZE * 4] = {0x0,};
static uint8_t m_loaded_data[FDS_VIRTUAL_PAGE_SIZE * 4] = {0x0,};

static bool m_notif_all_tel = true;
static bool m_notif_all_email = true;
static bool m_notif_all_line = true;
static bool m_notif_all_twitter = true;
static bool m_notif_all_facebook = true;
static bool m_notif_all_instagram = true;
static bool m_notif_all_others = true;

//2019.09.20 Add Led & Vib setting option Start
static bool m_notif_Led_vib = false; //true:Vib false:Led 
//2019.09.20 Add Led & Vib setting option End

/* Defined in main.c */
extern uint16_t m_wait_tick;
extern bool     m_sleep_mode;
extern bool     m_enter_sleep_mode_requested;
extern bool     m_exit_sleep_mode_requested;

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        err_code = ble_sss_command_handler(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        if (err_code != NRF_SUCCESS) {
            NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
            // APP_ERROR_CHECK(err_code);
        }
    }

}

/**@brief Function for preparing the reset, disabling SoftDevice and jump to the bootloader.
 */
static void bootloader_start(void)
{
    ble_dfu_buttonless_bootloader_start_finalize();
}

/**@brief Function for initializing services that will be used by the application.
 */
uint32_t ble_sss_init(void)
{
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    return ble_nus_init(&m_nus, &nus_init);
}

uint32_t ble_sss_command_handler(const uint8_t *data, const uint16_t length)
{
    uint32_t ret_val = NRF_SUCCESS;
    uint8_t cmd_header = data[0];

    // Long Data Command FIXME
    // if (cmd_header == SSS_CMD_MULTI_DATA) {
    //     if (length < 3) {
    //         NRF_LOG_ERROR("Invalid command data length");
    //         return NRF_ERROR_INVALID_LENGTH;
    //     }
    //     uint8_t data_len = data[1];
    //     uint8_t data_offset = data[2];
    //     for(uint16_t m=data_offset, n=3; m < (data_offset + data_len); m++, n++) {
    //         m_payload_data[m] = data[n];
    //     }

    //     return ret_val;
    // }

    // Normal Command
    if (length < 5) {
        NRF_LOG_ERROR("Invalid command data length");
        return NRF_ERROR_INVALID_LENGTH;
    }

    uint8_t cmd_id = data[1];
    uint8_t seq_num = data[2];
    uint8_t param_header = data[3];

    if (cmd_header != SSS_CMD_SEQ_START || param_header != SSS_CMD_SEQ_PARAM) {
        NRF_LOG_ERROR("Invalid command data format");
        return NRF_ERROR_INVALID_DATA;
    } 

    switch(cmd_id)
    {
        case SSS_CMD_BLINK_LED:
        {
            uint8_t count = data[4];
            NRF_LOG_INFO("[CMD] SSS_CMD_BLINK_LED : sequence=%d, count=%d", seq_num, count);
            if (count > 20) {
                count = 20; // Max 20
            }
            led_blink(count);
            ble_sss_send_response_ok(cmd_id, seq_num);
            break;
        }

        case SSS_CMD_VIBRATION:
        {
            uint8_t duration = data[4];
            NRF_LOG_INFO("[CMD] SSS_CMD_VIBRATION : sequence=%d, sec=%d", seq_num, duration);
            if (duration > 10) {
                duration = 10; // Max 10
            }
            vib_on_sec(duration);
            ble_sss_send_response_ok(cmd_id, seq_num);
            break;
        }

        case SSS_CMD_EXEC_VIB_PATTERN:
        {
            uint8_t pattern_id = data[4];
            if (pattern_id > 7) {
                pattern_id = 7; // Max 7
            }
            ble_sss_send_response_ok(cmd_id, seq_num);
            vib_play_pattern(pattern_id);
            NRF_LOG_INFO("[CMD] SSS_CMD_EXEC_VIB_PATTERN : sequence=%d, pattern_id=%d", seq_num, pattern_id);
            break;
        }

        case SSS_CMD_COMMON_VIB_MUTE_SET:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_COMMON_VIB_MUTE_SET : sequence=%d", seq_num);
            uint8_t mute_id = data[4];
            if (mute_id != 1) {
                mute_id = 0; // 1以外は全て0にする
            }
            SSS_CLR_BUF(m_payload_data);
            m_payload_data[0] = mute_id;
            uint16_t payload_len = 1;
            ret_val = record_write(CONFIG_FILE, CONFIG_VIB_MUTE_KEY, m_payload_data, payload_len);
            if (ret_val == NRF_SUCCESS) {
                vib_set_mute( mute_id );
                ble_sss_send_response_ok(cmd_id, seq_num);
            } else {
                ble_sss_send_response_ng(cmd_id, seq_num, ret_val);
            }
            break;
        }

        case SSS_CMD_COMMON_VIB_MUTE_GET:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_COMMON_VIB_MUTE_GET : sequence=%d", seq_num);
            uint32_t data_len;
            SSS_CLR_BUF(m_loaded_data);
            ret_val = record_read(CONFIG_FILE, CONFIG_VIB_MUTE_KEY, m_loaded_data, &data_len);
            if (ret_val == NRF_SUCCESS) {
                vib_set_mute( m_loaded_data[0] );
            }
            uint8_t mute_value = vib_get_mute();
            ble_sss_send_response_ok_with_param(cmd_id, seq_num, mute_value);
            break;
        }

        case SSS_CMD_COMMON_SLEEP_MODE_SET:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_COMMON_SLEEP_MODE_SET : sequence=%d", seq_num);
            uint8_t sleep_mode_status = data[4];
            if (sleep_mode_status != 1) {
                sleep_mode_status = 0; // 1以外は全て0 (=off)にする
            }
            SSS_CLR_BUF(m_payload_data);
            m_payload_data[0] = sleep_mode_status;
            uint16_t payload_len = 1;
            ret_val = record_write(CONFIG_FILE, CONFIG_SLEEP_MODE_KEY, m_payload_data, payload_len);
            if (ret_val == NRF_SUCCESS) {
                if (sleep_mode_status == 1) {
                    // Enter Sleep Mode
                    m_enter_sleep_mode_requested = true;
                    m_exit_sleep_mode_requested = false;
                    NRF_LOG_DEBUG(" -- Enter Sleep Mode...");
                } else {
                    // Exit Sleep Mode
                    m_enter_sleep_mode_requested = false;
                    m_exit_sleep_mode_requested = true;
                    NRF_LOG_DEBUG(" -- Exit Sleep Mode...");
                }
                m_wait_tick = 5 * 10; // 5 sec later
                ble_sss_send_response_ok(cmd_id, seq_num);
            } else {
                ble_sss_send_response_ng(cmd_id, seq_num, ret_val);
            }
            break;
        }

        case SSS_CMD_COMMON_SLEEP_MODE_GET:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_COMMON_SLEEP_MODE_GET : sequence=%d", seq_num);
            uint8_t sleep_mode_status = m_sleep_mode;
            ble_sss_send_response_ok_with_param(cmd_id, seq_num, sleep_mode_status);
            break;
        }

        case SSS_CMD_COMMON_VIB_STRENGTH_SET:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_COMMON_VIB_STRENGTH_SET : sequence=%d", seq_num);
            uint8_t strength_id = data[4];
            if (strength_id > 3) {
                strength_id = 3; // Max 3
            }
            SSS_CLR_BUF(m_payload_data);
            m_payload_data[0] = strength_id;
            uint16_t payload_len = 1;
            ret_val = record_write(CONFIG_FILE, CONFIG_VIB_STRENGTH_KEY, m_payload_data, payload_len);
            if (ret_val == NRF_SUCCESS) {
                vib_set_common_strength( strength_id );
                ble_sss_send_response_ok(cmd_id, seq_num);
            } else {
                ble_sss_send_response_ng(cmd_id, seq_num, ret_val);
            }
            break;
        }

        case SSS_CMD_COMMON_VIB_STRENGTH_GET:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_COMMON_VIB_STRENGTH_GET : sequence=%d", seq_num);
            uint32_t data_len;
            SSS_CLR_BUF(m_loaded_data);
            ret_val = record_read(CONFIG_FILE, CONFIG_VIB_STRENGTH_KEY, m_loaded_data, &data_len);
            if (ret_val == NRF_SUCCESS) {
                vib_set_common_strength(m_loaded_data[0]);
            }
            uint8_t strength_id = vib_get_common_strength();
            ble_sss_send_response_ok_with_param(cmd_id, seq_num, strength_id);
            break;
        }

        case SSS_CMD_COMMON_RECV_TARGET_SET:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_COMMON_RECV_TARGET_SET : sequence=%d", seq_num);
            ble_sss_send_response_ok(cmd_id, seq_num);
            // record_key: convert 2 bytes uint8_t to uint16_t
            uint16_t key = 0x5000; // Common Key Base
            key |= data[4];
            // Shift and make sub array
            SSS_CLR_BUF(m_payload_data);
            SSS_COPY_BUF(m_payload_data, data, 4, length);
            uint16_t payload_len = length - 5;
            ret_val = record_write(CONFIG_FILE, key, m_payload_data, payload_len);
            
            if (ret_val == NRF_SUCCESS) {

                switch( key ) {
                    case CONFIG_COMMON_TEL_KEY:
                        m_notif_all_tel = data[5];
                        break;
                    case CONFIG_COMMON_EMAIL_KEY:
                        m_notif_all_email = data[5];
                        break;
                    case CONFIG_COMMON_LINE_KEY:
                        m_notif_all_line = data[5];
                        break;
                    case CONFIG_COMMON_TW_KEY:
                        m_notif_all_twitter = data[5];
                        break;
                    case CONFIG_COMMON_FB_KEY:
                        m_notif_all_facebook = data[5];
                        break;
                    case CONFIG_COMMON_INSTA_KEY:
                        m_notif_all_instagram = data[5];
                        break;
                    case CONFIG_COMMON_OTHER_KEY:
                        m_notif_all_others = data[5];
                        break;
                }

                ble_sss_send_response_ok(cmd_id, seq_num);
            } else {
                ble_sss_send_response_ng(cmd_id, seq_num, ret_val);
            }
            break;
        }

        case SSS_CMD_COMMON_RECV_TARGET_GET:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_COMMON_RECV_TARGET_GET : sequence=%d", seq_num);
            // record_key: convert 2 bytes uint8_t to uint16_t
            uint16_t key = 0x5000; // Common Key Base
            key |= data[4]; // pattern_id

            uint32_t data_len = 0;
            SSS_CLR_BUF(m_loaded_data);

            ret_val = record_read(CONFIG_FILE, key, m_loaded_data, &data_len);
            if (ret_val == NRF_SUCCESS) {
                ble_sss_send_data(cmd_id, seq_num, m_loaded_data, data_len);
            } else if (ret_val == FDS_ERR_NOT_FOUND) {
                ble_sss_send_data(cmd_id, seq_num, m_loaded_data, data_len);
            } else {
                ble_sss_send_response_ng(cmd_id, seq_num, ret_val);
            }

            break;
        }
		//2019.09.20 Add Led & Vib setting option Start
		case SSS_CMD_COMMON_NOTIFY_MODE_SET:
		{
		   NRF_LOG_INFO("[CMD] SSS_CMD_COMMON_NOTIFY_MODE_SET : sequence=%d", seq_num);
           uint8_t notify_id = data[4];
            if (notify_id != 1) { //1:VIB 0:LED
                notify_id = 0; // 1以外は全て0にする
            }
            SSS_CLR_BUF(m_payload_data);
            m_payload_data[0] = notify_id;
            uint16_t payload_len = 1;
            ret_val = record_write(CONFIG_FILE, CONFIG_NOTIFY_MODE_KEY, m_payload_data, payload_len);
            if (ret_val == NRF_SUCCESS) {
                ble_sss_set_notif_Led_vib(notify_id);
                ble_sss_send_response_ok(cmd_id, seq_num);
            } else {
                ble_sss_send_response_ng(cmd_id, seq_num, ret_val);
            }
			break;
		}

		case SSS_CMD_COMMON_NOTIFY_MODE_GET:
		{
		   NRF_LOG_INFO("[CMD] SSS_CMD_COMMON_NOTIFY_MODE_GET : sequence=%d", seq_num);
           uint32_t data_len;
            SSS_CLR_BUF(m_loaded_data);
            ret_val = record_read(CONFIG_FILE, CONFIG_NOTIFY_MODE_KEY, m_loaded_data, &data_len);
            if (ret_val == NRF_SUCCESS) {
                ble_sss_set_notif_Led_vib(m_loaded_data[0]);
            }
            uint8_t led_vib_value = ble_sss_get_notif_Led_vib();
            ble_sss_send_response_ok_with_param(cmd_id, seq_num, led_vib_value);
			break;
		}
		//2019.09.20 Add Led & Vib setting option End
        case SSS_CMD_PATTERN_RECV_TARGET_SET:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_PATTERN_RECV_TARGET_SET : sequence=%d", seq_num);
            // record_key: convert 2 bytes uint8_t to uint16_t
            uint16_t key = 0x7000; // Pattern Key Base
            if (data[4] == 7) {
                // 7の指定のときのみFDSでエラーになるのでskip
                key |= 8;
            } else {
                key |= data[4];
            }
            // Shift and make sub array
            SSS_CLR_BUF(m_payload_data);
            SSS_COPY_BUF(m_payload_data, data, 5, length);
            uint16_t payload_len = length - 6;
            // 設定をOnMemoryキャッシュ
            ble_sss_set_ptn_setting_cache(key, m_payload_data, payload_len);
            // FDSに書込み
            ret_val = record_write(CONFIG_FILE, key, m_payload_data, payload_len);
            if (ret_val == NRF_SUCCESS) {
                ble_sss_send_response_ok(cmd_id, seq_num);
            } else {
                ble_sss_send_response_ng(cmd_id, seq_num, ret_val);
            }
            break;
        }

        case SSS_CMD_PATTERN_RECV_TARGET_GET:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_PATTERN_RECV_TARGET_GET : sequence=%d", seq_num);
            // record_key: convert 2 bytes uint8_t to uint16_t
            uint16_t key = 0x7000; // Pattern Key Base
            if (data[4] == 7) {
                // 7の指定のときのみFDSでエラーになるのでskip
                key |= 8;
            } else {
                key |= data[4]; // pattern_id
            }

            uint32_t data_len = 0;
            SSS_CLR_BUF(m_loaded_data);

            ret_val = record_read(CONFIG_FILE, key, m_loaded_data, &data_len);
            if (ret_val == NRF_SUCCESS) {
                ble_sss_send_data(cmd_id, seq_num, m_loaded_data, data_len);
            } else if (ret_val == FDS_ERR_NOT_FOUND) {
                ble_sss_send_data(cmd_id, seq_num, m_loaded_data, data_len);
            } else {
                ble_sss_send_response_ng(cmd_id, seq_num, ret_val);
            }
            break;
        }

        case SSS_CMD_PATTERN_RECV_TARGET_CLR:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_PATTERN_RECV_TARGET_CLR : sequence=%d", seq_num);
            // record_key: convert 2 bytes uint8_t to uint16_t
            uint16_t key = 0x7000;
            key |= data[4]; // pattern_id

            // OnMemoryキャッシュをクリア
            ble_sss_set_ptn_setting_cache(key, NULL, 0);
            // FDSから削除
            ret_val = record_delete(CONFIG_FILE, key);
            if (ret_val == NRF_SUCCESS || ret_val == FDS_ERR_NOT_FOUND) {
                ble_sss_send_response_ok(cmd_id, seq_num);
            } else {
                ble_sss_send_response_ng(cmd_id, seq_num, ret_val);
            }
            break;
        }

        case SSS_CMD_PAIRING_CLR:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_PAIRING_CLR : sequence=%d", seq_num);
            ret_val = ble_sss_delete_bonds();
            if (ret_val == NRF_SUCCESS) {
                ble_sss_send_response_ok(cmd_id, seq_num);
            } else {
                ble_sss_send_response_ng(cmd_id, seq_num, ret_val);
            }
            break;
        }

        case SSS_CMD_SHUTDOWN:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_SHUTDOWN : sequence=%d", seq_num);
            ret_val = ble_sss_shutdown();
            if (ret_val == NRF_SUCCESS) {
                ble_sss_send_response_ok(cmd_id, seq_num);
            } else {
                ble_sss_send_response_ng(cmd_id, seq_num, ret_val);
            }
            break;
        }

        case SSS_CMD_REBOOT:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_REBOOT : sequence=%d", seq_num);
            ble_sss_send_response_ok(cmd_id, seq_num);
            break;
        }

        case SSS_CMD_DISCONNECT:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_DISCONNECT : sequence=%d", seq_num);
            ble_sss_send_response_ok(cmd_id, seq_num);
            break;
        }

        case SSS_CMD_ENTER_DFU_MODE:
        {
            NRF_LOG_INFO("[CMD] SSS_CMD_ENTER_DFU_MODE : sequence=%d", seq_num);
            ble_sss_send_response_ok(cmd_id, seq_num);
            bootloader_start();
            break;
        }

        default:
        {
            NRF_LOG_WARNING("Command not supported : 0x%x, sequence=%d", cmd_id, seq_num);
            ble_sss_send_response_ng(cmd_id, seq_num, NRF_ERROR_NOT_SUPPORTED);
            ret_val = NRF_ERROR_NOT_SUPPORTED;
        }
    }

    return ret_val;
}

void ble_sss_send_data(const uint8_t cmd, const uint8_t seq_num, void * p_data, uint32_t len)
{
    uint32_t err_code = NRF_SUCCESS;
    uint16_t data_len = (uint16_t)len;
    err_code = ble_nus_string_send(&m_nus, p_data, &data_len);
    if (err_code == NRF_SUCCESS) {
        NRF_LOG_INFO("[CMD] data sent : sequence=%d, cmd=%d", seq_num, cmd);
    } else if (err_code == NRF_ERROR_INVALID_STATE) {
        NRF_LOG_WARNING("Smartphone does not register this Notify characteristics");
    } else {
        APP_ERROR_CHECK(err_code);
    }
}

void ble_sss_send_response_ok(const uint8_t cmd, const uint8_t seq_num)
{
    uint32_t err_code = NRF_SUCCESS;
    uint16_t length = 3;
    uint8_t d[] = {cmd, seq_num, SSS_CMD_STATUS_OK};
    err_code = ble_nus_string_send(&m_nus, d, &length);
    if (err_code == NRF_SUCCESS) {
        NRF_LOG_INFO("[CMD] OK response notified : sequence=%d, cmd=%d", seq_num, cmd);
    } else if (err_code == NRF_ERROR_INVALID_STATE) {
        NRF_LOG_WARNING("Smartphone does not register this Notify characteristics");
    } else {
        APP_ERROR_CHECK(err_code);
    }
}

void ble_sss_send_response_ok_with_param(const uint8_t cmd, const uint8_t seq_num, const uint8_t param)
{
    uint32_t err_code = NRF_SUCCESS;
    uint16_t length = 4;
    uint8_t d[] = {cmd, seq_num, SSS_CMD_STATUS_OK, param};
    err_code = ble_nus_string_send(&m_nus, d, &length);
    if (err_code == NRF_SUCCESS) {
        NRF_LOG_INFO("[CMD] OK response notified : sequence=%d, cmd=%d", seq_num, cmd);
    } else if (err_code == NRF_ERROR_INVALID_STATE) {
        NRF_LOG_WARNING("Smartphone does not register this Notify characteristics");
    } else {
        APP_ERROR_CHECK(err_code);
    }
}

void ble_sss_send_response_ng(const uint8_t cmd, const uint8_t seq_num, const uint8_t cmd_err_code)
{
    uint32_t err_code = NRF_SUCCESS;
    uint16_t length = 4;
    uint8_t d[] = {cmd, seq_num, SSS_CMD_STATUS_NG, cmd_err_code};
    err_code = ble_nus_string_send(&m_nus, d, &length);
    if (err_code == NRF_SUCCESS) {
        NRF_LOG_INFO("[CMD] NG response notified : sequence=%d, cmd=%d", seq_num, cmd);
    } else if (err_code == NRF_ERROR_INVALID_STATE) {
        NRF_LOG_WARNING("Smartphone does not register this Notify characteristics");
    } else {
        APP_ERROR_CHECK(err_code);
    }
}

ret_code_t record_write(uint32_t fid, uint32_t key, void const * p_data, uint32_t len)
{
    ret_code_t ret;
    fds_find_token_t tok   = {0};
    fds_record_desc_t desc = {0};
    fds_record_t const rec =
    {
        .file_id           = fid,
        .key               = key,
        .data.p_data       = p_data,
        .data.length_words = (len + 3) / sizeof(uint32_t)
    };

    NRF_LOG_DEBUG("writing record to flash...");
    NRF_LOG_INFO("file: 0x%x, key: 0x%x, \"%s\", len: %u bytes", fid, key, p_data, len);
    fds_debug_print(p_data, len);

    if (fds_record_find(fid, key, &desc, &tok) == FDS_SUCCESS)
    {
        // 既にデータが書き込まれている場合、アップデートする
        NRF_LOG_DEBUG(" -- updating existing record...");
        ret = fds_record_update(&desc, &rec);
    }
    else 
    {
        // 新規の場合
        NRF_LOG_DEBUG(" -- writing new record...");
        ret = fds_record_write(&desc, &rec);
    }

    if (ret != FDS_SUCCESS)
    {
        NRF_LOG_ERROR("error: fds_record_write() returned %s.", fds_err_str[ret]);
        fds_debug_error(ret);
    }
    return ret;
}

ret_code_t record_delete(uint32_t fid, uint32_t key)
{
    ret_code_t ret;
    fds_find_token_t tok   = {0};
    fds_record_desc_t desc = {0};

    NRF_LOG_DEBUG("deleting record...");
    NRF_LOG_DEBUG("file: 0x%x, key: 0x%x", fid, key);

    if (fds_record_find(fid, key, &desc, &tok) == FDS_SUCCESS)
    {
        ret = fds_record_delete(&desc);
        if (ret != FDS_SUCCESS)
        {
            NRF_LOG_ERROR("error: fds_record_delete() returned %s.", fds_err_str[ret]);
            fds_debug_error(ret);
            return ret;
        }

        NRF_LOG_DEBUG("record id: 0x%x", desc.record_id);
        return ret;
    }
    else
    {
        NRF_LOG_WARNING("error: record not found!");
        return FDS_ERR_NOT_FOUND;
    }
}

ret_code_t record_read(uint32_t fid, uint32_t key, uint8_t *p_data, uint32_t *p_data_len)
{
    ret_code_t ret;
    fds_flash_record_t flash_record;
    fds_record_desc_t record_desc = {0};
    fds_find_token_t ftok = {0};
    uint32_t *record_data;
    uint16_t length;

    // Loop until all records with the given key and file ID have been found.
    while (fds_record_find(fid, key, &record_desc, &ftok) == FDS_SUCCESS)
    {
        ret = fds_record_open(&record_desc, &flash_record);

        if (ret != FDS_SUCCESS) {
            // Handle error.
            NRF_LOG_ERROR(" + [FDS] read failed (fds_record_open)");
            fds_debug_error(ret);
            return ret;
        }

        record_data = (uint32_t *)flash_record.p_data;
        length = flash_record.p_header->length_words;

        // NRF_LOG_INFO(" + [FDS] read successful : file_id=%04x, record_key=%04x, record_id=%d, record_data_len=%d", fid, key, record_desc.record_id, length);
        // NRF_LOG_INFO(" + [FDS] p_data = ");
        for(uint16_t i=0; i < length; i++) {
            uint32_t w = record_data[i];
            // NRF_LOG_INFO(" + [FDS] - %08x ", record_data[i]);
            p_data[i*4+3] = (w >> 24) & 0xFF;
            p_data[i*4+2] = (w >> 16) & 0xFF;
            p_data[i*4+1] = (w >> 8) & 0xFF;
            p_data[i*4+0] = w & 0xFF;
        }

        // fds_debug_print(p_data, (length * 4));

        (*p_data_len) = length * 4;

        // Access the record through the flash_record structure.
        // Close the record when done.
        if (fds_record_close(&record_desc) != FDS_SUCCESS)
        {
            // Handle error.
            NRF_LOG_ERROR(" + [FDS] read failed (fds_record_close)");
        }

        return ret;
    }

    // NRF_LOG_WARNING(" + [FDS] not found read target : file_id=%04x, record_key=%04x", fid, key);
    return FDS_ERR_NOT_FOUND;
}

void fds_debug_print(const uint8_t *data, const uint16_t data_len)
{
    for(uint16_t i=0; i < data_len; i++) {
        NRF_LOG_RAW_INFO("%02x ", data[i]);
    }
    NRF_LOG_RAW_INFO("\n");
}

void fds_debug_error(const ret_code_t err_code)
{
    switch (err_code)
    {
        case FDS_ERR_OPERATION_TIMEOUT:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_OPERATION_TIMEOUT(%d), The operation timed out.", err_code);
            break;
        }

        case FDS_ERR_NOT_INITIALIZED:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_NOT_INITIALIZED(%d), The module has not been initialized.", err_code);
            break;
        }

        case FDS_ERR_UNALIGNED_ADDR:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_UNALIGNED_ADDR(%d), The input data is not aligned to a word boundary.", err_code);
            break;
        }

        case FDS_ERR_INVALID_ARG:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_INVALID_ARG(%d), The parameter contains invalid data.", err_code);
            break;
        }

        case FDS_ERR_NULL_ARG:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_NULL_ARG(%d), The parameter is NULL.", err_code);
            break;
        }

        case FDS_ERR_NO_OPEN_RECORDS:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_NO_OPEN_RECORDS(%d), The record is not open, so it cannot be closed.", err_code);
            break;
        }

        case FDS_ERR_NO_SPACE_IN_FLASH:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_NO_SPACE_IN_FLASH(%d), There is no space in flash memory.", err_code);
            break;
        }

        case FDS_ERR_NO_SPACE_IN_QUEUES:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_NO_SPACE_IN_QUEUES(%d), There is no space in the internal queues.", err_code);
            break;
        }

        case FDS_ERR_RECORD_TOO_LARGE:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_RECORD_TOO_LARGE(%d), The record exceeds the maximum allowed size.", err_code);
            break;
        }

        case FDS_ERR_NOT_FOUND:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_NOT_FOUND(%d), The record was not found.", err_code);
            break;
        }

        case FDS_ERR_NO_PAGES:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_NO_PAGES(%d), No flash pages are available.", err_code);
            break;
        }

        case FDS_ERR_USER_LIMIT_REACHED:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_USER_LIMIT_REACHED(%d), The maximum number of users has been reached.", err_code);
            break;
        }

        case FDS_ERR_CRC_CHECK_FAILED:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_CRC_CHECK_FAILED(%d), The CRC check failed.", err_code);
            break;
        }

        case FDS_ERR_BUSY:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_BUSY(%d), The underlying flash subsystem was busy.", err_code);
            break;
        }

        case FDS_ERR_INTERNAL:
        {
            NRF_LOG_ERROR(" + [ERR] FDS_ERR_INTERNAL(%d), An internal error occurred.", err_code);
            break;
        }
    };
}

uint8_t m_ptn1_setting[FDS_VIRTUAL_PAGE_SIZE] = {0,};
uint32_t m_ptn1_setting_len = 0;
uint8_t m_ptn2_setting[FDS_VIRTUAL_PAGE_SIZE] = {0,};
uint32_t m_ptn2_setting_len = 0;
uint8_t m_ptn3_setting[FDS_VIRTUAL_PAGE_SIZE] = {0,};
uint32_t m_ptn3_setting_len = 0;
uint8_t m_ptn4_setting[FDS_VIRTUAL_PAGE_SIZE] = {0,};
uint32_t m_ptn4_setting_len = 0;
uint8_t m_ptn5_setting[FDS_VIRTUAL_PAGE_SIZE] = {0,};
uint32_t m_ptn5_setting_len = 0;
uint8_t m_ptn6_setting[FDS_VIRTUAL_PAGE_SIZE] = {0,};
uint32_t m_ptn6_setting_len = 0;
uint8_t m_ptn7_setting[FDS_VIRTUAL_PAGE_SIZE] = {0,};
uint32_t m_ptn7_setting_len = 0;

void ble_sss_set_ptn_setting_cache(uint32_t key, uint8_t *p_data, uint32_t p_data_len)
{
    uint8_t *p_setting;
    switch(key) {
        case CONFIG_PTN_01_KEY:
            if (p_data == NULL) {
                memset(m_ptn1_setting, 0x0, FDS_VIRTUAL_PAGE_SIZE);
                return;
            }
            p_setting = m_ptn1_setting;
            m_ptn1_setting_len = p_data_len;
            break;

        case CONFIG_PTN_02_KEY:
            if (p_data == NULL) {
                memset(m_ptn2_setting, 0x0, FDS_VIRTUAL_PAGE_SIZE);
                return;
            }
            p_setting = m_ptn2_setting;
            m_ptn2_setting_len = p_data_len;
            break;

        case CONFIG_PTN_03_KEY:
            if (p_data == NULL) {
                memset(m_ptn3_setting, 0x0, FDS_VIRTUAL_PAGE_SIZE);
                return;
            }
            p_setting = m_ptn3_setting;
            m_ptn3_setting_len = p_data_len;
            break;

        case CONFIG_PTN_04_KEY:
            if (p_data == NULL) {
                memset(m_ptn4_setting, 0x0, FDS_VIRTUAL_PAGE_SIZE);
                return;
            }
            p_setting = m_ptn4_setting;
            m_ptn4_setting_len = p_data_len;
            break;

        case CONFIG_PTN_05_KEY:
            if (p_data == NULL) {
                memset(m_ptn5_setting, 0x0, FDS_VIRTUAL_PAGE_SIZE);
                return;
            }
            p_setting = m_ptn5_setting;
            m_ptn5_setting_len = p_data_len;
            break;

        case CONFIG_PTN_06_KEY:
            if (p_data == NULL) {
                memset(m_ptn6_setting, 0x0, FDS_VIRTUAL_PAGE_SIZE);
                return;
            }
            p_setting = m_ptn6_setting;
            m_ptn6_setting_len = p_data_len;
            break;

        case CONFIG_PTN_07_KEY:
            if (p_data == NULL) {
                memset(m_ptn7_setting, 0x0, FDS_VIRTUAL_PAGE_SIZE);
                return;
            }
            p_setting = m_ptn7_setting;
            m_ptn7_setting_len = p_data_len;
            break;
    }

    for(uint16_t i=0; i < p_data_len; i++ ) {
        p_setting[i] = p_data[i];
    }
}

ret_code_t ble_sss_pattern_record_read(uint32_t fid, uint32_t key, uint8_t *p_data, uint32_t *p_data_len)
{
    ret_code_t ret_val = NRF_SUCCESS;
    uint8_t zero_setting[FDS_VIRTUAL_PAGE_SIZE] = {0,};
    uint8_t *p_setting;
    switch(key) {
        case CONFIG_PTN_01_KEY:
            if (memcmp(m_ptn1_setting, zero_setting, FDS_VIRTUAL_PAGE_SIZE) == 0) return FDS_ERR_NOT_FOUND;
            p_setting = m_ptn1_setting;
            (*p_data_len) = m_ptn1_setting_len;
            break;

        case CONFIG_PTN_02_KEY:
            if (memcmp(m_ptn2_setting, zero_setting, FDS_VIRTUAL_PAGE_SIZE) == 0) return FDS_ERR_NOT_FOUND;
            p_setting = m_ptn2_setting;
            (*p_data_len) = m_ptn2_setting_len;
            break;

        case CONFIG_PTN_03_KEY:
            if (memcmp(m_ptn3_setting, zero_setting, FDS_VIRTUAL_PAGE_SIZE) == 0) return FDS_ERR_NOT_FOUND;
            p_setting = m_ptn3_setting;
            (*p_data_len) = m_ptn3_setting_len;
            break;

        case CONFIG_PTN_04_KEY:
            if (memcmp(m_ptn4_setting, zero_setting, FDS_VIRTUAL_PAGE_SIZE) == 0) return FDS_ERR_NOT_FOUND;
            p_setting = m_ptn4_setting;
            (*p_data_len) = m_ptn4_setting_len;
            break;

        case CONFIG_PTN_05_KEY:
            if (memcmp(m_ptn5_setting, zero_setting, FDS_VIRTUAL_PAGE_SIZE) == 0) return FDS_ERR_NOT_FOUND;
            p_setting = m_ptn5_setting;
            (*p_data_len) = m_ptn5_setting_len;
            break;

        case CONFIG_PTN_06_KEY:
            if (memcmp(m_ptn6_setting, zero_setting, FDS_VIRTUAL_PAGE_SIZE) == 0) return FDS_ERR_NOT_FOUND;
            p_setting = m_ptn6_setting;
            (*p_data_len) = m_ptn6_setting_len;
            break;

        case CONFIG_PTN_07_KEY:
            if (memcmp(m_ptn7_setting, zero_setting, FDS_VIRTUAL_PAGE_SIZE) == 0) return FDS_ERR_NOT_FOUND;
            p_setting = m_ptn7_setting;
            (*p_data_len) = m_ptn7_setting_len;
            break;
    }

    if (p_setting[0] == 0) {
        return FDS_ERR_NOT_FOUND;
    }

    for(uint16_t i=0; i < (*p_data_len); i++ ) {
        p_data[i] = p_setting[i];
    }

    NRF_LOG_DEBUG(" + Read cache pattern_id=%d, cat_id=%d", p_setting[0], p_setting[1]);
    return ret_val;
}


ret_code_t ble_sss_delete_bonds(void)
{
    ret_code_t err_code = NRF_SUCCESS;
    NRF_LOG_INFO(" + [BLE] Erase bonds!");
    err_code = pm_peers_delete();
    err_code = fds_gc();
    return err_code;
}

ret_code_t ble_sss_shutdown(void)
{
    ret_code_t err_code = NRF_SUCCESS;
    NRF_LOG_INFO(" + [BLE] Shutdown now ...");
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    return err_code;
}

bool ble_sss_get_all_tel(void) 
{
    return m_notif_all_tel;
}
bool ble_sss_get_all_email(void) 
{
    return m_notif_all_email;
}
bool ble_sss_get_all_line(void) 
{
    return m_notif_all_line;
}
bool ble_sss_get_all_twitter(void) 
{
    return m_notif_all_twitter;
}
bool ble_sss_get_all_facebook(void) 
{
    return m_notif_all_facebook;
}
bool ble_sss_get_all_instagram(void) 
{
    return m_notif_all_instagram;
}
bool ble_sss_get_all_others(void) 
{
    return m_notif_all_others;
}
//2019.09.20 Add Led & Vib setting option Start
bool ble_sss_get_notif_Led_vib()
{
   return m_notif_Led_vib;
}

bool ble_sss_set_notif_Led_vib(bool notif_Led_vib_flg)
{
    m_notif_Led_vib = notif_Led_vib_flg;
}

//2019.09.20 Add Led & Vib setting option End
ret_code_t ble_sss_load_settings(void)
{
    ret_code_t ret_val = NRF_SUCCESS;
    
    uint32_t data_len;

    NRF_LOG_INFO(" - ble_sss_load_settings");

    // 共通設定：振動ミュート
    SSS_CLR_BUF(m_loaded_data);
    ret_val = record_read(CONFIG_FILE, CONFIG_VIB_MUTE_KEY, m_loaded_data, &data_len);
    if (ret_val == NRF_SUCCESS) {
        vib_set_mute(m_loaded_data[0]);
    }
    // NRF_LOG_INFO(" + [Settings] vib_get_mute : %d", vib_get_mute());

    //2019.09.20 Add Led & Vib setting option Start
    // 共通設定：LED知らせるかバイブ知らせるか
	SSS_CLR_BUF(m_loaded_data);
    ret_val = record_read(CONFIG_FILE, CONFIG_NOTIFY_MODE_KEY, m_loaded_data, &data_len);
    if (ret_val == NRF_SUCCESS) {
        ble_sss_set_notif_Led_vib(m_loaded_data[0]);
    }
    NRF_LOG_INFO(" + [Settings] ble_sss_get_notif_Led_vib : %d", ble_sss_get_notif_Led_vib());
	//2019.09.20 Add Led & Vib setting option End

    // 共通設定：振動強度
    SSS_CLR_BUF(m_loaded_data);
    ret_val = record_read(CONFIG_FILE, CONFIG_VIB_STRENGTH_KEY, m_loaded_data, &data_len);
    if (ret_val == NRF_SUCCESS) {
        vib_set_common_strength(m_loaded_data[0]);
    }
    // NRF_LOG_INFO(" + [Settings] vib_get_common_strength : %d", vib_get_common_strength());

    
    // 共通設定：Sleep Mode
    SSS_CLR_BUF(m_loaded_data);
    ret_val = record_read(CONFIG_FILE, CONFIG_SLEEP_MODE_KEY, m_loaded_data, &data_len);
    if (ret_val == NRF_SUCCESS) {
        if (m_loaded_data[0] == 1) {
            m_sleep_mode = true;
            // FDSに保存されたSleepModeを戻す、電源再起動で通常に戻すため
            SSS_CLR_BUF(m_payload_data);
            m_payload_data[0] = 0;
            uint16_t payload_len = 1;
            ret_val = record_write(CONFIG_FILE, CONFIG_SLEEP_MODE_KEY, m_payload_data, payload_len);
        }
        NRF_LOG_INFO(" + [Settings] m_sleep_mode : %d", m_sleep_mode);
    }

    // 共通設定：電話、メール、LINE、Twitter、Facebook、Instagram、その他
    ret_val = record_read(CONFIG_FILE, CONFIG_COMMON_TEL_KEY, m_loaded_data, &data_len);
    if (ret_val == NRF_SUCCESS) {
        m_notif_all_tel = m_loaded_data[1];
    }
    NRF_LOG_INFO(" + [Settings] m_notif_all_tel : %d", m_notif_all_tel);
    
    ret_val = record_read(CONFIG_FILE, CONFIG_COMMON_EMAIL_KEY, m_loaded_data, &data_len);
    if (ret_val == NRF_SUCCESS) {
        m_notif_all_email = m_loaded_data[1];
    }
    // NRF_LOG_INFO(" + [Settings] m_notif_all_email : %d", m_notif_all_email);

    ret_val = record_read(CONFIG_FILE, CONFIG_COMMON_LINE_KEY, m_loaded_data, &data_len);
    if (ret_val == NRF_SUCCESS) {
        m_notif_all_line = m_loaded_data[1];
    }
    NRF_LOG_INFO(" + [Settings] m_notif_all_line : %d", m_notif_all_line);

    ret_val = record_read(CONFIG_FILE, CONFIG_COMMON_TW_KEY, m_loaded_data, &data_len);
    if (ret_val == NRF_SUCCESS) {
        m_notif_all_twitter = m_loaded_data[1];
        // NRF_LOG_INFO(" + %d %d", m_loaded_data[0], m_loaded_data[1]);
    }
    NRF_LOG_INFO(" + [Settings] m_notif_all_twitter : %d", m_notif_all_twitter);

    ret_val = record_read(CONFIG_FILE, CONFIG_COMMON_FB_KEY, m_loaded_data, &data_len);
    if (ret_val == NRF_SUCCESS) {
        m_notif_all_facebook = m_loaded_data[1];
    }
    // NRF_LOG_INFO(" + [Settings] m_notif_all_facebook : %d", m_notif_all_facebook);

    ret_val = record_read(CONFIG_FILE, CONFIG_COMMON_INSTA_KEY, m_loaded_data, &data_len);
    if (ret_val == NRF_SUCCESS) {
        m_notif_all_instagram = m_loaded_data[1];
    }
    // NRF_LOG_INFO(" + [Settings] m_notif_all_instagram : %d", m_notif_all_instagram);

    ret_val = record_read(CONFIG_FILE, CONFIG_COMMON_OTHER_KEY, m_loaded_data, &data_len);
    if (ret_val == NRF_SUCCESS) {
        m_notif_all_others = m_loaded_data[1];
    }
    NRF_LOG_INFO(" + [Settings] m_notif_all_others : %d", m_notif_all_others);

    return ret_val;
}