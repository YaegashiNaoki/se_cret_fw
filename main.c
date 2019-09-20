#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_err.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "peer_manager.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nrf_delay.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nrf_drv_saadc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "ble_db_discovery.h"
#include "nrf_ble_ancs_c.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "app_button.h"

#include "led.h"
#include "vibration.h"
#include "ble_sss.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define ATTR_DATA_SIZE                 BLE_ANCS_ATTR_DATA_MAX                       /**< Allocated size for attribute data. */

#define DISPLAY_MESSAGE_BUTTON_ID      1                                            /**< Button used to request notification attributes. */

#define DEVICE_NAME                    "Se_Cret-a"                                    /**< Name of the device. Will be included in the advertising data. */
#define DIS_MANUFACTURER_NAME          "Logic Pack"                             /**< Manufacturer. Will be passed to Device Information Service. */
#define DIS_MODEL_NUMBER               "SC-201907"
#define DIS_SERIAL_NUMBER              "LPSC03"
#define DIS_HW_REV                     "1.0.3"
#define DIS_FW_REV                     "1.1.1"


#define APP_DEFAULT_TX_POWER           4                                            /** TxPower : 4, 0, -4, -8, -12, -16, -20, -30 */
#define APP_BLE_OBSERVER_PRIO          3                                            /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG           1                                            /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */
#define APP_NOTIF_SKIP_INTERVAL_SEC    60                                          

#define APP_ADV_FAST_INTERVAL          40                                           /**< The advertising interval (in units of 0.625 ms). The default value corresponds to 25 ms. */
#define APP_ADV_FAST_TIMEOUT           30                                           /**< The advertising time-out in units of seconds. */
#define APP_ADV_SLOW_INTERVAL          3200                                          /**< Slow advertising interval (in units of 0.625 ms). The default value corresponds to 2 seconds. */
#define APP_ADV_SLOW_TIMEOUT           60                                           /**< The advertising time-out in units of seconds.*/

#define APP_ADV_SLEEP_FAST_INTERVAL    40                                           /**< advertising interval (in units of 0.625 ms). Sleep Mode : 2 seconds. */
#define APP_ADV_SLEEP_FAST_TIMEOUT     1                                            /**< The advertising time-out in units of seconds. */
#define APP_ADV_SLEEP_SLOW_INTERVAL    8000                                          /**< Slow advertising interval (in units of 0.625 ms). Sleep Mode : 2 seconds. */
#define APP_ADV_SLEEP_SLOW_TIMEOUT     60                                           /**< The advertising time-out in units of seconds. */

#define MIN_CONN_INTERVAL              MSEC_TO_UNITS(500, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL              MSEC_TO_UNITS(1000, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                  0                                            /**< Slave latency. */
#define CONN_SUP_TIMEOUT               MSEC_TO_UNITS(4000, UNIT_10_MS)              /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)                        /**< Time from initiating an event (connect or start of notification) to the first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(30000)                       /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT   3                                            /**< Number of attempts before giving up the connection parameter negotiation. */

#define MESSAGE_BUFFER_SIZE            18                                           /**< Size of buffer holding optional messages in notifications. */

#define SECURITY_REQUEST_DELAY         APP_TIMER_TICKS(1500)                        /**< Delay after connection until security request is sent, if necessary (ticks). */
#define THREAD_PROCESS_TICK_MS         APP_TIMER_TICKS(100)                          /**< Delay after connection until security request is sent, if necessary (ticks). */

#define SEC_PARAM_BOND                 1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                 0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                 0                                            /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS             0                                            /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES      BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                  0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE         7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE         16                                           /**< Maximum encryption key size. */

#define DEAD_BEEF                      0xDEADBEEF                                   /**< Value used as error code on stack dump. Can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE      APP_TIMER_SCHED_EVENT_DATA_SIZE              /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE               34                                           /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE               24                                           /**< Maximum number of events in the scheduler queue. */
#endif

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                         /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

APP_TIMER_DEF(m_sec_req_timer_id);                                                  /**< Security request timer. The timer lets us start pairing request if one does not arrive from the Central. */
APP_TIMER_DEF(m_thread_timer_id);                                                  /**< Thread process timer. */
BLE_ANCS_C_DEF(m_ancs_c);                                                           /**< Apple Notification Service Client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                                    /**< DB Discovery module instance. */

static ble_adv_evt_t m_cur_adv_mode;                                            /** BLE_ADV_EVT_FAST, BLE_ADV_EVT_SLOW, BLE_ADV_EVT_FAST_WHITELIST */

static pm_peer_id_t m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];            /**< List of peers currently in the whitelist. */
static uint32_t     m_whitelist_peer_cnt;                                           /**< Number of peers currently in the whitelist. */
static pm_peer_id_t m_peer_id;                                                      /**< Device reference handle to the current bonded central. */
static uint16_t     m_cur_conn_handle = BLE_CONN_HANDLE_INVALID;                    /**< Handle of the current connection. */

static ble_ancs_c_evt_notif_t m_notification_latest;                                /**< Local copy to keep track of the newest arriving notifications. */
static ble_ancs_c_attr_t      m_notif_attr_latest;                                  /**< Local copy of the newest notification attribute. */
static ble_ancs_c_attr_t      m_notif_attr_app_id_latest;                           /**< Local copy of the newest app attribute. */

static uint8_t m_attr_appid[ATTR_DATA_SIZE];                                        /**< Buffer to store attribute data. */
static uint8_t m_attr_title[ATTR_DATA_SIZE];                                        /**< Buffer to store attribute data. */
static uint8_t m_attr_subtitle[ATTR_DATA_SIZE];                                     /**< Buffer to store attribute data. */
static uint8_t m_attr_message[ATTR_DATA_SIZE];                                      /**< Buffer to store attribute data. */
static uint8_t m_attr_message_size[ATTR_DATA_SIZE];                                 /**< Buffer to store attribute data. */
static uint8_t m_attr_date[ATTR_DATA_SIZE];                                         /**< Buffer to store attribute data. */
static uint8_t m_attr_posaction[ATTR_DATA_SIZE];                                    /**< Buffer to store attribute data. */
static uint8_t m_attr_negaction[ATTR_DATA_SIZE];                                    /**< Buffer to store attribute data. */
static uint8_t m_attr_disp_name[ATTR_DATA_SIZE];                                    /**< Buffer to store attribute data. */

/**@brief String literals for the iOS notification categories. used then printing to UART. */
static char const * lit_catid[BLE_ANCS_NB_OF_CATEGORY_ID] =
{
    "Other",
    "Incoming Call",
    "Missed Call",
    "Voice Mail",
    "Social",
    "Schedule",
    "Email",
    "News",
    "Health And Fitness",
    "Business And Finance",
    "Location",
    "Entertainment"
};

/**@brief String literals for the iOS notification event types. Used then printing to UART. */
static char const * lit_eventid[BLE_ANCS_NB_OF_EVT_ID] =
{
    "Added",
    "Modified",
    "Removed"
};

/**@brief String literals for the iOS notification attribute types. Used when printing to UART. */
static char const * lit_attrid[BLE_ANCS_NB_OF_NOTIF_ATTR] =
{
    "App Identifier",
    "Title",
    "Subtitle",
    "Message",
    "Message Size",
    "Date",
    "Positive Action Label",
    "Negative Action Label"
};

/**@brief String literals for the iOS notification attribute types. Used When printing to UART. */
static char const * lit_appid[BLE_ANCS_NB_OF_APP_ATTR] =
{
    "Display Name"
};

/* Array to map FDS return values to strings. */
char const * fds_err_str[] =
{
    "FDS_SUCCESS",
    "FDS_ERR_OPERATION_TIMEOUT",
    "FDS_ERR_NOT_INITIALIZED",
    "FDS_ERR_UNALIGNED_ADDR",
    "FDS_ERR_INVALID_ARG",
    "FDS_ERR_NULL_ARG",
    "FDS_ERR_NO_OPEN_RECORDS",
    "FDS_ERR_NO_SPACE_IN_FLASH",
    "FDS_ERR_NO_SPACE_IN_QUEUES",
    "FDS_ERR_RECORD_TOO_LARGE",
    "FDS_ERR_NOT_FOUND",
    "FDS_ERR_NO_PAGES",
    "FDS_ERR_USER_LIMIT_REACHED",
    "FDS_ERR_CRC_CHECK_FAILED",
    "FDS_ERR_BUSY",
    "FDS_ERR_INTERNAL",
};

/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

#define NOTIF_CAT_UNDEF 0x00
#define NOTIF_CAT_TEL   0x01
#define NOTIF_CAT_EMAIL 0x02
#define NOTIF_CAT_LINE  0x03
#define NOTIF_CAT_TW    0x04
#define NOTIF_CAT_FB    0x05
#define NOTIF_CAT_INSTA 0x06
#define NOTIF_CAT_OTHER 0x99

static bool     m_notif_arrived;
static uint8_t  m_notif_category_id = NOTIF_CAT_UNDEF;
static uint8_t* m_notif_src_name;
static uint16_t m_notif_src_name_len;
static bool     m_apple_setup_requested;
uint16_t  m_wait_tick;

static void delete_bonds(void);

#define RTC_FREQUENCY 32                          //Determines the RTC frequency and prescaler
#define RTC_CC_VALUE 8                            //Determines the RTC interrupt frequency and thereby the SAADC sampling frequency
#define SAADC_CALIBRATION_INTERVAL 5              //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER 1                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_4X  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE 1                        //Set to 1 to enable BURST mode, otherwise set to 0.

void saadc_init(void);

static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t                m_adc_evt_counter = 0;
static bool                    m_battery_monitor_requested;
static bool                    m_saadc_calibrate = false;      

#define BATTERY_MONITOR_LOW_FROM 1000
#define BATTERY_MONITOR_LOW_UPTO 2600 // 2.6mV

/**
 * SeCret Sleep Mode
 *  - 超SlowAdvで実装（10秒windowタイム）
 *  - スリープモード中は通知は受付ない
 *  - アプリのTop画面でスリープ解除, スリープするボタンを設定画面に追加する
 */
bool     m_sleep_mode                 = false;
bool     m_enter_sleep_mode_requested = false;
bool     m_exit_sleep_mode_requested  = false;

/**@brief Callback function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product.
 *          You must analyze how your product should react to asserts.
 * @warning On assert from the SoftDevice, the system can recover only on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
        if((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0)                                  //Evaluate if offset calibration should be performed. Configure the SAADC_CALIBRATION_INTERVAL constant to change the calibration frequency
        {
            nrf_drv_saadc_abort();                                                                      // Abort all ongoing conversions. Calibration cannot be run if SAADC is busy
            m_saadc_calibrate = true;                                                                   // Set flag to trigger calibration in main context when SAADC is stopped
        }
        
        // NRF_LOG_INFO("ADC event number: %d",(int)m_adc_evt_counter);                                //Print the event number on UART

        for (int i = 0; i < p_event->data.done.size; i++)
        {
            // NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);                                     //Print the SAADC result on UART
            if (BATTERY_MONITOR_LOW_FROM < p_event->data.done.p_buffer[i] && p_event->data.done.p_buffer[i] <= BATTERY_MONITOR_LOW_UPTO) {
                // 2.6V Low Batteryの場合、赤色LED 10回点滅
                led_blink_red(10);
            }
        }

        if(m_saadc_calibrate == false)
        {
            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 
            APP_ERROR_CHECK(err_code);
        }
        
        m_adc_evt_counter++;
  
    }
    else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE)
    {
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);             //Need to setup both buffers, as they were both removed with the call to nrf_drv_saadc_abort before calibration.
        APP_ERROR_CHECK(err_code);
        // NRF_LOG_INFO("SAADC calibration complete!");                                              //Print on UART
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;

    //Configure SAADC
    saadc_config.low_power_mode = true;                                                   //Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
	
    //Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);                         //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
    APP_ERROR_CHECK(err_code);
		
    //Configure SAADC channel
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config.pin_p = NRF_SAADC_INPUT_AIN0;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin

    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
		
    if(SAADC_BURST_MODE)
    {
        NRF_SAADC->CH[0].CONFIG |= 0x01000000;                                            //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.		
    }

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
    APP_ERROR_CHECK(err_code);
}


/* Flag to check fds initialization. */
static bool volatile m_fds_initialized;

static void fds_evt_handler(fds_evt_t const * p_evt)
{
    // NRF_LOG_DEBUG("Event: %s received (%s)", fds_evt_str[p_evt->id], fds_err_str[p_evt->result]);

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == FDS_SUCCESS)
            {
                m_fds_initialized = true;
            }
            else
            {
                // fds_debug_error(p_evt->result);
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                // NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                // NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                // NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                // NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                // NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                // NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
        } break;

        default:
            break;
    }
}

/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}


/**@brief Function for starting advertising. */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t ret;

        memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
        m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

        peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

        ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
        APP_ERROR_CHECK(ret);

        // Setup the device identies list.
        // Some SoftDevices do not support this feature.
        ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
        if (ret != NRF_ERROR_NOT_SUPPORTED)
        {
            APP_ERROR_CHECK(ret);
        }

        ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(ret);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t ret;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_DEBUG("Connected to previously bonded device");
            m_peer_id = p_evt->peer_id;
        } break; // PM_EVT_BONDED_PEER_CONNECTED

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);

            m_peer_id = p_evt->peer_id;

            // Discover peer's services.
            ret  = ble_db_discovery_start(&m_db_disc, p_evt->conn_handle);
            APP_ERROR_CHECK(ret);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            ret = fds_gc();
            if (ret == FDS_ERR_BUSY || ret == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(ret);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            // Advertising再開前にDisconnectする
            // ret = sd_ble_gap_disconnect(m_cur_conn_handle,
            //                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        {
            // Note: You should check on what kind of white list policy your application should use.
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                NRF_LOG_DEBUG("New Bond, add the peer to the whitelist if possible");
                NRF_LOG_DEBUG("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                               m_whitelist_peer_cnt + 1,
                               BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

                if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                {
                    // Bonded to a new peer, add it to the whitelist.
                    m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;

                    // The whitelist has been modified, update it in the Peer Manager.
                    ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    if (ret != NRF_ERROR_NOT_SUPPORTED)
                    {
                        APP_ERROR_CHECK(ret);
                    }

                    ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    APP_ERROR_CHECK(ret);
                }
            }
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for handling the security request timer time-out.
 *
 * @details This function is called each time the security request timer expires.
 *
 * @param[in] p_context  Pointer used for passing context information from the
 *                       app_start_timer() call to the time-out handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    ret_code_t           ret;
    pm_conn_sec_status_t status;

    if (m_cur_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ret = pm_conn_sec_status_get(m_cur_conn_handle, &status);
        APP_ERROR_CHECK(ret);

        // If the link is still not secured by the peer, initiate security procedure.
        if (!status.encrypted)
        {
            ret = pm_conn_secure(m_cur_conn_handle, false);
            if (ret != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(ret);
            }
        }
    }
}

static void apple_notification_setup_impl(void);

static bool test_pattern_setting(const uint32_t key, const uint8_t ptn_id, uint8_t notif_cat_id, uint8_t* p_notif_src_name, uint16_t notif_src_name_len)
{
    ret_code_t ret_val = NRF_SUCCESS;
    uint32_t data_len;
    uint32_t comp_len;
    uint8_t p_loaded_data[FDS_VIRTUAL_PAGE_SIZE * 4] = {0,};

    ret_val = ble_sss_pattern_record_read(CONFIG_FILE, key, p_loaded_data, &data_len);
    // comp_len = (data_len > notif_src_name_len) ? notif_src_name_len : data_len;
    comp_len = (data_len - 2);
    if (ret_val == NRF_SUCCESS) {
        uint8_t cat_id = p_loaded_data[0]; // 電話01, Email02, LINE03
        char *name = p_loaded_data + 1; // 設定済み検索名称
        fds_debug_print(p_notif_src_name, notif_src_name_len);
        fds_debug_print(name, comp_len);
        NRF_LOG_INFO(" + [Hit(Cache)] cat=%d src=%02x %02x, cmp=%02x %02x", cat_id, p_notif_src_name[0], p_notif_src_name[1], name[0], name[1]);
        if (notif_cat_id == cat_id) {
            if (memcmp(p_notif_src_name, name, comp_len) == 0) {
                NRF_LOG_INFO(" + [Hit(Cache) OK] pattern_id=%d", ptn_id);
                vib_play_pattern(ptn_id);
                return true;
            } else {
                NRF_LOG_INFO(" --> No match");
            }
        }
    }

    ret_val = record_read(CONFIG_FILE, key, p_loaded_data, &data_len);
    comp_len = (data_len - 3);
    if (ret_val == NRF_SUCCESS) {
        ble_sss_set_ptn_setting_cache(key, p_loaded_data, data_len); // キャッシュに残す
        uint8_t cat_id = p_loaded_data[0]; // 電話01, Email02, LINE03
        char *name = p_loaded_data + 1; // 設定済み検索名称
        fds_debug_print(p_notif_src_name, notif_src_name_len);
        fds_debug_print(name, comp_len);
        NRF_LOG_INFO(" + [Hit] cat=%d src=%02x %02x, cmp=%02x %02x", cat_id, p_notif_src_name[0], p_notif_src_name[1], name[0], name[1]);
        if (notif_cat_id == cat_id) {
            if (memcmp(p_notif_src_name, name, comp_len) == 0) {
                NRF_LOG_INFO(" + [Hit OK] pattern_id=%d", ptn_id);
                vib_play_pattern(ptn_id);
                return true;
            } else {
                NRF_LOG_INFO(" --> No match");
            }
        }
    }

    return false;
}

static void notif_process_handler(uint8_t notif_cat_id, uint8_t* p_notif_src_name, uint16_t notif_src_name_len)
{
    bool hit = false;

    NRF_LOG_INFO("Notification is processed : category=%d, src_name=%08x, len=%d", notif_cat_id, p_notif_src_name, notif_src_name_len);
    // fds_debug_print(p_notif_src_name, notif_src_name_len);
    
    // Pattern #1
    hit = test_pattern_setting(CONFIG_PTN_01_KEY, 1, notif_cat_id, p_notif_src_name, notif_src_name_len);
    if (hit) return;
    // Pattern #2
    hit = test_pattern_setting(CONFIG_PTN_02_KEY, 2, notif_cat_id, p_notif_src_name, notif_src_name_len);
    if (hit) return;
    // Pattern #3
    hit = test_pattern_setting(CONFIG_PTN_03_KEY, 3, notif_cat_id, p_notif_src_name, notif_src_name_len);
    if (hit) return;
    // Pattern #4
    hit = test_pattern_setting(CONFIG_PTN_04_KEY, 4, notif_cat_id, p_notif_src_name, notif_src_name_len);
    if (hit) return;
    // Pattern #5
    hit = test_pattern_setting(CONFIG_PTN_05_KEY, 5, notif_cat_id, p_notif_src_name, notif_src_name_len);
    if (hit) return;
    // Pattern #6
    hit = test_pattern_setting(CONFIG_PTN_06_KEY, 6, notif_cat_id, p_notif_src_name, notif_src_name_len);
    if (hit) return;
    // Pattern #7
    hit = test_pattern_setting(CONFIG_PTN_07_KEY, 7, notif_cat_id, p_notif_src_name, notif_src_name_len);
    if (hit) return;

    // Non Specific Notif Handler
    switch(notif_cat_id) {
        case NOTIF_CAT_TEL:
            if (ble_sss_get_all_tel()) {
		      if(ble_sss_get_notif_Led_vib()){
				vib_play_common();
			  }else{ //LED
				led_notif(30);
			  }
			}
            NRF_LOG_INFO(" + [Hit] Non Specific : NOTIF_CAT_TEL, all=%d", ble_sss_get_all_tel());
            break;

        case NOTIF_CAT_EMAIL:
            if (ble_sss_get_all_email()){
		      if(ble_sss_get_notif_Led_vib()){
				vib_play_common();
			  }else{ //LED
				led_notif(30);
			  }
			 }
            NRF_LOG_INFO(" + [Hit] Non Specific : NOTIF_CAT_EMAIL, all=%d", ble_sss_get_all_email());
            break;

        case NOTIF_CAT_LINE:
            if (ble_sss_get_all_line()) {
		      if(ble_sss_get_notif_Led_vib()){
				vib_play_common();
			  }else{ //LED
				led_notif(30);
			  }
			 }
            NRF_LOG_INFO(" + [Hit] Non Specific : NOTIF_CAT_LINE, all=%d", ble_sss_get_all_line());
            break;

        case NOTIF_CAT_TW:
            if (ble_sss_get_all_twitter()){
		      if(ble_sss_get_notif_Led_vib()){
				vib_play_common();
			  }else{ //LED
				led_notif(30);
			  }
			}
            NRF_LOG_INFO(" + [Hit] Non Specific : NOTIF_CAT_TW, all=%d", ble_sss_get_all_twitter());
            break;

        case NOTIF_CAT_FB:
            if (ble_sss_get_all_facebook()) {
		      if(ble_sss_get_notif_Led_vib()){
				vib_play_common();
			  }else{ //LED
				led_notif(30);
			  }
			 }
            NRF_LOG_INFO(" + [Hit] Non Specific : NOTIF_CAT_FB, all=%d", ble_sss_get_all_facebook());
            break;

        case NOTIF_CAT_INSTA:
            if (ble_sss_get_all_instagram()) {
		      if(ble_sss_get_notif_Led_vib()){
				vib_play_common();
			  }else{ //LED
				led_notif(30);
			  }
			}
            NRF_LOG_INFO(" + [Hit] Non Specific : NOTIF_CAT_INSTA, all=%d", ble_sss_get_all_instagram());
            break;

        case NOTIF_CAT_OTHER:
            if (ble_sss_get_all_others()) {
		      if(ble_sss_get_notif_Led_vib()){
				vib_play_common();
			  }else{ //LED
				led_notif(30);
			  }
			 }
            NRF_LOG_INFO(" + [Hit] Non Specific : NOTIF_CAT_OTHER, all=%d", ble_sss_get_all_others());
            break;
    }
}


/**@brief Function for handling threads.
 * @param[in] p_context  Pointer used for passing context information from the
 *                       app_start_timer() call to the time-out handler.
 */
static void thread_process_handler(void * p_context)
{
    if (m_apple_setup_requested) {
        if (m_wait_tick > 0) m_wait_tick--;
        if (m_wait_tick == 0) {
            m_apple_setup_requested = false;
            apple_notification_setup_impl();
        }
    }
    if (m_battery_monitor_requested) {
        if (m_wait_tick > 0) m_wait_tick--;
        if (m_wait_tick == 0) {
            nrf_drv_saadc_sample();
            NRF_LOG_DEBUG(" - monitoring battery : nrf_drv_saadc_sample");
            m_wait_tick = 2;
            if (m_adc_evt_counter >= 1) {
                m_battery_monitor_requested = false;
            }
        }
    }
    if (m_enter_sleep_mode_requested) {
        if (m_wait_tick > 0) m_wait_tick--;
        if (m_wait_tick == 0) {
            NRF_LOG_DEBUG(" - entering se_cret sleep mode ...");
            m_enter_sleep_mode_requested = false;
            m_sleep_mode = true;
            // Restart to apply new Advertising settings
            sd_nvic_SystemReset();
        }
    }
    if (m_exit_sleep_mode_requested) {
        if (m_wait_tick > 0) m_wait_tick--;
        if (m_wait_tick == 0) {
            NRF_LOG_DEBUG(" - exiting se_cret sleep mode ...");
            m_exit_sleep_mode_requested = false;
            m_sleep_mode = false;
            // SleepMode解除時にバッテリーチェック
            // Restart to apply new Advertising settings
            sd_nvic_SystemReset();
        }
    }

    led_process_next_tick();
    vib_process_next_tick();

    if (m_wait_tick > 0) {
        m_wait_tick--;
        // NRF_LOG_INFO(" - m_wait_tick : %d", m_wait_tick);
        m_notif_category_id = NOTIF_CAT_UNDEF;
        return; // wait_tickが0になるまでは、新規notifを受け付けない
    }

    // 新規notifがあれば処理する
    if (m_notif_arrived) {
        m_notif_arrived = false;
        if (m_sleep_mode) {
            // Sleep Modeの場合はスキップ
            return;
        }
        notif_process_handler(m_notif_category_id, m_notif_src_name, m_notif_src_name_len);
        m_wait_tick = APP_NOTIF_SKIP_INTERVAL_SEC * 10; // sec wait
    }
}

/**@brief Function for setting up GATTC notifications from the Notification Provider.
 *
 * @details This function is called when a successful connection has been established.
 */
static void apple_notification_setup(void)
{
    m_apple_setup_requested = true;
    // Delay because we cannot add a CCCD to close to starting encryption. iOS specific.
    m_wait_tick = 10;
}

static void apple_notification_setup_impl(void)
{
    ret_code_t ret;
    ret = ble_ancs_c_notif_source_notif_enable(&m_ancs_c);
    APP_ERROR_CHECK(ret);

    ret = ble_ancs_c_data_source_notif_enable(&m_ancs_c);
    APP_ERROR_CHECK(ret);

    NRF_LOG_DEBUG("Notifications Enabled.");
}

/**@brief Function for printing an iOS notification.
 *
 * @param[in] p_notif  Pointer to the iOS notification.
 */
static void notif_pre_process(ble_ancs_c_evt_notif_t * p_notif)
{
    ret_code_t ret;
    if (p_notif->evt_id != BLE_ANCS_EVENT_ID_NOTIFICATION_ADDED)
    {
        // イベント種類がAdded以外は無視する
        return;
    }
    if (p_notif->evt_flags.silent == 1 ||
        p_notif->evt_flags.pre_existing == 1)
    {
        // 通知の属性が、Silent, 既存データの場合は無視する
        return;
    }
    
    if (m_notif_category_id != NOTIF_CAT_UNDEF) {
        // 処理中なのでスキップ
        return;
    }

    if (m_sleep_mode) {
        // Sleep Modeの場合はスキップ
        return;
    }
    
    if (m_wait_tick > 0) {
        // 処理中なのでスキップ、wait_tickが0になるまでは、新規notifを受け付けない
        return;
    }

    if (p_notif->category_id == BLE_ANCS_CATEGORY_ID_INCOMING_CALL)
    {
        // TELの場合
        m_notif_category_id = NOTIF_CAT_TEL;
        ret = nrf_ble_ancs_c_request_attrs(&m_ancs_c, &m_notification_latest);
        APP_ERROR_CHECK(ret);
    }
    else if (p_notif->category_id == BLE_ANCS_CATEGORY_ID_MISSED_CALL ||
        p_notif->category_id == BLE_ANCS_CATEGORY_ID_VOICE_MAIL )
    {
        // 着信逃し、留守電の場合、無視する
        return;
    }
    else if (p_notif->category_id == BLE_ANCS_CATEGORY_ID_EMAIL)
    {
        // Emailの場合のみ
        m_notif_category_id = NOTIF_CAT_EMAIL;
        ret = nrf_ble_ancs_c_request_attrs(&m_ancs_c, &m_notification_latest);
        APP_ERROR_CHECK(ret);
    }
    else if (p_notif->category_id == BLE_ANCS_CATEGORY_ID_SOCIAL || // Facebook, Instagram, LINE, Skype
        p_notif->category_id == BLE_ANCS_CATEGORY_ID_OTHER || // LINE
        p_notif->category_id == BLE_ANCS_CATEGORY_ID_NEWS ) // Twitter
    {
        // 通知の種類が、Social/News Appの場合のみ
        ret = nrf_ble_ancs_c_request_attrs(&m_ancs_c, &m_notification_latest);
        APP_ERROR_CHECK(ret);
    }
    else
    {
        // 通知の種類がその他、全部
        // カレンダー：BLE_ANCS_CATEGORY_ID_SCHEDULE
        m_notif_category_id = NOTIF_CAT_OTHER;
        m_notif_src_name = "\0";
        m_notif_src_name_len = 0;
        m_notif_arrived = true;
    }
}

/**@brief Function for printing an iOS notification.
 *
 * @param[in] p_notif  Pointer to the iOS notification.
 */
static void notif_attr_process(ble_ancs_c_attr_t * p_attr)
{
    char *app_id_str;
    if (p_attr->attr_len == 0)
    {
        return;
    }

    if (p_attr->attr_id == 0) {
        //
        // App Bundle Identifier
        //
        app_id_str = (char *)p_attr->p_attr_data;
        if (!strcmp(app_id_str, "jp.naver.line")) 
        {
            // LINE
            m_notif_category_id = NOTIF_CAT_LINE;
        }
        else if (!strcmp(app_id_str, "com.facebook.Facebook")) 
        {
            // Facebook
            m_notif_category_id = NOTIF_CAT_FB;
        }
        else if (!strcmp(app_id_str, "com.atebits.Tweetie2")) 
        {
            // Twitter
            m_notif_category_id = NOTIF_CAT_TW;
        }
        else if (!strcmp(app_id_str, "com.burbn.instagram")) 
        {
            // Instagram
            m_notif_category_id = NOTIF_CAT_INSTA;
        }
        else if (m_notif_category_id == NOTIF_CAT_UNDEF)
        {
            // Other
            m_notif_category_id = NOTIF_CAT_OTHER;
        }
    }

    if (p_attr->attr_id == 1) {
        //
        // Attribute ID 1 = Title 
        //
        m_notif_src_name = (char *)p_attr->p_attr_data;
        m_notif_src_name_len = p_attr->attr_len;
        m_notif_arrived = true;
    }

    if (p_attr->attr_id == 3) {
        // Attribute ID 3 = Notification Message 
        // m_notif_src_name = (char *)p_attr->p_attr_data;
        // m_notif_src_name_len = p_attr->attr_len;
        m_notif_arrived = true;
    }
}

/**@brief Function for printing an iOS notification.
 *
 * @param[in] p_notif  Pointer to the iOS notification.
 */
static void notif_print(ble_ancs_c_evt_notif_t * p_notif)
{
    NRF_LOG_INFO("\r\nNotification");
    NRF_LOG_INFO("Event:       %s", (uint32_t)lit_eventid[p_notif->evt_id]);
    NRF_LOG_INFO("Category ID: %s", (uint32_t)lit_catid[p_notif->category_id]);
    NRF_LOG_INFO("Category Cnt:%u", (unsigned int) p_notif->category_count);
    NRF_LOG_INFO("UID:         %u", (unsigned int) p_notif->notif_uid);

    NRF_LOG_INFO("Flags:");
    if (p_notif->evt_flags.silent == 1)
    {
        NRF_LOG_INFO(" Silent");
    }
    if (p_notif->evt_flags.important == 1)
    {
        NRF_LOG_INFO(" Important");
    }
    if (p_notif->evt_flags.pre_existing == 1)
    {
        NRF_LOG_INFO(" Pre-existing");
    }
    if (p_notif->evt_flags.positive_action == 1)
    {
        NRF_LOG_INFO(" Positive Action");
    }
    if (p_notif->evt_flags.negative_action == 1)
    {
        NRF_LOG_INFO(" Negative Action");
    }
}


/**@brief Function for printing iOS notification attribute data.
 *
 * @param[in] p_attr Pointer to an iOS notification attribute.
 */
static void notif_attr_print(ble_ancs_c_attr_t * p_attr)
{
    if (p_attr->attr_len != 0)
    {
        NRF_LOG_INFO("%s: %s", (uint32_t)lit_attrid[p_attr->attr_id], nrf_log_push((char *)p_attr->p_attr_data));
    }
    else if (p_attr->attr_len == 0)
    {
        NRF_LOG_INFO("%s: (N/A)", (uint32_t)lit_attrid[p_attr->attr_id]);
    }
}


/**@brief Function for printing iOS notification attribute data.
 *
 * @param[in] p_attr Pointer to an iOS App attribute.
 */
static void app_attr_print(ble_ancs_c_attr_t * p_attr)
{
    if (p_attr->attr_len != 0)
    {
        NRF_LOG_INFO("%s: %s", (uint32_t)lit_appid[p_attr->attr_id], (uint32_t)p_attr->p_attr_data);
    }
    else if (p_attr->attr_len == 0)
    {
        NRF_LOG_INFO("%s: (N/A)", (uint32_t) lit_appid[p_attr->attr_id]);
    }
}


/**@brief Function for printing out errors that originated from the Notification Provider (iOS).
 *
 * @param[in] err_code_np Error code received from NP.
 */
static void err_code_print(uint16_t err_code_np)
{
    switch (err_code_np)
    {
        case BLE_ANCS_NP_UNKNOWN_COMMAND:
            NRF_LOG_INFO("Error: Command ID was not recognized by the Notification Provider. ");
            break;

        case BLE_ANCS_NP_INVALID_COMMAND:
            NRF_LOG_INFO("Error: Command failed to be parsed on the Notification Provider. ");
            break;

        case BLE_ANCS_NP_INVALID_PARAMETER:
            NRF_LOG_INFO("Error: Parameter does not refer to an existing object on the Notification Provider. ");
            break;

        case BLE_ANCS_NP_ACTION_FAILED:
            NRF_LOG_INFO("Error: Perform Notification Action Failed on the Notification Provider. ");
            break;

        default:
            break;
    }
}
/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t ret;

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    // Create security request timer.
    ret = app_timer_create(&m_sec_req_timer_id,
                           APP_TIMER_MODE_SINGLE_SHOT,
                           sec_req_timeout_handler);
    APP_ERROR_CHECK(ret);

    // Create thread timer.
    ret = app_timer_create(&m_thread_timer_id,
                           APP_TIMER_MODE_REPEATED,
                           thread_process_handler);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling the Apple Notification Service client.
 *
 * @details This function is called for all events in the Apple Notification client that
 *          are passed to the application.
 *
 * @param[in] p_evt  Event received from the Apple Notification Service client.
 */
static void on_ancs_c_evt(ble_ancs_c_evt_t * p_evt)
{
    ret_code_t ret = NRF_SUCCESS;

    switch (p_evt->evt_type)
    {
        case BLE_ANCS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_DEBUG("Apple Notification Center Service discovered on the server.");
            ret = nrf_ble_ancs_c_handles_assign(&m_ancs_c, p_evt->conn_handle, &p_evt->service);
            APP_ERROR_CHECK(ret);
            apple_notification_setup();
            break;

        case BLE_ANCS_C_EVT_NOTIF:
            m_notification_latest = p_evt->notif;
            notif_print(&m_notification_latest);
            notif_pre_process(&m_notification_latest);
            break;

        case BLE_ANCS_C_EVT_NOTIF_ATTRIBUTE:
            m_notif_attr_latest = p_evt->attr;
            notif_attr_print(&m_notif_attr_latest);
            notif_attr_process(&m_notif_attr_latest);
            if (p_evt->attr.attr_id == BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER)
            {
                m_notif_attr_app_id_latest = p_evt->attr;
            }
            break;
        case BLE_ANCS_C_EVT_DISCOVERY_FAILED:
            NRF_LOG_DEBUG("Apple Notification Center Service not discovered on the server.");
            break;

        case BLE_ANCS_C_EVT_APP_ATTRIBUTE:
            app_attr_print(&p_evt->attr);
            break;
        case BLE_ANCS_C_EVT_NP_ERROR:
            err_code_print(p_evt->err_code_np);
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing GAP connection parameters.
 *
 * @details Use this function to set up all necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    ret_code_t              ret;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    // Device Name : Se_Cret-abcd
    uint8_t *dname[ strlen(DEVICE_NAME) + 2 ];
    uint32_t did0 = NRF_FICR->DEVICEID[0];
    char cbuf[8];
    strcat((char *)dname, DEVICE_NAME);
    sprintf(cbuf, "%x", (did0 >> 0 * 8) & 0xff);
    strcat((char *)dname, cbuf);
    sprintf(cbuf, "%x", (did0 >> 1 * 8) & 0xff);
    strcat((char *)dname, cbuf);
    ret = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)dname, strlen(dname));
    APP_ERROR_CHECK(ret);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    ret = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(ret);
}

static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_cur_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t ret = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling the Apple Notification Service client errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void apple_notification_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             ret;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    ret = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Database Discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective service instances.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_ancs_c_on_db_disc_evt(&m_ancs_c, p_evt);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           ret;

    ret = pm_init();
    APP_ERROR_CHECK(ret);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    ret = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(ret);

    ret = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(ret);
}


/**
 * @brief Delete all data stored for all peers
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    NRF_LOG_INFO("Enter sleep_mode ...");
    // uint32_t ret = bsp_indication_set(BSP_INDICATE_IDLE);
    // APP_ERROR_CHECK(ret);

    // Prepare wakeup buttons.
    uint32_t ret = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(ret);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    ret = sd_power_system_off();
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling advertising events.
 *
 * @details This function is called for advertising events that are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t ret;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_DEBUG("Fast advertising");
            m_cur_adv_mode = BLE_ADV_EVT_FAST;
            // ret = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            // APP_ERROR_CHECK(ret);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_DEBUG("Slow advertising");
            m_cur_adv_mode = BLE_ADV_EVT_SLOW;
            // ret = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            // APP_ERROR_CHECK(ret);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_DEBUG("Fast advertising with Whitelist");
            m_cur_adv_mode = BLE_ADV_EVT_FAST_WHITELIST;
            // ret = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            // APP_ERROR_CHECK(ret);
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_DEBUG("Fast advertising with Whitelist");
            m_cur_adv_mode = BLE_ADV_EVT_SLOW_WHITELIST;
            // ret = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            // APP_ERROR_CHECK(ret);
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_DEBUG("BLE_ADV_EVT_IDLE");
            m_cur_adv_mode = BLE_ADV_EVT_IDLE;
            // sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            ret = pm_whitelist_get(whitelist_addrs, &addr_cnt, whitelist_irks, &irk_cnt);
            if (ret == NRF_ERROR_NOT_FOUND) {
                NRF_LOG_DEBUG("[WARN] No whitelist");
                return;
            }
            APP_ERROR_CHECK(ret);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                           addr_cnt,
                           irk_cnt);

            // Apply the whitelist.
            ret = ble_advertising_whitelist_reply(&m_advertising,
                                                  whitelist_addrs,
                                                  addr_cnt,
                                                  whitelist_irks,
                                                  irk_cnt);
            APP_ERROR_CHECK(ret);
        }
        break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t ret = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_DEBUG("Connected.");
            led_action_start(1); // 接続しました
            m_cur_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            ret               = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(ret);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_DEBUG("Disconnected.");
            m_cur_conn_handle = BLE_CONN_HANDLE_INVALID;
            ret               = app_timer_stop(m_sec_req_timer_id);
            APP_ERROR_CHECK(ret);

            if (p_ble_evt->evt.gap_evt.conn_handle == m_ancs_c.conn_handle)
            {
                m_ancs_c.conn_handle = BLE_CONN_HANDLE_INVALID;
            }
            led_action_start(1); // 接続解除しました
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_TIMEOUT:
            NRF_LOG_DEBUG("BLE_GAP_EVT_TIMEOUT");
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING) {
                if (m_cur_conn_handle == BLE_CONN_HANDLE_INVALID && m_cur_adv_mode == BLE_ADV_EVT_IDLE) {
                    // 継続してSlowAdvertisingをする
                    ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_SLOW);
                    APP_ERROR_CHECK(ret);
                }
            }
            break; // BLE_GAP_EVT_TIMEOUT

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            ret = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(ret);
        } break;
#endif
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            ret = sd_ble_gatts_sys_attr_set(m_cur_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(ret);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            ret = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(ret);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            ret = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(ret);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            ret = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(ret);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    ret = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(ret);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
    APP_ERROR_CHECK(ret);
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case BUTTON_1:
        {
            NRF_LOG_DEBUG("BSP_EVENT_KEY_0");
            err_code = nrf_ble_ancs_c_request_attrs(&m_ancs_c, &m_notification_latest);
            APP_ERROR_CHECK(err_code);
            break;
        }
        default:
            NRF_LOG_DEBUG("BSP_EVENT_KEY none");
            break;
    }
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t ret;
    NRF_LOG_DEBUG("bsp_event_handler : %d", event);

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            // sleep_mode_enter();
            NRF_LOG_DEBUG(" - no longer going to sleep");
            break;

        case BSP_EVENT_DISCONNECT:
            ret = sd_ble_gap_disconnect(m_cur_conn_handle,
                                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (ret != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(ret);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_ancs_c.conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                ret = ble_advertising_restart_without_whitelist(&m_advertising);
                if (ret != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(ret);
                }
            }
            break;

        case BSP_EVENT_KEY_0:
            NRF_LOG_DEBUG("BSP_EVENT_KEY_0");
            ret = nrf_ble_ancs_c_request_attrs(&m_ancs_c, &m_notification_latest);
            APP_ERROR_CHECK(ret);
            break;

        case BSP_EVENT_KEY_1:
            NRF_LOG_DEBUG("BSP_EVENT_KEY_1");
            if (m_notif_attr_app_id_latest.attr_id == BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER
                && m_notif_attr_app_id_latest.attr_len != 0)
            {
                NRF_LOG_DEBUG("Request for %s: ", (uint32_t)m_notif_attr_app_id_latest.p_attr_data);
                ret = nrf_ble_ancs_c_app_attr_request(&m_ancs_c,
                                                      m_notif_attr_app_id_latest.p_attr_data,
                                                      m_notif_attr_app_id_latest.attr_len);
                APP_ERROR_CHECK(ret);
            }
            break;

        case BSP_EVENT_KEY_2:
            if (m_notification_latest.evt_flags.positive_action == true)
            {
                NRF_LOG_DEBUG("Performing Positive Action.");
                ret = nrf_ancs_perform_notif_action(&m_ancs_c,
                                                    m_notification_latest.notif_uid,
                                                    ACTION_ID_POSITIVE);
                APP_ERROR_CHECK(ret);
            }
            break;

        case BSP_EVENT_KEY_3:
            if (m_notification_latest.evt_flags.negative_action == true)
            {
                NRF_LOG_DEBUG("Performing Negative Action.");
                ret = nrf_ancs_perform_notif_action(&m_ancs_c,
                                                    m_notification_latest.notif_uid,
                                                    ACTION_ID_NEGATIVE);
                APP_ERROR_CHECK(ret);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing the Apple Notification Center Service.
 */
static void services_init(void)
{
    ble_ancs_c_init_t ancs_init_obj;
    ble_dis_init_t dis_init;
    ret_code_t        ret;

    memset(&ancs_init_obj, 0, sizeof(ancs_init_obj));

    ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                  BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER,
                                  m_attr_appid,
                                  ATTR_DATA_SIZE);
    APP_ERROR_CHECK(ret);

    ret = nrf_ble_ancs_c_app_attr_add(&m_ancs_c,
                                      BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME,
                                      m_attr_disp_name,
                                      sizeof(m_attr_disp_name));
    APP_ERROR_CHECK(ret);

    ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                  BLE_ANCS_NOTIF_ATTR_ID_TITLE,
                                  m_attr_title,
                                  ATTR_DATA_SIZE);
    APP_ERROR_CHECK(ret);

    ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                  BLE_ANCS_NOTIF_ATTR_ID_MESSAGE,
                                  m_attr_message,
                                  ATTR_DATA_SIZE);
    APP_ERROR_CHECK(ret);

    ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                  BLE_ANCS_NOTIF_ATTR_ID_SUBTITLE,
                                  m_attr_subtitle,
                                  ATTR_DATA_SIZE);
    APP_ERROR_CHECK(ret);

    ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                  BLE_ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE,
                                  m_attr_message_size,
                                  ATTR_DATA_SIZE);
    APP_ERROR_CHECK(ret);

    ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                  BLE_ANCS_NOTIF_ATTR_ID_DATE,
                                  m_attr_date,
                                  ATTR_DATA_SIZE);
    APP_ERROR_CHECK(ret);

    ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                  BLE_ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL,
                                  m_attr_posaction,
                                  ATTR_DATA_SIZE);
    APP_ERROR_CHECK(ret);

    ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
                                  BLE_ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL,
                                  m_attr_negaction,
                                  ATTR_DATA_SIZE);
    APP_ERROR_CHECK(ret);

    ancs_init_obj.evt_handler   = on_ancs_c_evt;
    ancs_init_obj.error_handler = apple_notification_error_handler;

    ret = ble_ancs_c_init(&m_ancs_c, &ancs_init_obj);
    APP_ERROR_CHECK(ret);

    //---------------------------------------
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)DIS_MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)DIS_MODEL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char *)DIS_SERIAL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)DIS_HW_REV);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)DIS_FW_REV);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    ret = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(ret);

    //---------------------------------------
    // Initialize NUS.
    ret = ble_sss_init();
    APP_ERROR_CHECK(ret);

    //---------------------------------------
    // 電波送信強度
    ret = sd_ble_gap_tx_power_set(APP_DEFAULT_TX_POWER);
    APP_ERROR_CHECK(ret);
}

/**@brief Function for initializing the advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             ret;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    static ble_uuid_t m_adv_solicited_uuids[1]; /**< Universally unique service identifiers. */

    m_adv_solicited_uuids[0].uuid = ANCS_UUID_SERVICE;
    m_adv_solicited_uuids[0].type = m_ancs_c.service.service.uuid.type;

    static ble_uuid_t m_adv_uuids[2]; /**< Universally unique service identifiers. */

    // Device information service
    m_adv_uuids[0].uuid = BLE_UUID_DEVICE_INFORMATION_SERVICE;
    m_adv_uuids[0].type = BLE_UUID_TYPE_BLE;
    // Nordic UART service
    m_adv_uuids[1].uuid = BLE_UUID_NUS_SERVICE;
    m_adv_uuids[1].type = NUS_SERVICE_UUID_TYPE;

    init.advdata.name_type                = BLE_ADVDATA_NO_NAME;
    init.advdata.include_appearance       = true;
    init.advdata.flags                    = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.uuids_solicited.uuid_cnt = sizeof(m_adv_solicited_uuids) / sizeof(m_adv_solicited_uuids[0]);
    init.advdata.uuids_solicited.p_uuids  = m_adv_uuids;

    // Scan Response Data
    init.srdata.name_type                = BLE_ADVDATA_FULL_NAME;
    init.srdata.uuids_complete.uuid_cnt  = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids   = m_adv_uuids;

    if (m_sleep_mode == false) {
        NRF_LOG_INFO(" - Advertising in NORMAL mode");
        init.config.ble_adv_whitelist_enabled = true;
        init.config.ble_adv_fast_enabled      = true;
        init.config.ble_adv_fast_interval     = APP_ADV_FAST_INTERVAL;
        init.config.ble_adv_fast_timeout      = APP_ADV_FAST_TIMEOUT;
        init.config.ble_adv_slow_enabled      = true;
        init.config.ble_adv_slow_interval     = APP_ADV_SLOW_INTERVAL;
        init.config.ble_adv_slow_timeout      = APP_ADV_SLOW_TIMEOUT;
    } else {
        NRF_LOG_INFO(" - Advertising in SLEEP mode");
        init.config.ble_adv_whitelist_enabled = true;
        init.config.ble_adv_fast_enabled      = true;
        init.config.ble_adv_fast_interval     = APP_ADV_SLEEP_FAST_INTERVAL;
        init.config.ble_adv_fast_timeout      = APP_ADV_SLEEP_FAST_TIMEOUT;
        init.config.ble_adv_slow_enabled      = true;
        init.config.ble_adv_slow_interval     = APP_ADV_SLEEP_SLOW_INTERVAL;
        init.config.ble_adv_slow_timeout      = APP_ADV_SLEEP_SLOW_TIMEOUT;
    }

    init.evt_handler = on_adv_evt;

    ret = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(ret);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  True if the clear bonds button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    uint32_t ret;
    bsp_event_t startup_event;

    ret = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(ret);

    ret = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(ret);

    //The array must be static because a pointer to it will be saved in the button handler module.
    // static app_button_cfg_t buttons[] =
    // {
    //     {0x02, APP_BUTTON_ACTIVE_HIGH, NRF_GPIO_PIN_NOPULL, button_event_handler}
    // };

    // ret = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
    //                            BUTTON_DETECTION_DELAY);
    // APP_ERROR_CHECK(ret);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the Event Scheduler.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing the database discovery module.
 */
static void db_discovery_init(void)
{
    ret_code_t ret = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    ret_code_t ret = sd_app_evt_wait();
    APP_ERROR_CHECK(ret);
}

/**@brief   Wait for fds to initialize. */
static void wait_for_fds_ready(void)
{
    while (!m_fds_initialized)
    {
        power_manage();
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    bool erase_bonds = false;

    // Initialize.
    log_init();
    
    //Enabling the DCDC converter for lower current consumption
    NRF_POWER->DCDCEN = 1;

    NRF_LOG_INFO(" - Initializing Libs...");
    timers_init();
    buttons_leds_init(&erase_bonds);
    scheduler_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    db_discovery_init();
    services_init();

    // Start execution.
    NRF_LOG_INFO(" - Initializing LEDs...");
    leds_init();

    NRF_LOG_INFO(" - Initializing Vibrators...");
    vib_init();

    NRF_LOG_INFO(" - Initializing FDS...");
    err_code = fds_register(fds_evt_handler);
    if (err_code != FDS_SUCCESS) {
        fds_debug_error(err_code);
    }
    err_code = fds_init();
    if (err_code != FDS_SUCCESS) {
        fds_debug_error(err_code);
    }
    APP_ERROR_CHECK(err_code);

    /* Wait for fds to initialize. */
    // wait_for_fds_ready();

    // fds_stat_t stat = {0};
    // err_code = fds_stat(&stat);
    // APP_ERROR_CHECK(err_code);

    // NRF_LOG_DEBUG(" - Found %d valid records.", stat.valid_records);
    // NRF_LOG_DEBUG(" - Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

    NRF_LOG_INFO(" - Initializing SAADC...");
    saadc_init();
    m_battery_monitor_requested = true;
    m_wait_tick = 30;

    ble_sss_load_settings();

    // Advertising準備
    advertising_init();
    conn_params_init();
    peer_manager_init();
    advertising_start(erase_bonds);

    // スレッド処理カウンタの実行
    err_code = app_timer_start(m_thread_timer_id, THREAD_PROCESS_TICK_MS, NULL);
    APP_ERROR_CHECK(err_code);

    // LED : 起動完了通知LED点滅
    if (m_sleep_mode) {
        // SleepModeに入っていた場合、
        led_action_start(2);
    } else {
        // 通常起動
        led_action_start(1);
    }

    NRF_LOG_INFO("\n**** SeCret with ANCS-client started. ****");
    
    // Enter main loop.
    for (;;)
    {
        if(m_saadc_calibrate == true)
        {
            // NRF_LOG_INFO("SAADC calibration starting...");    //Print on UART
            while(nrf_drv_saadc_calibrate_offset() != NRF_SUCCESS); //Trigger calibration task
            m_saadc_calibrate = false;
        }
        
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }

}


/**
 * @}
 */

