/**
 * SeCret Setting Commands
 */
#ifndef CMDS_H__
#define CMDS_H__

//------------------------------------------------------------------------------
//C701A9C6-5A0C-43C0-A2E1-3FE116F9316A
//32380001-F43D-4025-996A-6336FF62B155
// #define BLE_UUID_SSS_SERVICE {{0x32, 0x38, 0x00, 0x01, 0xF4, 0x3D, 0x40, 0x25, 0x99, 0x6A, 0x63, 0x36, 0xFF, 0x62, 0xB1, 0x55}}

// #define SSS_UUID_WRITE_SERVICE           0x0001
// #define SSS_UUID_RX_CHAR                 0x0002
// #define SSS_UUID_TX_CHAR                 0x0003

//------------------------------------------------------------------------------
// Command Packet Format
#define SSS_CMD_SEQ_START                0xFF
#define SSS_CMD_SEQ_PARAM                0xFF
#define SSS_CMD_SEQ_END                  0xC0
#define SSS_CMD_MULTI_DATA               0xF0

//------------------------------------------------------------------------------
// Command Response
#define SSS_CMD_STATUS_OK                0x00
#define SSS_CMD_STATUS_NG                0xEE

//------------------------------------------------------------------------------
// Error Code
#define SSS_CMD_STATUS_LOW_BATTERY       0xE1

//------------------------------------------------------------------------------
// Action Commands
#define SSS_CMD_BLINK_LED                0x11
#define SSS_CMD_VIBRATION                0x12
#define SSS_CMD_EXEC_VIB_PATTERN         0x13

//------------------------------------------------------------------------------
// Setting Commands
#define SSS_CMD_COMMON_VIB_STRENGTH_SET  0x21
#define SSS_CMD_COMMON_VIB_STRENGTH_GET  0x22
#define SSS_CMD_COMMON_RECV_TARGET_SET   0x23
#define SSS_CMD_COMMON_RECV_TARGET_GET   0x24
#define SSS_CMD_COMMON_VIB_MUTE_SET      0x25
#define SSS_CMD_COMMON_VIB_MUTE_GET      0x26
#define SSS_CMD_COMMON_SLEEP_MODE_SET    0x27
#define SSS_CMD_COMMON_SLEEP_MODE_GET    0x28
//2019.09.20 Add Led & Vib setting option Start
#define SSS_CMD_COMMON_NOTIFY_MODE_SET   0x29
#define SSS_CMD_COMMON_NOTIFY_MODE_GET   0x2A
//2019.09.20 Add Led & Vib setting option End

#define SSS_CMD_PATTERN_RECV_TARGET_SET  0x31
#define SSS_CMD_PATTERN_RECV_TARGET_GET  0x32
#define SSS_CMD_PATTERN_RECV_TARGET_CLR  0x33
#define SSS_CMD_PAIRING_CLR              0x90

//------------------------------------------------------------------------------
// System Commands
#define SSS_CMD_SHUTDOWN                 0xD0
#define SSS_CMD_REBOOT                   0xD1
#define SSS_CMD_DISCONNECT               0xDC
#define SSS_CMD_ENTER_DFU_MODE           0xDF

#define SSS_CMD_FS_SAVE                  0x61
#define SSS_CMD_FS_LOAD                  0x62
#define SSS_CMD_FS_ERASE                 0x63

#endif // CMDS_H__
