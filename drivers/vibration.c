#include "vibration.h"

#define NRF_LOG_MODULE_NAME vibration
#define NRF_LOG_LEVEL 4 // 4=DEBUG, 3=INFO
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_config.h"
#include "app_error.h"
#include "led.h"

static bool m_vib_is_on = false;
static bool m_vib_mute_enabled = false;
static uint8_t m_vib_strength_id = 2;
uint32_t vib_action_count = 0;
uint8_t vib_action_index = 0;
vib_step * vib_seq_array;
uint8_t vib_seq_length = 0;

#define TICK_SCALE 1

static vib_step VIB_SEQ_5SEC_L[2] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_L}, // ON
    {.time = 1+TICK_SCALE*5, .value = VIB_OFF},
};
static vib_step VIB_SEQ_5SEC_M[2] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*25, .value = VIB_OFF},
};
static vib_step VIB_SEQ_5SEC_H[2] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_H}, // ON
    {.time = 1+TICK_SCALE*25, .value = VIB_OFF},
};

#define VIB_SEQ_8SEC_L_LEN     14
static vib_step VIB_SEQ_8SEC_L[VIB_SEQ_8SEC_L_LEN] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_L}, // ON
    {.time = 1+TICK_SCALE*1, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*2, .value = VIB_ON_L}, // ON
    {.time = 1+TICK_SCALE*3, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*4, .value = VIB_ON_L}, // ON
    {.time = 1+TICK_SCALE*5, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*6, .value = VIB_ON_L}, // ON
    {.time = 1+TICK_SCALE*7, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*8, .value = VIB_ON_L}, // ON
    {.time = 1+TICK_SCALE*9, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*10, .value = VIB_ON_L}, // ON
    {.time = 1+TICK_SCALE*11, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*16, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*31, .value = VIB_OFF},
};

#define VIB_SEQ_8SEC_M_LEN     14
static vib_step VIB_SEQ_8SEC_M[VIB_SEQ_8SEC_M_LEN] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*1, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*2, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*3, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*4, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*5, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*6, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*7, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*8, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*9, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*10, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*11, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*16, .value = VIB_ON_M},
    {.time = 1+TICK_SCALE*31, .value = VIB_OFF},
};

#define VIB_SEQ_8SEC_H_LEN     14
static vib_step VIB_SEQ_8SEC_H[VIB_SEQ_8SEC_H_LEN] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*1, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*2, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*3, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*4, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*5, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*6, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*7, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*8, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*9, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*10, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*11, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*16, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*31, .value = VIB_OFF},
};

#define VIB_SEQ_BLINK_01_LEN     18
static vib_step VIB_SEQ_BLINK_01[VIB_SEQ_BLINK_01_LEN] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*1, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*2, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*3, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*4, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*5, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*6, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*7, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*8, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*9, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*10, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*11, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*15, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*20, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*25, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*30, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*35, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*40, .value = VIB_OFF},
};

#define VIB_SEQ_BLINK_02_LEN     24
static vib_step VIB_SEQ_BLINK_02[VIB_SEQ_BLINK_02_LEN] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*1, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*2, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*3, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*4, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*5, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*6, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*7, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*8, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*9, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*10, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*11, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*16, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*20, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*22, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*25, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*27, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*30, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*32, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*37, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*40, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*42, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*45, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*47, .value = VIB_OFF},
};

#define VIB_SEQ_BLINK_03_LEN     24
static vib_step VIB_SEQ_BLINK_03[VIB_SEQ_BLINK_03_LEN] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*1, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*2, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*3, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*4, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*5, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*6, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*7, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*8, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*9, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*10, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*11, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*17, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*18, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*19, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*20, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*21, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*22, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*25, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*26, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*27, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*28, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*29, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*34, .value = VIB_OFF},
};

#define VIB_SEQ_BLINK_04_LEN     24
static vib_step VIB_SEQ_BLINK_04[VIB_SEQ_BLINK_04_LEN] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*1, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*2, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*3, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*4, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*5, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*6, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*7, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*8, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*9, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*10, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*11, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*17, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*18, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*19, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*20, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*21, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*22, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*25, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*26, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*27, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*28, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*29, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*34, .value = VIB_OFF},
};

#define VIB_SEQ_BLINK_05_LEN     36
static vib_step VIB_SEQ_BLINK_05[VIB_SEQ_BLINK_05_LEN] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*1, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*2, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*3, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*4, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*5, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*6, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*7, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*8, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*9, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*10, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*11, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*17, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*20, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*21, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*22, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*23, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*24, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*27, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*28, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*29, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*30, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*31, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*32, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*35, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*40, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*41, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*42, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*43, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*44, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*45, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*46, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*47, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*48, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*49, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*50, .value = VIB_OFF},
};

#define VIB_SEQ_BLINK_06_LEN     24
static vib_step VIB_SEQ_BLINK_06[VIB_SEQ_BLINK_06_LEN] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*1, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*2, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*3, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*4, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*5, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*6, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*7, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*8, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*9, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*10, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*11, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*15, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*20, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*25, .value = VIB_ON_M},
    {.time = 1+TICK_SCALE*28, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*35, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*36, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*45, .value = VIB_ON_H},
    {.time = 1+TICK_SCALE*50, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*55, .value = VIB_ON_M},
    {.time = 1+TICK_SCALE*58, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*65, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*66, .value = VIB_OFF},
};

#define VIB_SEQ_BLINK_07_LEN     26
static vib_step VIB_SEQ_BLINK_07[VIB_SEQ_BLINK_07_LEN] = {
    {.time = 1+TICK_SCALE*0, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*1, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*2, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*3, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*4, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*5, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*6, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*7, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*8, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*9, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*10, .value = VIB_ON_M}, // ON
    {.time = 1+TICK_SCALE*11, .value = VIB_OFF}, // OFF
    {.time = 1+TICK_SCALE*15, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*16, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*17, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*18, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*19, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*20, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*21, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*22, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*23, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*24, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*25, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*26, .value = VIB_OFF},
    {.time = 1+TICK_SCALE*27, .value = VIB_ON_L},
    {.time = 1+TICK_SCALE*28, .value = VIB_OFF},
};


//------------------------------------------------------------------------------
void vib_init(void)
{
    // Configure pins as outputs.
    nrf_gpio_cfg_output(SECRET_VIBRATION_H);
    nrf_gpio_cfg_output(SECRET_VIBRATION_M);
    nrf_gpio_cfg_output(SECRET_VIBRATION_L);

    // Reset
    vib_off();
}

void vib_set_mute(bool mute_enabled)
{
    m_vib_mute_enabled = mute_enabled;
    NRF_LOG_DEBUG("vib_set_mute : mute=%d", m_vib_mute_enabled);
}

uint8_t vib_get_mute(void)
{
    return m_vib_mute_enabled;
}


void vib_set_common_strength(uint8_t strength_id)
{
    m_vib_strength_id = strength_id;
    NRF_LOG_DEBUG("vib_set_common_strength : strength_id=%d", m_vib_strength_id);
}

uint8_t vib_get_common_strength(void)
{
    return m_vib_strength_id;
}

//------------------------------------------------------------------------------
void vib_on_h(void)
{
    if (m_vib_mute_enabled) return;
    vib_off();
    nrf_gpio_pin_set(SECRET_VIBRATION_H);
    m_vib_is_on = true;
}

void vib_on_m(void)
{
    if (m_vib_mute_enabled) return;
	vib_off();
    nrf_gpio_pin_set(SECRET_VIBRATION_M);
    m_vib_is_on = true;
}

void vib_on_l(void)
{
    if (m_vib_mute_enabled) return;
    vib_off();
    nrf_gpio_pin_set(SECRET_VIBRATION_L);
    m_vib_is_on = true;
}

void vib_off(void)
{
    if (m_vib_mute_enabled) return;
    nrf_gpio_pin_clear(SECRET_VIBRATION_L);
    nrf_gpio_pin_clear(SECRET_VIBRATION_M);
    nrf_gpio_pin_clear(SECRET_VIBRATION_H);
    m_vib_is_on = false;
}

void vib_on_sec(uint8_t seconds)
{
    vib_action_index = 0;
    vib_action_count = 0;
    vib_seq_length = 2;
    if (m_vib_strength_id == 0x01) {
        vib_seq_array = VIB_SEQ_5SEC_L;
    } else if (m_vib_strength_id == 0x02) {
        vib_seq_array = VIB_SEQ_5SEC_M;
    } else {
        vib_seq_array = VIB_SEQ_5SEC_H;
    }
    NRF_LOG_DEBUG("vib_on_sec : length=%d", vib_seq_length);
    vib_off();
}

void vib_play_common(void)
{
    uint8_t strength_id = vib_get_common_strength();
    switch(strength_id) 
    {
        case 1:
            vib_action_start( VIB_SEQ_8SEC_L );
            vib_seq_length = VIB_SEQ_8SEC_L_LEN;
            break;
        case 2:
            vib_action_start( VIB_SEQ_8SEC_M );
            vib_seq_length = VIB_SEQ_8SEC_M_LEN;
            break;
        case 3:
            vib_action_start( VIB_SEQ_8SEC_H );
            vib_seq_length = VIB_SEQ_8SEC_H_LEN;
            break;
    }
}

void vib_play_pattern(uint8_t ptn_id)
{
    switch(ptn_id) 
    {
        case 1:
            vib_action_start( VIB_SEQ_BLINK_01 );
            vib_seq_length = VIB_SEQ_BLINK_01_LEN;
            break;
        case 2:
            vib_action_start( VIB_SEQ_BLINK_02 );
            vib_seq_length = VIB_SEQ_BLINK_02_LEN;
            break;
        case 3:
            vib_action_start( VIB_SEQ_BLINK_03 );
            vib_seq_length = VIB_SEQ_BLINK_03_LEN;
            break;
        case 4:
            vib_action_start( VIB_SEQ_BLINK_04 );
            vib_seq_length = VIB_SEQ_BLINK_04_LEN;
            break;
        case 5:
            vib_action_start( VIB_SEQ_BLINK_05 );
            vib_seq_length = VIB_SEQ_BLINK_05_LEN;
            break;
        case 6:
            vib_action_start( VIB_SEQ_BLINK_06 );
            vib_seq_length = VIB_SEQ_BLINK_06_LEN;
            break;
        case 7:
            vib_action_start( VIB_SEQ_BLINK_07 );
            vib_seq_length = VIB_SEQ_BLINK_07_LEN;
            break;
    }
}

void vib_action_start(vib_step* p_vib_seq_array)
{
    vib_action_index = 0;
    vib_action_count = 0;
    vib_seq_array = p_vib_seq_array;
    vib_seq_length = sizeof(vib_seq_array) / sizeof(vib_step);
    NRF_LOG_DEBUG("vib_action_start : length=%d", vib_seq_length);
    vib_off();
}

void vib_process_next_tick(void)
{
    if (vib_action_index >= vib_seq_length) {
        // アクション終了
        return;
    }

    vib_action_count++;

    if (vib_seq_array[ vib_action_index ].time == vib_action_count) {
        switch(vib_seq_array[ vib_action_index ].value)
        {
            case VIB_OFF:
            {
                vib_off();
                NRF_LOG_DEBUG(" -- vib_proc : off");
                break;
            }
            case VIB_ON_L:
            {
                vib_on_l();
                NRF_LOG_DEBUG(" -- vib_proc : on - low");
                break;
            }
            case VIB_ON_M:
            {
                vib_on_m();
                NRF_LOG_DEBUG(" -- vib_proc : on - mid");
                break;
            }
            case VIB_ON_H:
            {
                vib_on_h();
                NRF_LOG_DEBUG(" -- vib_proc : on - high");
                break;
            }
        }
        vib_action_index++;
    }
}