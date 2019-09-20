/**
 * Vibration controller
 */

#ifndef VIBRATION_H__
#define VIBRATION_H__

#include <stdint.h>
#include <stdbool.h>

#define VIB_ON_H 0x03
#define VIB_ON_M 0x02
#define VIB_ON_L 0x01
#define VIB_OFF  0x00

typedef struct _vib_step {
    uint32_t time;
    uint8_t value;
} vib_step;

void vib_init(void);

void vib_set_common_strength(uint8_t strength_id);
uint8_t vib_get_common_strength(void);

void vib_on_h(void);
void vib_on_m(void);
void vib_on_l(void);
void vib_off(void);

void vib_on_sec(uint8_t seconds);

void vib_play_common(void);
void vib_play_pattern(uint8_t ptn_id);

void vib_process_next_tick(void);

//2019.09.19 宣言追加
void vib_action_start(vib_step* p_vib_seq_array);
void vib_set_mute(bool mute_enabled);
uint8_t vib_get_mute(void);
#endif // VIBRATION_H__
