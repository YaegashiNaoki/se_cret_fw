/**
 * LED controller
 */

#ifndef LED_H__
#define LED_H__

#include <stdint.h>
#include <stdbool.h>

void leds_init(void);
void led_on_blue();
void led_on_red();
void led_off();
void led_toggle();
void led_blink(uint8_t count);
void led_blink_red(uint8_t count);

void led_action_start(uint8_t action_id);
void led_process_next_tick(void);
void led_notif(uint8_t count);

#endif // LED_H__
