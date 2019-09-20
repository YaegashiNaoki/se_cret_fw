#include "led.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_config.h"
#include "app_error.h"
#include "vibration.h"

static bool m_led_is_on = false;
uint32_t led_action_count = 0;
uint8_t led_action_index = 0;
uint32_t * led_seq_array;
uint8_t led_seq_length = 0;

//------------------------------------------------------------------------------
/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application and starts softblink timer.
 */
void leds_init(void)
{
    // Configure LED-pins as outputs.
    nrf_gpio_cfg_output(SECRET_LED_BLUE);
    nrf_gpio_cfg_output(SECRET_LED_RED);

    // Reset
    led_off();
}

//------------------------------------------------------------------------------
void led_on_blue()
{
    nrf_gpio_pin_set(SECRET_LED_BLUE);
    m_led_is_on = true;
}

void led_on_red()
{
    nrf_gpio_pin_set(SECRET_LED_RED);
    m_led_is_on = true;
}

void led_off()
{
    nrf_gpio_pin_clear(SECRET_LED_BLUE);
    nrf_gpio_pin_clear(SECRET_LED_RED);
    m_led_is_on = false;
}

void led_toggle()
{
    if (m_led_is_on) {
        led_off();
    } else {
        led_on_blue();
    }
}

void led_blink(uint8_t count)
{
    for (uint8_t i=0; i < count; i++) {
        led_on_blue();
        nrf_delay_ms(1000);
        led_off();
        nrf_delay_ms(1000);
    }
}


void led_notif(uint8_t count)
{
    if(vib_get_mute()) return;
    for (uint8_t i=0; i < count; i++) {
        led_on_blue();
        nrf_delay_ms(200);
        led_off();
        nrf_delay_ms(200);
    }
}

void led_blink_red(uint8_t count)
{
    for (uint8_t i=0; i < count; i++) {
        led_on_red();
        nrf_delay_ms(100);
        led_off();
        nrf_delay_ms(100);
    }
}

static uint32_t LED_SEQ_BLINK_01[6] = {
    1+2*0, // ON
    1+2*2, // OFF
    1+2*4, // ON
    1+2*6, // OFF
    1+2*8, // ON
    1+2*10, // OFF
};

static uint32_t LED_SEQ_BLINK_02[2] = {
    1+2*0, // ON
    1+2*2, // OFF
};

void led_action_start(uint8_t action_id)
{
    led_action_index = 0;
    led_action_count = 0;
    if (action_id == 1) {
        led_seq_array = LED_SEQ_BLINK_01;
        led_seq_length = 6;
    } else {
        led_seq_array = LED_SEQ_BLINK_02;
        led_seq_length = 2;
    }
    led_off();
}

void led_process_next_tick(void)
{
    if (led_action_index >= led_seq_length) {
        // アクション終了
        return;
    }

    led_action_count++;

    if (led_seq_array[ led_action_index ] == led_action_count) {
        led_toggle();
        led_action_index++;
    }
}