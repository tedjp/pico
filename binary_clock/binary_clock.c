#include "boards/pico.h"
#include "hardware/gpio.h"
#include "pico/time.h"

static const uint32_t PIN_MASK = 0x1ffffu; // 17 bits; pins GP0 -> GP16

uint32_t add_tick(uint32_t secs) {
    ++secs;
    // wrap daily
    secs %= 86400;
    return secs;
}

static void update_pins(uint32_t value) {
    uint32_t pinState = value;

#if defined(PICO_DEFAULT_LED_PIN)
    // Set default LED to low bit too
    pinState |= (value & 0x01) << PICO_DEFAULT_LED_PIN;
#endif

    gpio_put_all(pinState);
}

static void setup() {
    uint32_t pinMask = PIN_MASK;

#if defined(PICO_DEFAULT_LED_PIN)
    pinMask |= 1u << PICO_DEFAULT_LED_PIN;
#endif

    gpio_init_mask(pinMask);
    gpio_set_dir_out_masked(pinMask);
}

void main() {
    setup();

    absolute_time_t nextWakeup = get_absolute_time();
    uint32_t secs = 0;

    for (;;) {
        nextWakeup = delayed_by_ms(nextWakeup, 1000);
        sleep_until(nextWakeup);
        secs = add_tick(secs);
        update_pins(secs);
    }
}
