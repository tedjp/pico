#include "boards/pico.h"
#include "hardware/gpio.h"
#include "pico/time.h"

static const uint32_t PIN_MASK = 0x1ffffu; // 17 bits; pins GP0 -> GP16

uint32_t add_tick(uint32_t semiSecs) {
    ++semiSecs;
    // wrap daily
    semiSecs %= 86400;
    return semiSecs;
}

static void set_gpio_pins(uint32_t semiSecs) {
    uint32_t pinState = semiSecs / 2;

#if defined(PICO_DEFAULT_LED_PIN)
    // Set default LED to low bit too
    pinState |= (semiSecs & 0x01) << PICO_DEFAULT_LED_PIN;
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

int main() {
    setup();

    absolute_time_t nextWakeup = get_absolute_time();
    uint32_t semiSecs = 0;

    for (;;) {
        nextWakeup = delayed_by_ms(nextWakeup, 500);
        sleep_until(nextWakeup);
        semiSecs = add_tick(semiSecs);
        set_gpio_pins(semiSecs);
    }

    return 0;
}
