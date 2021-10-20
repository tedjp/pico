#include "boards/pico.h"
#include "hardware/gpio.h"
#include "pico/time.h"

static uint32_t gCounter = 0;

static const uint32_t PIN_MASK = 0x1ffffu; // 17 bits; pins GP0 -> GP16

static bool update_clock(repeating_timer_t *rt) {
    ++gCounter;

    // Reset daily
    gCounter %= 86400;

    uint32_t pinState = gCounter;

#if defined(PICO_DEFAULT_LED_PIN)
    // Set default LED to low bit too
    pinState |= (gCounter & 0x01) << PICO_DEFAULT_LED_PIN;
#endif

    gpio_put_all(pinState);

    // return true to keep repeating
    return true;
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

    static repeating_timer_t timer_1hz;
    // negative interval schedules the start times to align; positive interval
    // waits the duration after each callback completes, which would drift.
    add_repeating_timer_ms(-1000, &update_clock, NULL, &timer_1hz);

    for (;;)
        sleep_ms(1000); // FIXME: This probably drifts
}
