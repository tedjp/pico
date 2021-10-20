#include "boards/pico.h"
#include "hardware/gpio.h"
#include "pico/time.h"

static const uint32_t PIN_MASK = 0x1ffffu; // 17 bits; pins GP0 -> GP16

static uint32_t add_tick(uint32_t secs) {
    ++secs;
    // wrap daily
    secs %= 86400;
    return secs;
}

static void set_gpio_pins(uint32_t secs) {
    gpio_put_all(secs);
}

static void setup() {
    gpio_init_mask(PIN_MASK);
    gpio_set_dir_out_masked(PIN_MASK);
}

int main() {
    setup();

    absolute_time_t nextWakeup = get_absolute_time();
    uint32_t secs = 0;

    for (;;) {
        nextWakeup = delayed_by_ms(nextWakeup, 1000);
        sleep_until(nextWakeup);
        secs = add_tick(secs);
        set_gpio_pins(secs);
    }

    return 0;
}
