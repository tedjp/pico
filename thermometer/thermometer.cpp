#include <cmath>

#include "hardware/adc.h"
#include "pico/time.h"

static constexpr int NUM_DIGITS = 3;
// common pin for each significant digit begins at this gpio
// Suggest starting at GPIO 2 or higher to reserve 0 & 1 for UART.
static constexpr int least_significant_digit_common_gpio = 2;
// segment pins begin at this gpio
static constexpr int first_segment_gpio = 6;

// measured at 3.28 with external reference, but readings indicate 3.3 closer to
// reality; perhaps the error was in the reference measurement.
static constexpr float adc_voltage = 3.3f;
static constexpr float temp_scale = adc_voltage / (1u << 12);
// TODO: Increase to 10... maybe poll temp on core 1 while core 0 updates the
// display.
// TODO more: Have PIO run the LED pins and only wake up to update the reading.
static constexpr int secs_between_updates = 1;

static constexpr uint32_t get_common_gpio_mask() noexcept {
    uint32_t mask = 0;
    for (int i = 0; i < NUM_DIGITS; ++i) {
        mask |= 1ul << (least_significant_digit_common_gpio + i);
    }

    return mask;
}

// GPIO masks used repeatedly when setting digit (common) and segment GPIOs
static constexpr uint32_t segment_gpio_mask = 0xff << first_segment_gpio;
static constexpr uint32_t common_gpio_mask = get_common_gpio_mask();
static constexpr uint32_t all_digit_gpios_mask = segment_gpio_mask | common_gpio_mask;

static float get_temperature_celsius() {
    // make sure resulting type is big enough to hold max 12-bit value (4095) x
    // numSamples.
    constexpr int numSamples = 4;
    uint32_t accumulatedSamples = 0;
    static_assert(std::numeric_limits<decltype(accumulatedSamples)>::max() / (1U << 12) >= numSamples,
            "accumulatedSamples might wraparound, use fewer samples or a larger type");
    for (int i = 0; i < numSamples; ++i)
        accumulatedSamples += adc_read();

    float adcTemp = accumulatedSamples / float(numSamples);

    // from datasheet
    return 27.0f - (adcTemp * temp_scale - 0.706f) / 0.001721f;
}

static float celsius_to_fahrenheit(float degrees_celsius) {
    return degrees_celsius * 9 / 5 + 32;
}

static float get_temperature_farenheit() {
    return celsius_to_fahrenheit(get_temperature_celsius());
}

static void setup() {
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4); // temp sensor

    gpio_init_mask(all_digit_gpios_mask);
    gpio_set_dir_out_masked(all_digit_gpios_mask);
}

// Allow easy software remapping of LED pins.
// Letters are the typical clockwise assignment with A at the top, B upper
// right, etc.
namespace led_pins {
    constexpr uint8_t _  = 0b00000000;
    constexpr uint8_t a  = 0b00000001;
    constexpr uint8_t b  = 0b00000010;
    constexpr uint8_t c  = 0b00000100;
    constexpr uint8_t d  = 0b00001000;
    constexpr uint8_t e  = 0b00010000;
    constexpr uint8_t f  = 0b00100000;
    constexpr uint8_t g  = 0b01000000;
    constexpr uint8_t dp = 0b10000000;

    // 7-segment display pin bit pattern for digits [0-9].
    static constexpr uint8_t display_pins[10] = {
        a | b | c | d | e | f | _,
        _ | b | c | _ | _ | _ | _,
        a | b | _ | d | e | _ | g,
        a | b | c | d | _ | _ | g,
        _ | b | c | _ | _ | f | g,
        a | _ | c | d | _ | f | g,
        a | _ | c | d | e | f | g,
        a | b | c | _ | _ | _ | _,
        a | b | c | d | e | f | g,
        a | b | c | d | _ | f | g,
    };
} // namespace led_pins

// place: 0 for the least significant digit, etc.
// value: 0-9.
// decimal_point: Whether to light the decimal point segment.
static void display_digit(int place, int_fast8_t value, bool decimal_point = false) {
    // don't trust the caller
    value %= 10;
    uint8_t pins = led_pins::display_pins[value];
    if (decimal_point)
        pins |= led_pins::dp;

    // flip segment pins lit=LOW dark=HIGH (common anode)
    pins = ~pins;
    uint32_t gpio_pins = static_cast<uint32_t>(pins) << first_segment_gpio;

    // Set the digit's common pin HIGH
    gpio_pins |= 1ul << (least_significant_digit_common_gpio + place);

    // Set all 7-segment display pins (segment & digit selector) in one call.
    gpio_put_masked(all_digit_gpios_mask, gpio_pins);
}

static void clear_digits() {
    // set common & segment GPIOs LOW
    gpio_clr_mask(all_digit_gpios_mask);
}

static void display(float value) {
    float dekaValue = value * 10.0f;
    dekaValue = std::round(dekaValue);
    for (int i = 0; i < 30; ++i) {
        display_digit(2, static_cast<uint_fast8_t>(dekaValue / 100) % 10);
        sleep_ms(10);
        display_digit(1, static_cast<uint_fast8_t>(dekaValue / 10) % 10, true);
        sleep_ms(10);
        display_digit(0, static_cast<uint_fast8_t>(dekaValue) % 10);
        sleep_ms(10);
    }
}

int main() {
    setup();

    absolute_time_t nextWakeup = get_absolute_time();
    float temperature_celsius = 0.0f;

    for (;;) {
        temperature_celsius = get_temperature_celsius();
        display(celsius_to_fahrenheit(temperature_celsius));
        nextWakeup = delayed_by_ms(nextWakeup, secs_between_updates * 1000);
        sleep_until(nextWakeup);
    }

    return 0;
}
