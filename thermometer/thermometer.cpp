#include <chrono>
#include <cmath>

#include "hardware/adc.h"
#include "pico/time.h"

static constexpr int NUM_DIGITS = 3;
// common pin for each significant digit begins at this gpio
// Suggest starting at GPIO 2 or higher to reserve 0 & 1 for UART.
static constexpr int least_significant_digit_common_gpio = 2;
// segment pins begin at this gpio
static constexpr int first_segment_gpio = 6;

// Temperature sensor ADC inputs
static constexpr int EXT_TEMP_SENSOR_ADC = 0;
static constexpr int PICO_TEMP_SENSOR_ADC = 4; // Raspberry Pi Pico
// ADC input tied to ground for more accurate calculaton of 0 mV ADC input.
static constexpr int ZERO_ADC = 2;

// ADC_VREF as measured. Nominally 3.3, but typically measures 3.27 to 3.28 V.
static constexpr float ADC_VOLTAGE = 3.27f;
static constexpr int ADC_BITS = 12;
static constexpr unsigned int ADC_MAX = 1u << ADC_BITS;
static constexpr float ADC_VOLTS_PER_UNIT = ADC_VOLTAGE / ADC_MAX;
// Future: PIO will update LED pins so the core can sleep between samples
// (likely 1 sample per second, display average of last 10 samples).
static constexpr std::chrono::seconds TEMP_UPDATE_INTERVAL = std::chrono::seconds(1);

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

static float get_temperature_celsius_pico() {
    // make sure resulting type is big enough to hold max 12-bit value (4095) x
    // numSamples.
    constexpr int numSamples = 4;
    uint32_t accumulatedSamples = 0;
    static_assert(std::numeric_limits<decltype(accumulatedSamples)>::max() / (1U << 12) >= numSamples,
            "accumulatedSamples might wraparound, use fewer samples or a larger type");
    adc_select_input(4);
    for (int i = 0; i < numSamples; ++i)
        accumulatedSamples += adc_read();

    // Get a sample of a tied-to-ground ADC input to subtract the baseline
    adc_select_input(2);
    uint32_t zero = 0;
    accumulatedSamples -= zero * numSamples;
    // Restore the input to the temperature sensor
    adc_select_input(4);

    float adcTemp = accumulatedSamples / float(numSamples);

    // from datasheet
    return 27.0f - (adcTemp * ADC_VOLTS_PER_UNIT - 0.706f) / 0.001721f;
}

static float get_temperature_celsius_external() {
    adc_select_input(EXT_TEMP_SENSOR_ADC);

    // Eventually replace with a single read and a history buffer that is
    // averaged.
    constexpr int numSamples = 4;
    uint32_t accumulatedSamples = 0;
    for (int i = 0; i < numSamples; ++i)
        accumulatedSamples += adc_read();

    const uint16_t adcValue = static_cast<uint16_t>(accumulatedSamples / numSamples);
    const float adcVolts = adcValue * ADC_VOLTS_PER_UNIT;
    const float adcMilliVolts = adcVolts * 1000.0f;
    // Datasheet: 10 mV per degree celsius
    const float degreesCelsiusAboveBase = adcMilliVolts / 10.0f;
    // TMP36 0 volts is -50 C (though its min spec is 100 mV aka -40 C).
    // (Max output is 2000 mV or 150 degrees Celsius, though its max spec is 125
    // degrees Celsius.)
    constexpr float TMP36_BASE_TEMP = -50.0f;
    return TMP36_BASE_TEMP + degreesCelsiusAboveBase;
}

static float get_temperature_celsius() {
    //return get_temperature_celsius_pico();
    return get_temperature_celsius_external();
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
    constexpr auto updateIntervalMS
        = std::chrono::duration_cast<std::chrono::milliseconds>(TEMP_UPDATE_INTERVAL);

    for (;;) {
        temperature_celsius = get_temperature_celsius();
        display(celsius_to_fahrenheit(temperature_celsius));
        nextWakeup = delayed_by_ms(nextWakeup, updateIntervalMS.count());
        sleep_until(nextWakeup);
    }

    return 0;
}
