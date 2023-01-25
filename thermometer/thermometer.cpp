#include <chrono>
#include <cmath>

#include "hardware/adc.h"
#include "hardware/pio.h"
#include "pico/time.h"

#include "segment_display.pio.h"

// Number of samples to average over, to smooth variance in the display.
// A longer period provides a steadier value at the expense of slower adaptation
// to changing conditions.
static constexpr std::chrono::seconds TEMP_SAMPLE_DURATION = std::chrono::seconds(60);
// Time between updates. Less frequent updates are more relaxing.
static constexpr std::chrono::seconds TEMP_UPDATE_INTERVAL = std::chrono::seconds(2);
static constexpr int TEMP_SAMPLES = TEMP_SAMPLE_DURATION / TEMP_UPDATE_INTERVAL;

// How many samples to read back-to-back each update.
// This is not worth setting higher than 1, averaging over a longer period is better.
static constexpr int SAMPLES_PER_UPDATE = 1;

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
    constexpr int numSamples = SAMPLES_PER_UPDATE;
    uint32_t accumulatedSamples = 0;
    static_assert(std::numeric_limits<decltype(accumulatedSamples)>::max() / (1U << 12) >= numSamples,
            "accumulatedSamples might wraparound, use fewer samples or a larger type");
    adc_select_input(4);
    for (int i = 0; i < numSamples; ++i)
        accumulatedSamples += adc_read();

    // Get a sample of a tied-to-ground ADC input to subtract the baseline
    // This might be excessive.
    adc_select_input(2);
    uint32_t zero = adc_read();
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
    constexpr int numSamples = SAMPLES_PER_UPDATE;
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

static void disable_unused_clocks() {
#if 0 // Enable after verifying PIO
    clocks_init();

    for (int clock = 0; clock < CLK_COUNT; ++clock) {
        // Keep clk_ref (XOSC, for sleeps), clk_sys (for CPU) & clk_adc running
        if (clock != clk_ref && clock != clk_sys && clock != clk_adc)
            clock_stop(clock);
    }
#endif
}

static void setup_adc() {
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4); // temp sensor
}

struct display_pio {
    PIO pio;
    uint offset;
    uint sm;
};

static display_pio setup_pio() {
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &segment_display_program);
    uint sm = pio_claim_unused_sm(pio, true);
    segment_display_pio_init(
            pio,
            sm,
            offset,
            first_segment_gpio,
            least_significant_digit_common_gpio);

    return display_pio{pio, offset, sm};
}

static display_pio setup() {
    disable_unused_clocks();
    // TODO (optional): Slow clk_sys (CPU) as low as 1 kHz.

    setup_adc();

    //gpio_init_mask(all_digit_gpios_mask);
    //gpio_set_dir_out_masked(all_digit_gpios_mask);

    return setup_pio();
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

static uint8_t encode_digit(int_fast8_t value, bool decimal_point = false)
{
    // don't trust the caller
    value %= 10;
    uint8_t encoded = led_pins::display_pins[value];
    if (decimal_point)
        encoded |= led_pins::dp;
    // Segment pins are flipped; lit = LOW, dark = HIGH (common anode).
    return ~encoded;
}

static void clear_digits() {
    // set common & segment GPIOs LOW
    gpio_clr_mask(all_digit_gpios_mask);
}

static uint32_t encode_for_pio(float value) {
    float dekaValue = value * 10.0f;
    dekaValue = std::round(dekaValue);
    uint32_t dekaInt = static_cast<uint32_t>(dekaValue);
    // digits encoded as set of seven-segment display bits.
    uint32_t digits = 0;
    digits |= static_cast<uint32_t>(encode_digit(dekaInt / 100 % 10)) << 16;
    // For now, decimal point after 2nd digit.
    digits |= static_cast<uint32_t>(encode_digit(dekaInt / 10 % 10, true)) << 8;
    digits |= encode_digit(dekaInt % 10);
    return digits;
}

static void display(const display_pio& disp, float value)
{
    uint32_t encoded = encode_for_pio(value);
    pio_sm_put(disp.pio, disp.sm, encoded);
}

int main() {
    display_pio disp = setup();

    absolute_time_t nextWakeup = get_absolute_time();
    float temperature_celsius = get_temperature_celsius();
    constexpr auto updateIntervalMS
        = std::chrono::duration_cast<std::chrono::milliseconds>(TEMP_UPDATE_INTERVAL);

    for (;;) {
        display(disp, celsius_to_fahrenheit(temperature_celsius));
        nextWakeup = delayed_by_ms(nextWakeup, updateIntervalMS.count());
        sleep_until(nextWakeup);
        static_assert(TEMP_SAMPLES != 0);
        temperature_celsius
            = temperature_celsius * (float(TEMP_SAMPLES - 1) / float(TEMP_SAMPLES))
            + get_temperature_celsius() * (1.0f / TEMP_SAMPLES);
    }

    return 0;
}
