#define TEST_NEOPIXEL_CONTROLLER_HPP
#include <gtest/gtest.h>
#include "NeopixelController.hpp"


struct FakeNeopixelStrip
{
    static constexpr uint16_t NEOPIXEL_COUNT = 16;

    std::array<uint32_t, NEOPIXEL_COUNT> pixel_colors {};
    uint8_t brightness = 0;
    int show_count = 0;

    void setPixelColor(uint16_t id, uint32_t color) { pixel_colors.at(id) = color; }  // .at() fails the test on a bad id
    void setBrightness(uint8_t value) { brightness = value; }
    void show() { ++show_count; }
};

class NeopixelControllerTest : public ::testing::Test
{
protected:

    // Declared before the controller so it exists when the delegates are bound to it
    FakeNeopixelStrip _strip;

    NeopixelController _controller {
        etl::delegate<void(uint16_t, uint32_t)>::create<FakeNeopixelStrip, &FakeNeopixelStrip::setPixelColor>(_strip),
        etl::delegate<void(uint8_t)>::create<FakeNeopixelStrip, &FakeNeopixelStrip::setBrightness>(_strip),
        etl::delegate<void()>::create<FakeNeopixelStrip, &FakeNeopixelStrip::show>(_strip),
        NeopixelControllerInputs_s {},  // all false / zero; not used by the tests below
        FakeNeopixelStrip::NEOPIXEL_COUNT
    };

    uint32_t pixelColor(LED_ID_e led) const { return _strip.pixel_colors.at(led); }
};

static uint32_t to_raw(LED_color_e color)
{
    return static_cast<uint32_t>(color);
}

/* ---------- init() ---------- */

TEST_F(NeopixelControllerTest, init_sets_default_brightness)
{
    _controller.init();
    EXPECT_EQ(_strip.brightness, 64);
}

TEST_F(NeopixelControllerTest, init_turns_bms_and_imd_leds_off)
{
    _controller.init();

    EXPECT_EQ(pixelColor(LED_ID_e::BMS), to_raw(LED_color_e::OFF));
    EXPECT_EQ(pixelColor(LED_ID_e::IMD), to_raw(LED_color_e::OFF));
    EXPECT_EQ(pixelColor(LED_ID_e::BMS_WING), to_raw(LED_color_e::OFF));
    EXPECT_EQ(pixelColor(LED_ID_e::IMD_WING), to_raw(LED_color_e::OFF));
}

TEST_F(NeopixelControllerTest, init_sets_all_other_leds_to_init_color)
{
    _controller.init();

    for (uint16_t id = 0; id < FakeNeopixelStrip::NEOPIXEL_COUNT; ++id)
    {
        const bool starts_off = (id == LED_ID_e::BMS) || (id == LED_ID_e::IMD)
                             || (id == LED_ID_e::BMS_WING) || (id == LED_ID_e::IMD_WING);
        if (!starts_off)
        {
            EXPECT_EQ(_strip.pixel_colors.at(id), to_raw(LED_color_e::INIT_COLOR)) << "LED id " << id;
        }
    }
}

TEST_F(NeopixelControllerTest, init_shows_the_strip_once)
{
    _controller.init();
    EXPECT_EQ(_strip.show_count, 1);
}

/* ---------- setNeopixel() / setNeopixelColor() ---------- */

TEST_F(NeopixelControllerTest, set_neopixel_color_writes_buffer_without_showing)
{
    _controller.setNeopixelColor(LED_ID_e::BRAKE, LED_color_e::RED);

    EXPECT_EQ(pixelColor(LED_ID_e::BRAKE), to_raw(LED_color_e::RED));
    EXPECT_EQ(_strip.show_count, 0);
}

TEST_F(NeopixelControllerTest, set_neopixel_writes_raw_color)
{
    _controller.setNeopixel(3, 0x12345678);
    EXPECT_EQ(_strip.pixel_colors.at(3), 0x12345678U);
}

/* ---------- dimNeopixels() ---------- */

TEST_F(NeopixelControllerTest, dim_from_default_wraps_to_max_then_steps_down)
{
    // Default 64 - 63 = 1, which is below the minimum of 3, so it wraps to full brightness
    _controller.dimNeopixels();
    EXPECT_EQ(_strip.brightness, 255);

    _controller.dimNeopixels();
    EXPECT_EQ(_strip.brightness, 192);
}

TEST_F(NeopixelControllerTest, dim_cycles_through_levels_and_wraps_after_min)
{
    _controller.dimNeopixels();  // 64 -> 255

    const uint8_t expected_levels[] = {192, 129, 66, 3, 255};
    for (const uint8_t expected : expected_levels)
    {
        _controller.dimNeopixels();
        EXPECT_EQ(_strip.brightness, expected);
    }
    // NOTE: fails with the current dimNeopixels(): after 3 it underflows to 196 instead of wrapping to 255.
}
