/* Hackflight ESP32 transmitter sketch
 * 
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, in version 3.  This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.  You should have received a copy of
 * the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <hackflight.h>
#include <firmware/blink_timer.hpp>
#include <firmware/espnow.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/serializer.hpp>
#include <firmware/voltage_divider.hpp>

// Hardware-dependent --------------------------------------------------------

static const uint8_t kReceiverAddress[6] = {
    
    // OMGS3
    0x98,0x3D,0xAE,0xEF,0x0E,0xAC
};


// Pins ----------------------------------------------------------------------
static const uint8_t kThrottlePin = A4;
static const uint8_t kRollPin = A3;
static const uint8_t kPitchPin = A2;
static const uint8_t kYawPin = A1;
static const uint8_t kVoltageDividerPin = A6;
static const uint8_t kArmingButtonPin = A8;
static const uint8_t kHoverButtonPin = A0;
static const uint8_t kAutonomousButtonPin = A7;
static const uint8_t kLedPin = 21;

// Weak-ass button debouncing ------------------------------------------------
static const uint16_t kButtonAnalogMax = 50;

// Axis extrema determined empirically --------------------------------------
static const uint16_t kThrottleMid = 1903;
static const uint16_t kRollMid = 1951;
static const uint16_t kPitchMid = 1950;
static const uint16_t kYawMid = 1900;

// For traditional (slack throttle) TX emulation -----------------------------
static const bool kIsTraditional = false;
static const int16_t kThrottleArmingMin = -1900;

// Votage divider constants to scale down for 3.3V analog-read ---------------
static const float kVoltageDividerR1Ohms = 1000;
static const float kVoltageDividerR2Ohms = 2200;

// Other constants -----------------------------------------------------------
static const float kLowVoltage = 3.0;
static const uint8_t kLedIntensity = 255;
static const float kFreqHz = 100;

// Momentary pushbutton as on/off toggle
class MomentaryButton {

    public:

        MomentaryButton(const bool state, const bool was_down)
            : state_(state), was_down_(was_down) {}

        MomentaryButton() = default;

        MomentaryButton(const MomentaryButton & other) = default;

        static auto Update(const MomentaryButton & pb,
                const uint16_t analog_value,
                const bool should_reset) -> MomentaryButton
        {
            const auto is_down = analog_value < kButtonAnalogMax;

            return MomentaryButton(

                    should_reset ? false :
                    is_down && !pb.was_down_ ? !pb.state_ :
                    pb.state_,

                    is_down);
        }

        static auto GetState(const MomentaryButton & pb) -> bool
        {
            return pb.state_;
        }

    private:

        bool state_;
        bool was_down_;
};

// Debouncer for latching pushbutton
class DebouncedButton {

    private:

        static constexpr uint32_t kDebounceDelayMsec = 250;

    public:

        DebouncedButton(
                const bool analog_value,
                const uint32_t last_debounce_msec,
                const uint8_t state) 
            : analog_value_(analog_value),
            last_debounce_msec_(last_debounce_msec),
            state_(state) {}

        DebouncedButton() = default;

        DebouncedButton(const DebouncedButton & other) = default;

        static auto Update(
                const DebouncedButton & button,
                const uint16_t analog_value,
                const uint32_t msec) -> DebouncedButton
        {
            const auto is_down = analog_value < kButtonAnalogMax;

            const auto last_debounce_msec = 
                analog_value != button.analog_value_ ?
                msec : button.last_debounce_msec_;

            const auto state =
                ((msec - button.last_debounce_msec_) >
                 kDebounceDelayMsec && analog_value != button.state_) ?
                analog_value : button.state_;

            return DebouncedButton(analog_value, last_debounce_msec, state);
        }

        static auto GetState(const DebouncedButton & button) -> uint8_t
        {
            return button.state_;
        }

    private:

        bool analog_value_;
        uint32_t last_debounce_msec_;
        uint8_t state_;
};

// Scale to [-2048, +2047]
static auto ReadAxis(const uint8_t pin, const int16_t mid) -> int16_t
{
    return analogRead(pin) - mid;
}

void setup()
{
    Serial.begin(115200);

    pinMode(kLedPin, OUTPUT);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kReceiverAddress);
}


void loop()
{
    static hf::Mode mode_;

    const auto is_idle = mode_ == hf::kModeIdle;

    static DebouncedButton arming_button_;
    arming_button_ = DebouncedButton::Update(
            arming_button_, analogRead(kArmingButtonPin), millis());

    static MomentaryButton hover_button_;
    hover_button_ = MomentaryButton::Update(
            hover_button_, analogRead(kHoverButtonPin), is_idle);

    static MomentaryButton autonomous_button_;
    autonomous_button_ = MomentaryButton::Update(
            autonomous_button_, analogRead(kAutonomousButtonPin), is_idle);

    // 12-bit voltage reading
    const auto volts = hf::VoltageDivider::Convert(
            kVoltageDividerR1Ohms, kVoltageDividerR2Ohms, 
            analogRead(kVoltageDividerPin), 12);

    static hf::BlinkTimer blink_timer_; 

    blink_timer_ = hf::BlinkTimer::Update(blink_timer_, millis());

    const auto led_state = volts < kLowVoltage ? 
        hf::BlinkTimer::IsOn(blink_timer_) : true;

    analogWrite(kLedPin, led_state ? kLedIntensity : 0);

    const auto throttle = -ReadAxis(kThrottlePin, kThrottleMid);
    const auto roll = -ReadAxis(kRollPin, kRollMid);
    const auto pitch =  ReadAxis(kPitchPin, kPitchMid);
    const auto yaw = ReadAxis(kYawPin, kYawMid);

    const auto is_arming_button_pressed =
        DebouncedButton::GetState(arming_button_);

    const auto is_throttle_down = throttle < kThrottleArmingMin;

    // Implement a little state/transition machine for mode
    mode_ = 

        !is_arming_button_pressed ?
        hf::kModeIdle :

        mode_ == hf::kModeIdle &&
        (!kIsTraditional || is_throttle_down) &&
        is_arming_button_pressed ?
        hf::kModeArmed :

        mode_ == hf::kModeArmed &&
        !is_arming_button_pressed ?
        hf::kModeIdle :

        mode_ == hf::kModeArmed &&
        !kIsTraditional && 
        MomentaryButton::GetState(hover_button_) ? 
        hf::kModeHovering :

        mode_ == hf::kModeHovering &&
        !MomentaryButton::GetState(hover_button_) ? 
        hf::kModeArmed :

        mode_ == hf::kModeHovering &&
        MomentaryButton::GetState(autonomous_button_) ? 
        hf::kModeAutonomous :

        mode_ == hf::kModeAutonomous &&
        !MomentaryButton::GetState(autonomous_button_) ? 
        hf::kModeHovering :

        mode_;

    static hf::MspSerializer serializer_;

    static const char * kModeNames[4] = {
        "idle", "armed", "hover", "autonomous"
    };

    Serial.printf("volts=%2.1f mode=%-10s " 
            "throttle=%+05d roll=%+05d pitch=%+05d yaw=%+05d\n",
            volts, kModeNames[mode_], throttle, roll, pitch, yaw);

    const short vals[5] = {throttle, roll, pitch, yaw, mode_};

    serializer_ = hf::MspSerializer::SerializeShorts(
            serializer_, kMspSetChannels, vals, 5);

    const auto result = esp_now_send(kReceiverAddress,
            hf::MspSerializer::GetPayloadBytes(serializer_),
            hf::MspSerializer::GetPayloadSize(serializer_));

    if (result != ESP_OK) {
        //Serial.printf("ERROR sending to vehicle: %d\n", result);
    }

    delay((int)1000/kFreqHz);
}
