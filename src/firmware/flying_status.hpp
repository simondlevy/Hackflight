/**
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <vector>

namespace hf {

    class FlyingStatus {

        private:

            static constexpr float kMotorIdleMax = 0.1;
            static const uint32_t kFlyingHysteresisThresholdMsec = 2000;

        public:

            FlyingStatus(const bool is_flying, const bool last_check_msec)
                : is_flying_(is_flying), last_check_msec_(last_check_msec) {}

            FlyingStatus() = default;

            FlyingStatus(const FlyingStatus &other) = default;

            static auto Update(
                    const FlyingStatus & fs,
                    const uint32_t msec,
                    const std::vector<float> motorvals) -> FlyingStatus
            {
                auto is_thrust_hover_idle = false;

                for (auto motorval : motorvals) {
                    if (motorval > kMotorIdleMax) {
                        is_thrust_hover_idle = true;
                        break;
                    }
                }

                const auto last_check_msec = is_thrust_hover_idle ? msec :
                    fs.last_check_msec_;

                const auto is_flying = last_check_msec > 0 &&
                    (msec - last_check_msec) <
                    kFlyingHysteresisThresholdMsec;

                return FlyingStatus(last_check_msec, is_flying);
            }

            static auto IsFlying(const FlyingStatus & fs) -> bool
            {
                return fs.is_flying_;
            }

        private:

            bool is_flying_;
            uint32_t last_check_msec_;
    };

}
