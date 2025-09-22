#pragma once

#include <cmath>
#include <fmt/format.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace LocalizationHelpers {

    inline double wrapAngle(double angle) {
        angle = std::fmod(angle + M_PI, 2 * M_PI);  // shift by +π, wrap modulo 2π
        if (angle < 0)
            angle += 2 * M_PI;                       // ensure it's positive
        return angle - M_PI;                          // shift back to [-π, π]
    }

}

template <typename T>
struct fmt::formatter<T, char, std::enable_if_t<
    std::is_base_of_v<Eigen::EigenBase<T>, T>>> {
    
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const T& mat, FormatContext& ctx) {
        std::ostringstream oss;
        oss << mat;
        return fmt::format_to(ctx.out(), "{}", oss.str());
    }
};