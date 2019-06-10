//
// Created by cxn on 19-2-18.
#include "localization_math.h"

namespace leonard_localization {
    Vec3d CoordAdd(const Vec3d &a, const Vec3d &b) {
        Vec3d c;
        c(0) = b(0) + a(0) * std::cos(b(2)) - a(1) * std::sin(b(2));
        c(1) = b(1) + a(0) * std::sin(b(2)) + a(1) * std::cos(b(2));
        c(2) = b(2) + a(2);
        c(2) = std::atan2(sin(c(2)), cos(c(2)));
        return c;
    };
}