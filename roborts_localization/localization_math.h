//
// Created by cxn on 19-2-18.
//

#ifndef PROJECT_LOCALIZATION_MATH_H
#define PROJECT_LOCALIZATION_MATH_H


//


#include <math.h>
#include "types.h"
namespace leonard_localization {
    template<typename T>
    static T normalize(T z) {
        return atan2(sin(z), cos(z));
    }

    template<typename T>
    static T angle_diff(double a, double b) {
        T d1, d2;
        a = normalize(a);
        b = normalize(b);
        d1 = a - b;
        d2 = 2 * M_PI - fabs(d1);
        if (d1 > 0)
            d2 *= -1.0;
        if (fabs(d1) < fabs(d2))
            return (d1);
        else
            return (d2);
    }

    //这是一个由 Marsaglia 首创 Knuth 推荐的生成高斯分布随机数的方法，虽然看起来怪怪的
    template<typename T>
    T RandomGaussianNumByStdDev(T sigma) {
        T x1, x2, w, r;
        do {
            do {
                r = drand48();
            } while (r == 0.0);
            x1 = 2.0 * r - 1.0;
            do {
                r = drand48();
            } while (r == 0.0);
            x2 = 2.0 * r - 1.0;
            w = x1 * x1 + x2 * x2;
        } while (w > 1.0 || w == 0.0);
        return (sigma * x2 * std::sqrt(-2.0 * std::log(w) / w));
    };

    /**
     * @brief Add coordinates of two pose vectors.
     * @param a Pose vector a (x, y , yaw)
     * @param b Pose vector a (x, y , yaw)
     * @return Returns the result pose.
     */
    Vec3d CoordAdd(const Vec3d &a, const Vec3d &b);
}
#endif //PROJECT_LOCALIZATION_MATH_H
