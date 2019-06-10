//
// Created by cxn on 19-2-15.
//

#ifndef PROJECT_TYPES_H
#define PROJECT_TYPES_H

#include <Eigen/Dense>

namespace decision {

// Use eigen3 as base data structure
    using Vec2d = Eigen::Vector2d;
    using Vec3d = Eigen::Vector3d;
    using Vec4d = Eigen::Vector4d;

    using Mat3d = Eigen::Matrix3d;
    using Mat2d = Eigen::Matrix2d;

    using MatX2d = Eigen::MatrixX2d;

}


#endif //PROJECT_TYPES_H
