//
// Created by cxn on 19-2-16.
//

#ifndef PROJECT_PARTICLE_FILTER_GAUSSIAN_PDF_H
#define PROJECT_PARTICLE_FILTER_GAUSSIAN_PDF_H

#include <stdlib.h>
#include <cmath>
#include "localization_math.h"
namespace leonard_localization {

    void EigenDecomposition(const Mat3d &matrix_a,
                            Mat3d &matrix_v,
                            Vec3d &vector_d);
    void Tql2(Mat3d &matrix_v, Vec3d &vector_d, Vec3d &vector_e);
    void Tred2(Mat3d &matrix_v, Vec3d &vector_d, Vec3d &vector_e);

    /**
 * @brief Gaussian pdf class
 */
    class ParticleFilterGaussianPdf {

    public:
        /**
         * @brief Default constructor
         */
        ParticleFilterGaussianPdf() = delete;
        /**
         * @brief Create a gaussian pdf by mean and covariance
         * @param mean Mean to initialize the gaussian pdf
         * @param covariance Covariance initialize the gaussian pdf
         */
        //概率密度函数 probability density function (pdf)
        ParticleFilterGaussianPdf(const Vec3d &mean, const Mat3d &covariance);

        /**
         * @brief Generate random pose particle sample
         * @return Return the random pose particle sample
         */
        Vec3d GenerateSample();
    private:

        Vec3d mean_;
        Mat3d covariance_;

        // Decomposed covariance matrix (rotation * diagonal)
        Mat3d covariance_rotation_;
        Vec3d covariance_diagonal_;
    };

}


#endif //PROJECT_PARTICLE_FILTER_GAUSSIAN_PDF_H
