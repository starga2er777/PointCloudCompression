// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "../precomp.hpp"
#include "../usac.hpp"
#include "../ptcloud/ptcloud_wrapper.hpp"

namespace cv { namespace usac {
class HomographyEstimatorImpl : public HomographyEstimator {
private:
    const Ptr<MinimalSolver> min_solver;
    const Ptr<NonMinimalSolver> non_min_solver;
    const Ptr<Degeneracy> degeneracy;
public:
    HomographyEstimatorImpl (const Ptr<MinimalSolver> &min_solver_,
            const Ptr<NonMinimalSolver> &non_min_solver_, const Ptr<Degeneracy> &degeneracy_) :
            min_solver (min_solver_), non_min_solver (non_min_solver_), degeneracy (degeneracy_) {}

    inline int estimateModels (const std::vector<int> &sample, std::vector<Mat> &models) const override {
        if (! degeneracy->isSampleGood(sample)) return 0;
        return min_solver->estimate (sample, models);
    }
    int estimateModelNonMinimalSample(const std::vector<int> &sample, int sample_size,
            std::vector<Mat> &models, const std::vector<double> &weights) const override {
        return non_min_solver->estimate (sample, sample_size, models, weights);
    };
    int getMaxNumSolutions () const override {
        return min_solver->getMaxNumberOfSolutions();
    }
    int getMaxNumSolutionsNonMinimal () const override {
        return non_min_solver->getMaxNumberOfSolutions();
    }
    int getMinimalSampleSize () const override {
        return min_solver->getSampleSize();
    }
    int getNonMinimalSampleSize () const override {
        return non_min_solver->getMinimumRequiredSampleSize();
    }
    Ptr<Estimator> clone() const override {
        return makePtr<HomographyEstimatorImpl>(min_solver->clone(), non_min_solver->clone(),
                degeneracy->clone(0 /*we don't need state here*/));
    }
};
Ptr<HomographyEstimator> HomographyEstimator::create (const Ptr<MinimalSolver> &min_solver_,
        const Ptr<NonMinimalSolver> &non_min_solver_, const Ptr<Degeneracy> &degeneracy_) {
    return makePtr<HomographyEstimatorImpl>(min_solver_, non_min_solver_, degeneracy_);
}

/////////////////////////////////////////////////////////////////////////
class FundamentalEstimatorImpl : public FundamentalEstimator {
private:
    const Ptr<MinimalSolver> min_solver;
    const Ptr<NonMinimalSolver> non_min_solver;
    const Ptr<Degeneracy> degeneracy;
public:
    FundamentalEstimatorImpl (const Ptr<MinimalSolver> &min_solver_,
            const Ptr<NonMinimalSolver> &non_min_solver_, const Ptr<Degeneracy> &degeneracy_) :
         min_solver (min_solver_), non_min_solver (non_min_solver_), degeneracy (degeneracy_) {}

    inline int
    estimateModels(const std::vector<int> &sample, std::vector<Mat> &models) const override {
        std::vector<Mat> F;
        const int models_count = min_solver->estimate(sample, F);
        int valid_models_count = 0;
        for (int i = 0; i < models_count; i++)
            if (degeneracy->isModelValid(F[i], sample))
                models[valid_models_count++] = F[i];
        return valid_models_count;
    }
    int estimateModelNonMinimalSample(const std::vector<int> &sample, int sample_size,
            std::vector<Mat> &models, const std::vector<double> &weights) const override {
        return non_min_solver->estimate(sample, sample_size, models, weights);
    }
    int getMaxNumSolutions () const override {
        return min_solver->getMaxNumberOfSolutions();
    }
    int getMinimalSampleSize () const override {
        return min_solver->getSampleSize();
    }
    int getNonMinimalSampleSize () const override {
        return non_min_solver->getMinimumRequiredSampleSize();
    }
    int getMaxNumSolutionsNonMinimal () const override {
        return non_min_solver->getMaxNumberOfSolutions();
    }
    Ptr<Estimator> clone() const override {
        return makePtr<FundamentalEstimatorImpl>(min_solver->clone(), non_min_solver->clone(),
                degeneracy->clone(0));
    }
};
Ptr<FundamentalEstimator> FundamentalEstimator::create (const Ptr<MinimalSolver> &min_solver_,
        const Ptr<NonMinimalSolver> &non_min_solver_, const Ptr<Degeneracy> &degeneracy_) {
    return makePtr<FundamentalEstimatorImpl>(min_solver_, non_min_solver_, degeneracy_);
}

/////////////////////////////////////////////////////////////////////////
class EssentialEstimatorImpl : public EssentialEstimator {
private:
    const Ptr<MinimalSolver> min_solver;
    const Ptr<NonMinimalSolver> non_min_solver;
    const Ptr<Degeneracy> degeneracy;
public:
    explicit EssentialEstimatorImpl (const Ptr<MinimalSolver> &min_solver_,
        const Ptr<NonMinimalSolver> &non_min_solver_, const Ptr<Degeneracy> &degeneracy_) :
        min_solver (min_solver_), non_min_solver (non_min_solver_), degeneracy (degeneracy_) {}

    inline int
    estimateModels(const std::vector<int> &sample, std::vector<Mat> &models) const override {
            std::vector<Mat> E;
        const int models_count = min_solver->estimate(sample, E);
        int valid_models_count = 0;
        for (int i = 0; i < models_count; i++)
            if (degeneracy->isModelValid (E[i], sample))
                models[valid_models_count++] = E[i];
        return valid_models_count;
    }

    int estimateModelNonMinimalSample(const std::vector<int> &sample, int sample_size,
            std::vector<Mat> &models, const std::vector<double> &weights) const override {
        return non_min_solver->estimate(sample, sample_size, models, weights);
    };
    int getMaxNumSolutions () const override {
        return min_solver->getMaxNumberOfSolutions();
    }
    int getMinimalSampleSize () const override {
        return min_solver->getSampleSize();
    }
    int getNonMinimalSampleSize () const override {
        return non_min_solver->getMinimumRequiredSampleSize();
    }
    int getMaxNumSolutionsNonMinimal () const override {
        return non_min_solver->getMaxNumberOfSolutions();
    }
    Ptr<Estimator> clone() const override {
        return makePtr<EssentialEstimatorImpl>(min_solver->clone(), non_min_solver->clone(),
                degeneracy->clone(0));
    }
};
Ptr<EssentialEstimator> EssentialEstimator::create (const Ptr<MinimalSolver> &min_solver_,
        const Ptr<NonMinimalSolver> &non_min_solver_, const Ptr<Degeneracy> &degeneracy_) {
    return makePtr<EssentialEstimatorImpl>(min_solver_, non_min_solver_, degeneracy_);
}

/////////////////////////////////////////////////////////////////////////
class AffineEstimatorImpl : public AffineEstimator {
private:
    const Ptr<MinimalSolver> min_solver;
    const Ptr<NonMinimalSolver> non_min_solver;
public:
    explicit AffineEstimatorImpl (const Ptr<MinimalSolver> &min_solver_,
        const Ptr<NonMinimalSolver> &non_min_solver_) :
        min_solver (min_solver_), non_min_solver (non_min_solver_) {}

    int estimateModels(const std::vector<int> &sample, std::vector<Mat> &models) const override {
        return min_solver->estimate(sample, models);
    }
    int estimateModelNonMinimalSample (const std::vector<int> &sample, int sample_size,
            std::vector<Mat> &models, const std::vector<double> &weights) const override {
        return non_min_solver->estimate(sample, sample_size, models, weights);
    }
    int getMinimalSampleSize() const override {
        return min_solver->getSampleSize(); // 3 points required
    }
    int getNonMinimalSampleSize() const override {
        return non_min_solver->getMinimumRequiredSampleSize();
    }
    int getMaxNumSolutions () const override {
        return min_solver->getMaxNumberOfSolutions();
    }
    int getMaxNumSolutionsNonMinimal () const override {
        return non_min_solver->getMaxNumberOfSolutions();
    }
    Ptr<Estimator> clone() const override {
        return makePtr<AffineEstimatorImpl>(min_solver->clone(), non_min_solver->clone());
    }
};
Ptr<AffineEstimator> AffineEstimator::create (const Ptr<MinimalSolver> &min_solver_,
        const Ptr<NonMinimalSolver> &non_min_solver_) {
    return makePtr<AffineEstimatorImpl>(min_solver_, non_min_solver_);
}

/////////////////////////////////////////////////////////////////////////
class PnPEstimatorImpl : public PnPEstimator {
private:
    const Ptr<MinimalSolver> min_solver;
    const Ptr<NonMinimalSolver> non_min_solver;
public:
    explicit PnPEstimatorImpl (const Ptr<MinimalSolver> &min_solver_,
        const Ptr<NonMinimalSolver> &non_min_solver_) :
        min_solver(min_solver_), non_min_solver(non_min_solver_) {}

    int estimateModels (const std::vector<int> &sample, std::vector<Mat> &models) const override {
        return min_solver->estimate(sample, models);
    }
    int estimateModelNonMinimalSample (const std::vector<int> &sample, int sample_size,
            std::vector<Mat> &models, const std::vector<double> &weights) const override {
        return non_min_solver->estimate(sample, sample_size, models, weights);
    }
    int getMinimalSampleSize() const override {
        return min_solver->getSampleSize();
    }
    int getNonMinimalSampleSize() const override {
        return non_min_solver->getMinimumRequiredSampleSize();
    }
    int getMaxNumSolutions () const override {
        return min_solver->getMaxNumberOfSolutions();
    }
    int getMaxNumSolutionsNonMinimal () const override {
        return non_min_solver->getMaxNumberOfSolutions();
    }
    Ptr<Estimator> clone() const override {
        return makePtr<PnPEstimatorImpl>(min_solver->clone(), non_min_solver->clone());
    }
};
Ptr<PnPEstimator> PnPEstimator::create (const Ptr<MinimalSolver> &min_solver_,
        const Ptr<NonMinimalSolver> &non_min_solver_) {
    return makePtr<PnPEstimatorImpl>(min_solver_, non_min_solver_);
}

/////////////////////////////////////////////////////////////////////////
class PointCloudModelEstimatorImpl : public PointCloudModelEstimator {
private:
    const Ptr<MinimalSolver> min_solver;
    const Ptr<NonMinimalSolver> non_min_solver;
    const ModelConstraintFunction custom_model_constraints;

    inline int filteringModel(std::vector<Mat> M, int models_count,
            std::vector<Mat> &valid_models) const {
        int valid_models_count = 0;
        if (!custom_model_constraints) {
            valid_models_count = models_count;
            for (int i = 0; i < models_count; ++i)
                valid_models[i] = M[i];
        } else {
            for (int i = 0; i < models_count; ++i)
                // filtering Models with custom_model_constraints
                if (custom_model_constraints(M[i]))
                    valid_models[valid_models_count++] = M[i];
        }

        return valid_models_count;
    }

public:
    explicit PointCloudModelEstimatorImpl (const Ptr<MinimalSolver> &min_solver_,
            const Ptr<NonMinimalSolver> &non_min_solver_,
            ModelConstraintFunction custom_model_constraints_) :
            min_solver(min_solver_), non_min_solver(non_min_solver_),
            custom_model_constraints(std::move(custom_model_constraints_)) {}

    int estimateModels (const std::vector<int> &sample, std::vector<Mat> &models) const override {
        std::vector<Mat> M;
        const int models_count = min_solver->estimate(sample, M);
        return filteringModel(M, models_count, models);
    }

    int estimateModelNonMinimalSample (const std::vector<int> &sample, int sample_size,
            std::vector<Mat> &models, const std::vector<double> &weights) const override {
        std::vector<Mat> M;
        const int models_count = non_min_solver->estimate(sample, sample_size, M, weights);
        return filteringModel(M, models_count, models);
    }

    int getMinimalSampleSize() const override {
        return min_solver->getSampleSize();
    }
    int getNonMinimalSampleSize() const override {
        return non_min_solver->getMinimumRequiredSampleSize();
    }
    int getMaxNumSolutions () const override {
        return min_solver->getMaxNumberOfSolutions();
    }
    int getMaxNumSolutionsNonMinimal () const override {
        return non_min_solver->getMaxNumberOfSolutions();
    }
    Ptr<Estimator> clone() const override {
        return makePtr<PointCloudModelEstimatorImpl>(min_solver->clone(), non_min_solver->clone(),
                                                     custom_model_constraints);
    }
};
Ptr<PointCloudModelEstimator> PointCloudModelEstimator::create (const Ptr<MinimalSolver> &min_solver_,
        const Ptr<NonMinimalSolver> &non_min_solver_,
        const ModelConstraintFunction &custom_model_constraints_) {
    return makePtr<PointCloudModelEstimatorImpl>(min_solver_, non_min_solver_, custom_model_constraints_);
}

///////////////////////////////////////////// ERROR /////////////////////////////////////////
// Symmetric Reprojection Error
class ReprojectionErrorSymmetricImpl : public ReprojectionErrorSymmetric {
private:
    const Mat * points_mat;
    const float * const points;
    float m11, m12, m13, m21, m22, m23, m31, m32, m33;
    float minv11, minv12, minv13, minv21, minv22, minv23, minv31, minv32, minv33;
    std::vector<float> errors;
public:
    explicit ReprojectionErrorSymmetricImpl (const Mat &points_)
        : points_mat(&points_), points ((float *) points_.data)
        , m11(0), m12(0), m13(0), m21(0), m22(0), m23(0), m31(0), m32(0), m33(0)
        , minv11(0), minv12(0), minv13(0), minv21(0), minv22(0), minv23(0), minv31(0), minv32(0), minv33(0)
        , errors(points_.rows)
    {
        CV_DbgAssert(points);
    }

    inline void setModelParameters(const Mat& model) override
    {
        CV_Assert(!model.empty());
        CV_CheckTypeEQ(model.depth(), CV_64F, "");

        const auto * const m = (double *) model.data;
        m11=static_cast<float>(m[0]); m12=static_cast<float>(m[1]); m13=static_cast<float>(m[2]);
        m21=static_cast<float>(m[3]); m22=static_cast<float>(m[4]); m23=static_cast<float>(m[5]);
        m31=static_cast<float>(m[6]); m32=static_cast<float>(m[7]); m33=static_cast<float>(m[8]);

        const Mat model_inv = model.inv();
        CV_CheckTypeEQ(model_inv.depth(), CV_64F, "");
        const auto * const minv = (double *) model_inv.data;
        minv11=(float)minv[0]; minv12=(float)minv[1]; minv13=(float)minv[2];
        minv21=(float)minv[3]; minv22=(float)minv[4]; minv23=(float)minv[5];
        minv31=(float)minv[6]; minv32=(float)minv[7]; minv33=(float)minv[8];
    }
    inline float getError (int point_idx) const override {
        const int smpl = 4*point_idx;
        const float x1=points[smpl], y1=points[smpl+1], x2=points[smpl+2], y2=points[smpl+3];
        const float est_z2 = 1 / (m31 * x1 + m32 * y1 + m33),
                    dx2 =  x2 -  (m11 * x1 + m12 * y1 + m13) * est_z2,
                    dy2 =  y2 -  (m21 * x1 + m22 * y1 + m23) * est_z2;
        const float est_z1 = 1 / (minv31 * x2 + minv32 * y2 + minv33),
                    dx1 =  x1 -  (minv11 * x2 + minv12 * y2 + minv13) * est_z1,
                    dy1 =  y1 -  (minv21 * x2 + minv22 * y2 + minv23) * est_z1;
        return (dx2 * dx2 + dy2 * dy2 + dx1 * dx1 + dy1 * dy1) * .5f;
    }
    const std::vector<float> &getErrors (const Mat &model) override {
        setModelParameters(model);
        for (int point_idx = 0; point_idx < points_mat->rows; point_idx++) {
            const int smpl = 4*point_idx;
            const float x1=points[smpl], y1=points[smpl+1], x2=points[smpl+2], y2=points[smpl+3];
            const float est_z2 = 1 / (m31 * x1 + m32 * y1 + m33),
                    dx2 =  x2 -  (m11 * x1 + m12 * y1 + m13) * est_z2,
                    dy2 =  y2 -  (m21 * x1 + m22 * y1 + m23) * est_z2;
            const float est_z1 = 1 / (minv31 * x2 + minv32 * y2 + minv33),
                    dx1 =  x1 -  (minv11 * x2 + minv12 * y2 + minv13) * est_z1,
                    dy1 =  y1 -  (minv21 * x2 + minv22 * y2 + minv23) * est_z1;
            errors[point_idx] = (dx2 * dx2 + dy2 * dy2 + dx1 * dx1 + dy1 * dy1) * .5f;
        }
        return errors;
    }
    Ptr<Error> clone () const override {
        return makePtr<ReprojectionErrorSymmetricImpl>(*points_mat);
    }
};
Ptr<ReprojectionErrorSymmetric>
ReprojectionErrorSymmetric::create(const Mat &points) {
    return makePtr<ReprojectionErrorSymmetricImpl>(points);
}

// Forward Reprojection Error
class ReprojectionErrorForwardImpl : public ReprojectionErrorForward {
private:
    const Mat * points_mat;
    const float * const points;
    float m11, m12, m13, m21, m22, m23, m31, m32, m33;
    std::vector<float> errors;
public:
    explicit ReprojectionErrorForwardImpl (const Mat &points_)
        : points_mat(&points_), points ((float *)points_.data)
        , m11(0), m12(0), m13(0), m21(0), m22(0), m23(0), m31(0), m32(0), m33(0)
        , errors(points_.rows)
    {
        CV_DbgAssert(points);
    }

    inline void setModelParameters(const Mat& model) override
    {
        CV_Assert(!model.empty());
        CV_CheckTypeEQ(model.depth(), CV_64F, "");

        const auto * const m = (double *) model.data;
        m11=static_cast<float>(m[0]); m12=static_cast<float>(m[1]); m13=static_cast<float>(m[2]);
        m21=static_cast<float>(m[3]); m22=static_cast<float>(m[4]); m23=static_cast<float>(m[5]);
        m31=static_cast<float>(m[6]); m32=static_cast<float>(m[7]); m33=static_cast<float>(m[8]);
    }
    inline float getError (int point_idx) const override {
        const int smpl = 4*point_idx;
        const float x1 = points[smpl], y1 = points[smpl+1], x2 = points[smpl+2], y2 = points[smpl+3];
        const float est_z2 = 1 / (m31 * x1 + m32 * y1 + m33),
                    dx2 =  x2 -  (m11 * x1 + m12 * y1 + m13) * est_z2,
                    dy2 =  y2 -  (m21 * x1 + m22 * y1 + m23) * est_z2;
        return dx2 * dx2 + dy2 * dy2;
    }
    const std::vector<float> &getErrors (const Mat &model) override {
        setModelParameters(model);
        for (int point_idx = 0; point_idx < points_mat->rows; point_idx++) {
            const int smpl = 4*point_idx;
            const float x1=points[smpl], y1=points[smpl+1], x2=points[smpl+2], y2=points[smpl+3];
            const float est_z2 = 1 / (m31 * x1 + m32 * y1 + m33),
                        dx2 =  x2 -  (m11 * x1 + m12 * y1 + m13) * est_z2,
                        dy2 =  y2 -  (m21 * x1 + m22 * y1 + m23) * est_z2;
            errors[point_idx] = dx2 * dx2 + dy2 * dy2;
        }
        return errors;
    }
    Ptr<Error> clone () const override {
        return makePtr<ReprojectionErrorForwardImpl>(*points_mat);
    }
};
Ptr<ReprojectionErrorForward>
ReprojectionErrorForward::create(const Mat &points) {
    return makePtr<ReprojectionErrorForwardImpl>(points);
}

class SampsonErrorImpl : public SampsonError {
private:
    const Mat * points_mat;
    const float * const points;
    float m11, m12, m13, m21, m22, m23, m31, m32, m33;
    std::vector<float> errors;
public:
    explicit SampsonErrorImpl (const Mat &points_)
        : points_mat(&points_), points ((float *) points_.data)
        , m11(0), m12(0), m13(0), m21(0), m22(0), m23(0), m31(0), m32(0), m33(0)
        , errors(points_.rows)
    {
        CV_DbgAssert(points);
    }

    inline void setModelParameters(const Mat& model) override
    {
        CV_Assert(!model.empty());
        CV_CheckTypeEQ(model.depth(), CV_64F, "");

        const auto * const m = (double *) model.data;
        m11=static_cast<float>(m[0]); m12=static_cast<float>(m[1]); m13=static_cast<float>(m[2]);
        m21=static_cast<float>(m[3]); m22=static_cast<float>(m[4]); m23=static_cast<float>(m[5]);
        m31=static_cast<float>(m[6]); m32=static_cast<float>(m[7]); m33=static_cast<float>(m[8]);
    }

    /*
     *                                       (pt2^t * F * pt1)^2)
     * Sampson error = ------------------------------------------------------------------------
     *                  (((F⋅pt1)(0))^2 + ((F⋅pt1)(1))^2 + ((F^t⋅pt2)(0))^2 + ((F^t⋅pt2)(1))^2)
     *
     * [ x2 y2 1 ] * [ F(1,1)  F(1,2)  F(1,3) ]   [ x1 ]
     *               [ F(2,1)  F(2,2)  F(2,3) ] * [ y1 ]
     *               [ F(3,1)  F(3,2)  F(3,3) ]   [ 1  ]
     *
     */
    inline float getError (int point_idx) const override {
        const int smpl = 4*point_idx;
        const float x1=points[smpl], y1=points[smpl+1], x2=points[smpl+2], y2=points[smpl+3];
        const float F_pt1_x = m11 * x1 + m12 * y1 + m13,
                    F_pt1_y = m21 * x1 + m22 * y1 + m23;
        const float pt2_F_x = x2 * m11 + y2 * m21 + m31,
                    pt2_F_y = x2 * m12 + y2 * m22 + m32;
        const float pt2_F_pt1 = x2 * F_pt1_x + y2 * F_pt1_y + m31 * x1 + m32 * y1 + m33;
        return pt2_F_pt1 * pt2_F_pt1 / (F_pt1_x * F_pt1_x + F_pt1_y * F_pt1_y +
                                        pt2_F_x * pt2_F_x + pt2_F_y * pt2_F_y);
    }
    const std::vector<float> &getErrors (const Mat &model) override {
        setModelParameters(model);
        for (int point_idx = 0; point_idx < points_mat->rows; point_idx++) {
            const int smpl = 4*point_idx;
            const float x1=points[smpl], y1=points[smpl+1], x2=points[smpl+2], y2=points[smpl+3];
            const float F_pt1_x = m11 * x1 + m12 * y1 + m13,
                        F_pt1_y = m21 * x1 + m22 * y1 + m23;
            const float pt2_F_x = x2 * m11 + y2 * m21 + m31,
                        pt2_F_y = x2 * m12 + y2 * m22 + m32;
            const float pt2_F_pt1 = x2 * F_pt1_x + y2 * F_pt1_y + m31 * x1 + m32 * y1 + m33;
            errors[point_idx] = pt2_F_pt1 * pt2_F_pt1 / (F_pt1_x * F_pt1_x + F_pt1_y * F_pt1_y +
                                                         pt2_F_x * pt2_F_x + pt2_F_y * pt2_F_y);
        }
        return errors;
    }
    Ptr<Error> clone () const override {
        return makePtr<SampsonErrorImpl>(*points_mat);
    }
};
Ptr<SampsonError>
SampsonError::create(const Mat &points) {
    return makePtr<SampsonErrorImpl>(points);
}

class SymmetricGeometricDistanceImpl : public SymmetricGeometricDistance {
private:
    const Mat * points_mat;
    const float * const points;
    float m11, m12, m13, m21, m22, m23, m31, m32, m33;
    std::vector<float> errors;
public:
    explicit SymmetricGeometricDistanceImpl (const Mat &points_)
        : points_mat(&points_), points ((float *) points_.data)
        , m11(0), m12(0), m13(0), m21(0), m22(0), m23(0), m31(0), m32(0), m33(0)
        , errors(points_.rows)
    {
        CV_DbgAssert(points);
    }

    inline void setModelParameters(const Mat& model) override
    {
        CV_Assert(!model.empty());
        CV_CheckTypeEQ(model.depth(), CV_64F, "");

        const auto * const m = (double *) model.data;
        m11=static_cast<float>(m[0]); m12=static_cast<float>(m[1]); m13=static_cast<float>(m[2]);
        m21=static_cast<float>(m[3]); m22=static_cast<float>(m[4]); m23=static_cast<float>(m[5]);
        m31=static_cast<float>(m[6]); m32=static_cast<float>(m[7]); m33=static_cast<float>(m[8]);
    }

    inline float getError (int point_idx) const override {
        const int smpl = 4*point_idx;
        const float x1=points[smpl], y1=points[smpl+1], x2=points[smpl+2], y2=points[smpl+3];
        // pt2^T * E, line 1 = [l1 l2]
        const float l1 = x2 * m11 + y2 * m21 + m31,
                    l2 = x2 * m12 + y2 * m22 + m32;
        // E * pt1, line 2 = [t1 t2]
        const float t1 = m11 * x1 + m12 * y1 + m13,
                    t2 = m21 * x1 + m22 * y1 + m23;
        float p2Ep1 = l1 * x1 + l2 * y1 + x2 * m13 + y2 * m23 + m33;
        p2Ep1 *= p2Ep1;
        return p2Ep1 / (l1 * l1 + l2 * l2)  // distance from pt1 to line 1
                +
               p2Ep1 / (t1 * t1 + t2 * t2); // distance from pt2 to line 2
    }
    const std::vector<float> &getErrors (const Mat &model) override {
        setModelParameters(model);
        for (int point_idx = 0; point_idx < points_mat->rows; point_idx++) {
            const int smpl = 4*point_idx;
            const float x1=points[smpl], y1=points[smpl+1], x2=points[smpl+2], y2=points[smpl+3];
            const float l1 = x2 * m11 + y2 * m21 + m31, t1 = m11 * x1 + m12 * y1 + m13,
                        l2 = x2 * m12 + y2 * m22 + m32, t2 = m21 * x1 + m22 * y1 + m23;
            float p2Ep1 = l1 * x1 + l2 * y1 + x2 * m13 + y2 * m23 + m33;
            p2Ep1 *= p2Ep1;
            errors[point_idx] = p2Ep1 / (l1 * l1 + l2 * l2) + p2Ep1 / (t1 * t1 + t2 * t2);
        }
        return errors;
    }
    Ptr<Error> clone () const override {
        return makePtr<SymmetricGeometricDistanceImpl>(*points_mat);
    }
};
Ptr<SymmetricGeometricDistance>
SymmetricGeometricDistance::create(const Mat &points) {
    return makePtr<SymmetricGeometricDistanceImpl>(points);
}

class ReprojectionErrorPmatrixImpl : public ReprojectionErrorPmatrix {
private:
    const Mat * points_mat;
    const float * const points;
    float p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34;
    std::vector<float> errors;
public:
    explicit ReprojectionErrorPmatrixImpl (const Mat &points_)
        : points_mat(&points_), points ((float *) points_.data)
        , p11(0), p12(0), p13(0), p14(0), p21(0), p22(0), p23(0), p24(0), p31(0), p32(0), p33(0), p34(0)
        , errors(points_.rows)
    {
        CV_DbgAssert(points);
    }


    inline void setModelParameters (const Mat& model) override
    {
        CV_Assert(!model.empty());
        CV_CheckTypeEQ(model.depth(), CV_64F, "");

        const auto * const p = (double *) model.data;
        p11 = (float)p[0]; p12 = (float)p[1]; p13 = (float)p[2];  p14 = (float)p[3];
        p21 = (float)p[4]; p22 = (float)p[5]; p23 = (float)p[6];  p24 = (float)p[7];
        p31 = (float)p[8]; p32 = (float)p[9]; p33 = (float)p[10]; p34 = (float)p[11];
    }

    inline float getError (int point_idx) const override {
        const int smpl = 5*point_idx;
        const float u = points[smpl  ], v = points[smpl+1],
                    x = points[smpl+2], y = points[smpl+3], z = points[smpl+4];
        const float depth = 1 / (p31 * x + p32 * y + p33 * z + p34);
        const float du =    u - (p11 * x + p12 * y + p13 * z + p14) * depth;
        const float dv =    v - (p21 * x + p22 * y + p23 * z + p24) * depth;
        return du * du + dv * dv;
    }
    const std::vector<float> &getErrors (const Mat &model) override {
        setModelParameters(model);
        for (int point_idx = 0; point_idx < points_mat->rows; point_idx++) {
            const int smpl = 5*point_idx;
            const float u = points[smpl  ], v = points[smpl+1],
                        x = points[smpl+2], y = points[smpl+3], z = points[smpl+4];
            const float depth = 1 / (p31 * x + p32 * y + p33 * z + p34);
            const float du =    u - (p11 * x + p12 * y + p13 * z + p14) * depth;
            const float dv =    v - (p21 * x + p22 * y + p23 * z + p24) * depth;
            errors[point_idx] = du * du + dv * dv;
        }
        return errors;
    }
    Ptr<Error> clone () const override {
        return makePtr<ReprojectionErrorPmatrixImpl>(*points_mat);
    }
};
Ptr<ReprojectionErrorPmatrix> ReprojectionErrorPmatrix::create(const Mat &points) {
    return makePtr<ReprojectionErrorPmatrixImpl>(points);
}

class PlaneModelErrorImpl : public PlaneModelError, public PointCloudWrapper {
private:
    //! ax + by + cz + d = 0
    float a, b, c ,d;
    std::vector<float> errors_cache;
    bool cache_valid;

public:
    explicit PlaneModelErrorImpl(const Mat &points_)
    : PointCloudWrapper(points_),
    a(0), b(0), c(0), d(0), errors_cache(pts_cnt), cache_valid(false)
    {
    }

    inline void setModelParameters(const Mat &model) override
    {
        CV_Assert(!model.empty());
        CV_CheckTypeEQ(model.depth(), CV_64F, "Model parameter type should use double");

        const double * const p = (double *) model.data;
        double coeff_a = p[0], coeff_b = p[1], coeff_c = p[2], coeff_d = p[3];
        // Format for easy distance calculation
        double magnitude_abc = sqrt(coeff_a * coeff_a + coeff_b * coeff_b + coeff_c * coeff_c);
        if (0 != magnitude_abc) {
            double inv_magnitude_abc = 1.0 / magnitude_abc;
            coeff_a = coeff_a * inv_magnitude_abc;
            coeff_b = coeff_b * inv_magnitude_abc;
            coeff_c = coeff_c * inv_magnitude_abc;
            coeff_d = coeff_d * inv_magnitude_abc;
        }
        cache_valid = cache_valid && a == (float) coeff_a && b == (float) coeff_b
                &&  c == (float) coeff_c && d == (float) coeff_d;

        a = (float) coeff_a;
        b = (float) coeff_b;
        c = (float) coeff_c;
        d = (float) coeff_d;
    }

    inline float getError(int point_idx) const override
    {
        float error = a * pts_ptr_x[point_idx] + b * pts_ptr_y[point_idx]
                      + c * pts_ptr_z[point_idx] + d;
        return error * error;
    }

    const std::vector<float> &getErrors(const Mat &model) override
    {
        setModelParameters(model);

        if (cache_valid)
            return errors_cache;

        int i = 0;
#ifdef CV_SIMD
        v_float32 v_a = vx_setall_f32(a);
        v_float32 v_b = vx_setall_f32(b);
        v_float32 v_c = vx_setall_f32(c);
        v_float32 v_d = vx_setall_f32(d);

        float* errors_cache_ptr = errors_cache.data();
        for (; i <= pts_cnt - v_float32::nlanes; i += v_float32::nlanes)
        {
            v_float32 v_error = v_a * vx_load(pts_ptr_x + i) + v_b * vx_load(pts_ptr_y + i)
                    + v_c * vx_load(pts_ptr_z + i) + v_d;
            v_store(errors_cache_ptr + i, v_error * v_error);
        }
#endif
        for (; i < pts_cnt; ++i) {
            float error = a * pts_ptr_x[i] + b * pts_ptr_y[i] + c * pts_ptr_z[i] + d;
            errors_cache[i] = error * error;
        }
        cache_valid = true;
        return errors_cache;
    }

    Ptr<Error> clone () const override {
        return makePtr<PlaneModelErrorImpl>(*points_mat);
    }

};
Ptr<PlaneModelError> PlaneModelError::create(const Mat &points) {
    return makePtr<PlaneModelErrorImpl>(points);
}

class SphereModelErrorImpl : public SphereModelError, public PointCloudWrapper {
private:
    //! Center and radius
    float center_x, center_y, center_z, radius;
    std::vector<float> errors_cache;
    bool cache_valid;

public:
    explicit SphereModelErrorImpl(const Mat &points_)
            : PointCloudWrapper(points_),
              center_x(0), center_y(0), center_z(0), radius(0),
              errors_cache(pts_cnt), cache_valid(false)
    {
    }

    inline void setModelParameters(const Mat &model) override
    {
        CV_Assert(!model.empty());
        CV_CheckTypeEQ(model.depth(), CV_64F, "Model parameter type should use double");

        const double *const p = (double *) model.data;
        float coeff_x = (float) p[0], coeff_y = (float) p[1],
              coeff_z = (float) p[2], coeff_r = (float) p[3];

        cache_valid = cache_valid && center_x == coeff_x && coeff_y == coeff_y
                      && coeff_z == coeff_z && radius == coeff_r;

        center_x = coeff_x;
        center_y = coeff_y;
        center_z = coeff_z;
        radius = coeff_r;
    }

    inline float getError(int point_idx) const override
    {
        float diff_x = center_x - pts_ptr_x[point_idx];
        float diff_y = center_y - pts_ptr_y[point_idx];
        float diff_z = center_z - pts_ptr_z[point_idx];
        // A better way to avoid sqrt() ?
        float distance_from_center = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
        float diff_dist = distance_from_center - radius;
        double distance_square_from_surface = diff_dist * diff_dist;

        return (float) distance_square_from_surface;
    }

    const std::vector<float> &getErrors(const Mat &model) override
    {
        setModelParameters(model);

        if (cache_valid)
            return errors_cache;

        int i = 0;
#ifdef CV_SIMD
        v_float32 v_center_x = vx_setall_f32(center_x);
        v_float32 v_center_y = vx_setall_f32(center_y);
        v_float32 v_center_z = vx_setall_f32(center_z);
        v_float32 v_radius = vx_setall_f32(radius);

        float* errors_cache_ptr = errors_cache.data();
        for (; i <= pts_cnt - v_float32::nlanes; i += v_float32::nlanes)
        {
            v_float32 v_diff_x = v_center_x - vx_load(pts_ptr_x + i);
            v_float32 v_diff_y = v_center_y - vx_load(pts_ptr_y + i);
            v_float32 v_diff_z = v_center_z - vx_load(pts_ptr_z + i);

            v_float32 v_distance_from_center = v_sqrt(v_diff_x * v_diff_x +
                    v_diff_y * v_diff_y + v_diff_z * v_diff_z);
            v_float32 v_diff_dist = v_distance_from_center - v_radius;
            v_store(errors_cache_ptr + i, v_diff_dist * v_diff_dist);
        }
#endif
        for (; i < pts_cnt; ++i)
        {
            float diff_x = center_x - pts_ptr_x[i];
            float diff_y = center_y - pts_ptr_y[i];
            float diff_z = center_z - pts_ptr_z[i];
            // A better way to avoid sqrt() ?
            float distance_from_center = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
            float diff_dist = distance_from_center - radius;
            errors_cache[i] = diff_dist * diff_dist;
        }

        return errors_cache;
    }

    Ptr<Error> clone () const override {
        return makePtr<SphereModelErrorImpl>(*points_mat);
    }

};
Ptr<SphereModelError> SphereModelError::create(const Mat &points) {
    return makePtr<SphereModelErrorImpl>(points);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Computes forward reprojection error for affine transformation.
class ReprojectionDistanceAffineImpl : public ReprojectionErrorAffine {
private:
    /*
     * m11 m12 m13
     * m21 m22 m23
     * 0   0   1
     */
    const Mat * points_mat;
    const float * const points;
    float m11, m12, m13, m21, m22, m23;
    std::vector<float> errors;
public:
    explicit ReprojectionDistanceAffineImpl (const Mat &points_)
        : points_mat(&points_), points ((float *) points_.data)
        , m11(0), m12(0), m13(0), m21(0), m22(0), m23(0)
        , errors(points_.rows)
    {
        CV_DbgAssert(points);
    }

    inline void setModelParameters(const Mat& model) override
    {
        CV_Assert(!model.empty());
        CV_CheckTypeEQ(model.depth(), CV_64F, "");

        const auto * const m = (double *) model.data;
        m11 = (float)m[0]; m12 = (float)m[1]; m13 = (float)m[2];
        m21 = (float)m[3]; m22 = (float)m[4]; m23 = (float)m[5];
    }
    inline float getError (int point_idx) const override {
        const int smpl = 4*point_idx;
        const float x1=points[smpl], y1=points[smpl+1], x2=points[smpl+2], y2=points[smpl+3];
        const float dx2 = x2 - (m11 * x1 + m12 * y1 + m13), dy2 = y2 - (m21 * x1 + m22 * y1 + m23);
        return dx2 * dx2 + dy2 * dy2;
    }
    const std::vector<float> &getErrors (const Mat &model) override {
        setModelParameters(model);
        for (int point_idx = 0; point_idx < points_mat->rows; point_idx++) {
            const int smpl = 4*point_idx;
            const float x1=points[smpl], y1=points[smpl+1], x2=points[smpl+2], y2=points[smpl+3];
            const float dx2 = x2 - (m11 * x1 + m12 * y1 + m13), dy2 = y2 - (m21 * x1 + m22 * y1 + m23);
            errors[point_idx] = dx2 * dx2 + dy2 * dy2;
        }
        return errors;
    }
    Ptr<Error> clone () const override {
        return makePtr<ReprojectionDistanceAffineImpl>(*points_mat);
    }
};
Ptr<ReprojectionErrorAffine>
ReprojectionErrorAffine::create(const Mat &points) {
    return makePtr<ReprojectionDistanceAffineImpl>(points);
}

////////////////////////////////////// NORMALIZING TRANSFORMATION /////////////////////////
class NormTransformImpl : public NormTransform {
private:
    const float * const points;
public:
    explicit NormTransformImpl (const Mat &points_) : points((float*)points_.data) {}

    // Compute normalized points and transformation matrices.
    void getNormTransformation (Mat& norm_points, const std::vector<int> &sample,
                                int sample_size, Matx33d &T1, Matx33d &T2) const override {
        double mean_pts1_x = 0, mean_pts1_y = 0, mean_pts2_x = 0, mean_pts2_y = 0;

        // find average of each coordinate of points.
        int smpl;
        for (int i = 0; i < sample_size; i++) {
            smpl = 4 * sample[i];

            mean_pts1_x += points[smpl    ];
            mean_pts1_y += points[smpl + 1];
            mean_pts2_x += points[smpl + 2];
            mean_pts2_y += points[smpl + 3];
        }

        mean_pts1_x /= sample_size; mean_pts1_y /= sample_size;
        mean_pts2_x /= sample_size; mean_pts2_y /= sample_size;

        double avg_dist1 = 0, avg_dist2 = 0, x1_m, y1_m, x2_m, y2_m;
        for (int i = 0; i < sample_size; i++) {
            smpl = 4 * sample[i];
            /*
             * Compute a similarity transform T that takes points xi
             * to a new set of points x̃i such that the centroid of
             * the points x̃i is the coordinate origin and their
             * average distance from the origin is √2
             *
             * sqrt(x̃*x̃ + ỹ*ỹ) = sqrt(2)
             * ax*ax + by*by = 2
             */
            x1_m = points[smpl    ] - mean_pts1_x;
            y1_m = points[smpl + 1] - mean_pts1_y;
            x2_m = points[smpl + 2] - mean_pts2_x;
            y2_m = points[smpl + 3] - mean_pts2_y;

            avg_dist1 += sqrt (x1_m * x1_m + y1_m * y1_m);
            avg_dist2 += sqrt (x2_m * x2_m + y2_m * y2_m);
        }

        // scale
        avg_dist1 = M_SQRT2 / (avg_dist1 / sample_size);
        avg_dist2 = M_SQRT2 / (avg_dist2 / sample_size);

        const double transl_x1 = -mean_pts1_x * avg_dist1, transl_y1 = -mean_pts1_y * avg_dist1;
        const double transl_x2 = -mean_pts2_x * avg_dist2, transl_y2 = -mean_pts2_y * avg_dist2;

        // transformation matrices
        T1 = Matx33d (avg_dist1, 0, transl_x1,0, avg_dist1, transl_y1,0, 0, 1);
        T2 = Matx33d (avg_dist2, 0, transl_x2,0, avg_dist2, transl_y2,0, 0, 1);

        norm_points = Mat_<float>(sample_size, 4); // normalized points Nx4 matrix
        auto * norm_points_ptr = (float *) norm_points.data;

        // Normalize points: Npts = T*pts    3x3 * 3xN
        const float avg_dist1f = (float)avg_dist1, avg_dist2f = (float)avg_dist2;
        const float transl_x1f = (float)transl_x1, transl_y1f = (float)transl_y1;
        const float transl_x2f = (float)transl_x2, transl_y2f = (float)transl_y2;
        for (int i = 0; i < sample_size; i++) {
            smpl = 4 * sample[i];
            (*norm_points_ptr++) = avg_dist1f * points[smpl    ] + transl_x1f;
            (*norm_points_ptr++) = avg_dist1f * points[smpl + 1] + transl_y1f;
            (*norm_points_ptr++) = avg_dist2f * points[smpl + 2] + transl_x2f;
            (*norm_points_ptr++) = avg_dist2f * points[smpl + 3] + transl_y2f;
        }
    }
};
Ptr<NormTransform> NormTransform::create (const Mat &points) {
    return makePtr<NormTransformImpl>(points);
}
}}
