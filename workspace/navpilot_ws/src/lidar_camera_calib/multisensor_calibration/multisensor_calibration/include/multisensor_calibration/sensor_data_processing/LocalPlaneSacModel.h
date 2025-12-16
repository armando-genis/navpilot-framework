// Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef MULTISENSORCALIBRATION_LOCALPLANESACMODEL_H
#define MULTISENSORCALIBRATION_LOCALPLANESACMODEL_H

// Std
#include <memory>

// PCL
#include <pcl/sample_consensus/sac_model_plane.h>

namespace multisensor_calibration
{

/**
 * @ingroup  ransac
 * @brief PCL sample consensus model of a local surface plane with
 * a location and a radius.
 *
 * The model coefficients are defined as:
 *   - @b normal_x : the X coordinate of the plane's normal vector (normalized)
 *   - @b normal_y : the Y coordinate of the plane's normal vector (normalized)
 *   - @b normal_z : the Z coordinate of the plane's normal vector (normalized)
 *   - @b d : the fourth <a href="http://mathworld.wolfram.com/HessianNormalForm.html">Hessian component</a> of the board's equation
 *   - @b center_x : the X coordinate of the plane's center
 *   - @b center_y : the Y coordinate of the plane's center
 *   - @b center_z : the Z coordinate of the plane's center
 *   - @b radius : the radius of the local plane
 *
 * When used within the RANSAC algorithm of the PCL, the inliers are chosen not only based on the
 * distance with respect to the plane, if it lies within the given radius to the center point.
 */
template <typename PointT>
class LocalPlaneSacModel : public pcl::SampleConsensusModelPlane<PointT>
{
  protected:
    using pcl::SampleConsensusModel<PointT>::model_name_;
    using pcl::SampleConsensusModel<PointT>::input_;
    using pcl::SampleConsensusModel<PointT>::indices_;
    using pcl::SampleConsensusModel<PointT>::error_sqr_dists_;

  public:
    using PointCloud         = typename pcl::SampleConsensusModel<PointT>::PointCloud;
    using PointCloudPtr      = typename pcl::SampleConsensusModel<PointT>::PointCloudPtr;
    using PointCloudConstPtr = typename pcl::SampleConsensusModel<PointT>::PointCloudConstPtr;

    using Ptr      = pcl::shared_ptr<LocalPlaneSacModel<PointT>>;
    using ConstPtr = pcl::shared_ptr<const LocalPlaneSacModel<PointT>>;

    //--- METHOD DECLARATION ---//

    /** @brief Constructor for base LocalPlaneSacModel.
     * @param[in] cloud the input point cloud dataset
     * @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
     */
    LocalPlaneSacModel(const PointCloudConstPtr& cloud,
                       bool random = false);

    /** @brief Constructor for base LocalPlaneSacModel.
     * @param[in] cloud the input point cloud dataset
     * @param[in] indices a vector of point indices to be used from @a cloud
     * @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
     */
    LocalPlaneSacModel(const PointCloudConstPtr& cloud,
                       const std::vector<int>& indices,
                       bool random = false);

    /** @brief Empty destructor */
    ~LocalPlaneSacModel();

    /**
     * @brief Clear list of indices.
     */
    void clearIndices();

    /**
     * @brief Method to set center of local plane as seen from the sensors coordinate system.
     *
     * @param[in] iCenter Translational vector as seen from the reference coordinate system.
     */
    void setCenter(const Eigen::Vector3f& iCenter);

    /**
     * @brief Method to set radius of local plane.
     *
     * @param[in] iRadius Value of radius to set.
     */
    void setRadius(const float& iRadius);

    /**
     * @brief Method to set incremental center update.
     *
     * If set to true, this will allow the model to keep track of the number of inliers and update
     * the center point each time the number of inlier increases.
     */
    void setIncrementalCenterUpdate(const bool iValue);

    /**
     * @brief Set the shift value (with respect to the pose guess) with which different positions of
     * the target should be testet.
     *
     * @param iValue Value in meters.
     */
    void setTranslationVariance(const double iValue);

    /** \brief Check whether the given index samples can form a valid plane model, compute the model coefficients from
     * these samples and store them internally in model_coefficients_. The plane coefficients are:
     * a, b, c, d (ax+by+cz+d=0)
     * \param[in] samples the point indices found as possible good candidates for creating a valid model
     * \param[out] model_coefficients the resultant model coefficients
     */
    bool computeModelCoefficients(const std::vector<int>& samples,
                                  Eigen::VectorXf& model_coefficients) const override;

    /** \brief Select all the points which respect the given model coefficients as inliers.
     * \param[in] model_coefficients the coefficients of a plane model that we need to compute distances to
     * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
     * \param[out] inliers the resultant model inliers
     */
    void selectWithinDistance(const Eigen::VectorXf& model_coefficients,
                              const double threshold,
                              std::vector<int>& inliers) override;

    /** \brief Count all the points which respect the given model coefficients as inliers.
     *
     * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
     * \param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
     * \return the resultant number of inliers
     */
    std::size_t countWithinDistance(const Eigen::VectorXf& model_coefficients,
                                    const double threshold) const override;

    /** \brief Recompute the plane coefficients using the given inlier set and return them to the user.
     * @note: these are the coefficients of the plane model after refinement (e.g. after SVD)
     * \param[in] inliers the data inliers found as supporting the model
     * \param[in] model_coefficients the initial guess for the model coefficients
     * \param[out] optimized_coefficients the resultant recomputed coefficients after non-linear optimization
     */
    void optimizeModelCoefficients(const std::vector<int>& inliers,
                                   const Eigen::VectorXf& model_coefficients,
                                   Eigen::VectorXf& optimized_coefficients) const override;

    /**
     * @brief Provide a pointer to the input dataset. This will first clear the list of indices.
     * @param[in] cloud the const boost shared pointer to a PointCloud message
     */
    void setInputCloud(const PointCloudConstPtr& cloud) override;

  protected:
    /** \brief Check whether a model is valid given the user constraints.
     * \param[in] model_coefficients the set of model coefficients
     */
    bool isModelValid(const Eigen::VectorXf& model_coefficients) const override;

    //--- MEMBER DECLARATION ---//

  protected:
    using pcl::SampleConsensusModel<PointT>::sample_size_;
    using pcl::SampleConsensusModel<PointT>::model_size_;

  private:
    /// Center point of the local plane
    std::unique_ptr<Eigen::Vector3f> pPlaneCenter_;

    /// Radius of the local plane
    float planeRadius_;

    /// Flag to activate the incremental update of the center point
    bool isCenterUpdateSet_;

    /// Maximum number of inliers counted so far.
    std::unique_ptr<std::size_t> pWtaInlierCount_;

    /// Variance in translation, i.e. by how many meters should the translation be sampled when
    /// computing the new coefficients
    double translationVarianceMeters_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_LOCALPLANESACMODEL_H