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

#ifndef MULTISENSORCALIBRATION_CALIBRATIONTARGETSACMODEL_H
#define MULTISENSORCALIBRATION_CALIBRATIONTARGETSACMODEL_H

// Std
#include <memory>

// PCL
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

// small_gicp
#include <small_gicp/registration/registration_helper.hpp>

// multisensor_calibration
#include "../calibration_target/CalibrationTarget.hpp"

namespace multisensor_calibration
{

/**
 * @ingroup  target_detection
 * @ingroup  ransac
 * @brief PCL sample consensus model of the calibration target to be used within the RANSAC
 * algorithm of the PCL to detect and fit the calibration target within in point cloud
 *
 * The model coefficients are defined as:
 *   - @b normal_x : the X coordinate of the target's normal vector (normalized)
 *   - @b normal_y : the Y coordinate of the target's normal vector (normalized)
 *   - @b normal_z : the Z coordinate of the target's normal vector (normalized)
 *   - @b d : the fourth <a href="http://mathworld.wolfram.com/HessianNormalForm.html">
 *            Hessian component</a> of the board's equation
 *   - @b up_x : the X coordinate of the target's up vector (normalized)
 *   - @b up_y : the Y coordinate of the target's up vector (normalized)
 *   - @b up_z : the Z coordinate of the target's up vector (normalized)
 *   - @b center_x : the X coordinate of the target's center
 *   - @b center_y : the Y coordinate of the target's center
 *   - @b center_z : the Z coordinate of the target's center
 *   - @b width : the width of the target
 *   - @b height : the height of the target
 *
 * When used within the RANSAC algorithm of the PCL, in each iteration the up vector, as well as
 * the center point will be varied according to the values set in setRotationVariance() and
 * setTranslationVariance() in order to find the best fitting pose. With
 * incrementalPoseGuessUpdate() the model can be set to keep track of the number of inliers and update
 * the poseGuess_ each time the number of inlier increases.
 */
template <typename PointT>
class CalibrationTargetSacModel : public pcl::SampleConsensusModelPerpendicularPlane<PointT>
{
  protected:
    using pcl::SampleConsensusModel<PointT>::model_name_;
    using pcl::SampleConsensusModel<PointT>::input_;
    using pcl::SampleConsensusModel<PointT>::indices_;
    using pcl::SampleConsensusModel<PointT>::error_sqr_dists_;
    using pcl::SampleConsensusModelPerpendicularPlane<PointT>::axis_;
    using pcl::SampleConsensusModelPerpendicularPlane<PointT>::eps_angle_;

  public:
    using PointCloud         = typename pcl::SampleConsensusModel<PointT>::PointCloud;
    using PointCloudPtr      = typename pcl::SampleConsensusModel<PointT>::PointCloudPtr;
    using PointCloudConstPtr = typename pcl::SampleConsensusModel<PointT>::PointCloudConstPtr;

    using Ptr      = pcl::shared_ptr<CalibrationTargetSacModel<PointT>>;
    using ConstPtr = pcl::shared_ptr<const CalibrationTargetSacModel<PointT>>;

    //--- METHOD DECLARATION ---//

    /** @brief Constructor for CalibrationTargetSacModel.
     * @param[in] target object of the calibration target.
     * @param[in] cloud the input point cloud dataset.
     * @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
     */
    CalibrationTargetSacModel(const CalibrationTarget& target, const PointCloudConstPtr& cloud,
                              bool random = false);

    /** @brief Constructor for CalibrationTargetSacModel.
     * @param[in] target object of the calibration target.
     * @param[in] cloud the input point cloud dataset
     * @param[in] indices a vector of point indices to be used from @a cloud
     * @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
     */
    CalibrationTargetSacModel(const CalibrationTarget& target,
                              const PointCloudConstPtr& cloud,
                              const std::vector<int>& indices,
                              bool random = false);

    /** @brief Empty destructor */
    ~CalibrationTargetSacModel();

    /**
     * @brief Clear list of indices.
     */
    void clearIndices();

    /**
     * @brief Method to set guess of board pose as seen from the sensors coordinate system.
     *
     * @param[in] iCenter Translational vector as seen from the reference coordinate system.
     * @param[in] iUp Up vector as seen from the reference coordinate system.
     * @param[in] iNormal Normal vector as seen from the reference coordinate system and pointing
     * into direction of the origin of the reference coordinate system.
     */
    void setPoseGuess(const Eigen::Vector3f& iCenter,
                      const Eigen::Vector3f& iUp,
                      const Eigen::Vector3f& iNormal);

    /**
     * @brief Method to set incremental pose guess.
     *
     * If set to true, this will allow the model to keep track of the number of inliers and update
     * the poseGuess_ each time the number of inlier increases.
     */
    void setIncrementalPoseGuessUpdate(const bool iValue);

    /**
     * @brief Set the variance up to which different rotations of the target pose (to the left and
     * right of the up-vector) should be testet.
     *
     * @param iValue Value in degrees, in the range of [0,180]
     */
    void setRotationVariance(const double iValue);

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

    /** \brief Optimize the coefficients with the help of ICP and the full input cloud and return
     * them to the user.
     * @note: these are the coefficients of the plane model after refinement (e.g. after SVD)
     * \param[in] model_coefficients the initial guess for the model coefficients
     * \param[out] optimized_coefficients the resultant recomputed coefficients after non-linear optimization
     */
    void optimizeModelCoefficients(const Eigen::VectorXf& model_coefficients,
                                   Eigen::VectorXf& optimized_coefficients) const;

    /** \brief Optimize the coefficients with the help of ICP and the given inlier set and return them
     * to the user.
     * @note: these are the coefficients of the plane model after refinement (e.g. after SVD)
     * \param[in] inliers the data inliers found as supporting the model
     * \param[in] model_coefficients the initial guess for the model coefficients
     * \param[out] optimized_coefficients the resultant recomputed coefficients after non-linear optimization
     */
    void optimizeModelCoefficients(const std::vector<int>& inliers,
                                   const Eigen::VectorXf& model_coefficients,
                                   Eigen::VectorXf& optimized_coefficients) const override;

    /**
     * @brief Set maximum distance for ICP to search for point correspondences. Given as ratio with
     * respect to shorter side of calibration target.
     */
    void setIcpCorrespondenceDistance(const double& ratio);

    /**
     * @brief Rotation tolerance for convergence check. Given in degrees.
     */
    void setIcpRotationTolerance(const double& tolerance_deg);

    /**
     * @brief Translation tolerance for convergence check. Given in unit of LiDAR point cloud,
     * typically meters.
     */
    void setIcpTranslationTolerance(const double& tolerance);

    /**
     * @brief Set the ICP variant used for the optimization of coefficients.
     */
    void setIcpVariant(const small_gicp::RegistrationSetting::RegistrationType& type);

    /**
     * @brief Provide a pointer to the input dataset. This will first clear the list of indices.
     * @param[in] cloud the const boost shared pointer to a PointCloud message
     */
    void setInputCloud(const PointCloudConstPtr& cloud) override;

  protected:
    /**
     * @brief Initialize model.
     */
    virtual void init(const CalibrationTarget& target);

    /** \brief Check whether a model is valid given the user constraints.
     * \param[in] model_coefficients the set of model coefficients
     */
    bool isModelValid(const Eigen::VectorXf& model_coefficients) const override;

    //--- MEMBER DECLARATION ---//

  protected:
    using pcl::SampleConsensusModel<PointT>::sample_size_;
    using pcl::SampleConsensusModel<PointT>::model_size_;

  private:
    /// Object of the calibration target
    CalibrationTarget calibrationTarget_;

    /// Pointer to cloud of CAD model (if available)
    typename pcl::PointCloud<PointT>::Ptr pCadModelCloud_;

    /// Initial guess of target pose as seen from the sensor origin
    std::unique_ptr<lib3d::Extrinsics> pPoseGuess_;

    /// Flag to activate the incremental update of the pose guess
    bool isPoseGuessUpdateSet_;

    /// Maximum number of inliers counted so far.
    std::unique_ptr<std::size_t> pWtaInlierCount_;

    /// Variance in rotation, i.e. by how many degrees (+-) should the rotation be sampled when
    /// computing the new coefficients
    double rotationVarianceAngle_;

    /// Variance in translation, i.e. by how many meters should the translation be sampled when
    /// computing the new coefficients
    double translationVarianceMeters_;

    /// Settings for ICP to optimize coefficients
    small_gicp::RegistrationSetting icpSettings_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_CALIBRATIONTARGETSACMODEL_H