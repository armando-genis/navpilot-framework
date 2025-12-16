/***********************************************************************
 *
 *   Copyright (c) 2022 - 2023 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#ifndef MULTISENSORCALIBRATION_EMPTYCIRCLE3DSACMODEL_H
#define MULTISENSORCALIBRATION_EMPTYCIRCLE3DSACMODEL_H

#include <pcl/sample_consensus/sac_model_circle3d.h>

namespace multisensor_calibration
{

/**
 * @ingroup  ransac
 * @brief EmptyCircle3DSacModel defines a pcl sample consensus model of an empty planar circle.
 *
 * The model coefficients are defined as:
 *   - @b center_x : the X coordinate of the circles' center point
 *   - @b center_y : the Y coordinate of the circles' center point
 *   - @b center_z : the Z coordinate of the circles' center point
 *   - @b radius : the radius of the circle
 *   - @b normal_x : the X coordinate of the circles' normal vector (normalized)
 *   - @b normal_y : the Y coordinate of the circles' normal vector (normalized)
 *   - @b normal_z : the Z coordinate of the circles' normal vector (normalized)
 *
 */
template <typename PointT>
class EmptyCircle3DSacModel : public pcl::SampleConsensusModelCircle3D<PointT>
{
  protected:
    using pcl::SampleConsensusModel<PointT>::model_name_;
    using pcl::SampleConsensusModel<PointT>::input_;
    using pcl::SampleConsensusModel<PointT>::indices_;
    using pcl::SampleConsensusModel<PointT>::radius_min_;
    using pcl::SampleConsensusModel<PointT>::radius_max_;

  public:
    using PointCloud         = typename pcl::SampleConsensusModel<PointT>::PointCloud;
    using PointCloudPtr      = typename pcl::SampleConsensusModel<PointT>::PointCloudPtr;
    using PointCloudConstPtr = typename pcl::SampleConsensusModel<PointT>::PointCloudConstPtr;

    using Ptr      = pcl::shared_ptr<EmptyCircle3DSacModel<PointT>>;
    using ConstPtr = pcl::shared_ptr<const EmptyCircle3DSacModel<PointT>>;

    //--- METHOD DECLARATION ---//

    /** @brief Constructor for base EmptyCircle3DSacModel.
     * @param[in] cloud the input point cloud dataset
     * @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
     */
    EmptyCircle3DSacModel(const PointCloudConstPtr& cloud,
                          bool random = false) :
      pcl::SampleConsensusModelCircle3D<PointT>(cloud, random),
      normalAxis_(Eigen::Vector3f::Zero()),
      epsAngleNormal_(0.0),
      invalidParameterization_(),
      expectedPointDensity_(-1),
      minSupportRadius_(-1),
      maxSupportRadius_(-1)
    {
        model_name_  = "SampleConsensusModelEmptyCircle3D";
        sample_size_ = 3;
        model_size_  = 7;
    }

    /** @brief Constructor for base SampleConsensusModelEmptyCircle3D.
     * @param[in] cloud the input point cloud dataset
     * @param[in] indices a vector of point indices to be used from @a cloud
     * @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
     */
    EmptyCircle3DSacModel(const PointCloudConstPtr& cloud,
                          const std::vector<int>& indices,
                          bool random = false) :
      pcl::SampleConsensusModelCircle3D<PointT>(cloud, indices, random),
      normalAxis_(Eigen::Vector3f::Zero()),
      epsAngleNormal_(0.0),
      invalidParameterization_(),
      expectedPointDensity_(-1),
      minSupportRadius_(-1),
      maxSupportRadius_(-1)
    {
        model_name_  = "SampleConsensusModelEmptyCircle3D";
        sample_size_ = 3;
        model_size_  = 7;
    }

    /** @brief Empty destructor */
    ~EmptyCircle3DSacModel()
    {
    }

    /** @brief Set the axis along which we need to search for a circle normal.
     * @param[in] ax the axis
     */
    void setAxis(const Eigen::Vector3f& ax)
    {
        normalAxis_ = ax;
    }

    /** @brief Get the axis along which we need to search for a circle normal. */
    Eigen::Vector3f getAxis() const
    {
        return (normalAxis_);
    }

    /** @brief Set the angle epsilon (delta) threshold.
     * @param[in] ea the maximum allowed difference (in degrees) between the plane normal and the given axis.
     * @note You need to specify an angle > 0 in order to activate the axis-angle constraint!
     */
    void setEpsAngle(const double ea)
    {
        epsAngleNormal_ = ea;
    }

    /** @brief Get the angle epsilon (in degrees) threshold. */
    double getEpsAngle() const
    {
        return (epsAngleNormal_);
    }

    /**
     * @brief Set the point density expected of the point cloud in which the target is to be detected.
     *
     * @note Density needs to be greater than 0
     */
    void setExpectedPointDensity(const float& iExpectedDensity)
    {
        if (iExpectedDensity <= 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("multisensor_calibration::EmptyCircle3DSacModel"),
                         "%s: Expected point density needs to be greater than 0.",
                         __PRETTY_FUNCTION__);
            return;
        }

        expectedPointDensity_ = iExpectedDensity;
    }

    /**
     * @brief Set the limits (min and max radius) of the support region.
     *
     * @note Both radii need to be greater than 0 and the max radius needs to be larger to
     * min radius.
     */
    void setSupportRegionLimits(const float& iMinSupportRadius, const float& iMaxSupportRadius)
    {
        if ((iMinSupportRadius <= 0) || (iMinSupportRadius >= iMaxSupportRadius))
        {
            RCLCPP_ERROR(rclcpp::get_logger("multisensor_calibration::EmptyCircle3DSacModel"),
                         "%s: Both radii need to be greater than 0 and the max radius needs to be "
                         "larger to min radius.",
                         __PRETTY_FUNCTION__);
            return;
        }

        minSupportRadius_ = iMinSupportRadius;
        maxSupportRadius_ = iMaxSupportRadius;
    }

    /**
     * @brief Method to add a parameterization that is to be considered as invalid.
     *
     * @param[in] newInvalidParameterization parameterization as center and radius in the same order
     *  as stored in the model coefficients.
     */
    void addInvalidParameterization(const Eigen::VectorXf& newInvalidParameterization)
    {
        if (newInvalidParameterization.size() < 4)
        {
            RCLCPP_ERROR(rclcpp::get_logger("multisensor_calibration::EmptyCircle3DSacModel"),
                         "%s: Parameterization to be added as 'invalid' is not of expected size.",
                         __PRETTY_FUNCTION__);
            return;
        }

        invalidParameterization_.push_back(newInvalidParameterization);
    }

    /**
     * @brief Method to check whether given samples are good.
     *
     * This checks that all enclosed angle of the triangle with these three points are greater than
     * 30°.
     */
    bool isSampleGood(const std::vector<int>& samples) const
    {
        //--- Get the values at the three points
        Eigen::Vector3d p0(input_->points[samples[0]].x,
                           input_->points[samples[0]].y,
                           input_->points[samples[0]].z);
        Eigen::Vector3d p1(input_->points[samples[1]].x,
                           input_->points[samples[1]].y,
                           input_->points[samples[1]].z);
        Eigen::Vector3d p2(input_->points[samples[2]].x,
                           input_->points[samples[2]].y,
                           input_->points[samples[2]].z);

        //--- calculate normalized vectors between points
        Eigen::Vector3d v10 = (p1 - p0).normalized();
        Eigen::Vector3d v20 = (p2 - p0).normalized();
        Eigen::Vector3d v01 = (p0 - p1).normalized();
        Eigen::Vector3d v21 = (p2 - p1).normalized();
        Eigen::Vector3d v12 = (p1 - p2).normalized();
        Eigen::Vector3d v02 = (p0 - p2).normalized();

        //--- compute cosine of enclosed angle
        float c0 = v10.dot(v20);
        float c1 = v01.dot(v21);
        float c2 = v12.dot(v02);

        //--- compute threshold
        float thresh = std::cos(30.0 / 180.0 * M_PI);

        return (c0 <= thresh && c1 <= thresh && c2 <= thresh);
    }

    /** @brief Count all the points which respect the given model coefficients as inliers.
     *
     * @param[in] modelCoefficients the coefficients of a model that we need to compute distances to
     * @param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
     * @return the resultant number of inliers
     */
    std::size_t countWithinDistance(const Eigen::VectorXf& modelCoefficients,
                                    const double threshold) const override
    {
        //--- Check if the model is valid given the user constraints
        if (!isModelValid(modelCoefficients))
            return 0;

        //--- check if point density is initialized
        if (expectedPointDensity_ <= 0)
            return 0;

        //--- set radius bounds of support region if uninitialized
        float tmpMinSupportRadius = (minSupportRadius_ == -1) ? radius_min_ : minSupportRadius_;
        float tmpMaxSupportRadius = (maxSupportRadius_ == -1) ? radius_max_ : maxSupportRadius_;

        //--- initialize inlier counter
        float innerSupportRegionCount = (calculateCircleArea(tmpMinSupportRadius) * expectedPointDensity_);
        float outerSupportRegionCount = 0;

        //--- Iterate through the 3d points and calculate the distances from them to the sphere
        for (std::size_t i = 0; i < indices_->size(); ++i)
        {
            // what i have:
            // P : Sample Point
            Eigen::Vector3d P(input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z);
            // C : Circle Center
            Eigen::Vector3d C(modelCoefficients[0], modelCoefficients[1], modelCoefficients[2]);
            // N : Circle (Plane) Normal
            Eigen::Vector3d N(modelCoefficients[4], modelCoefficients[5], modelCoefficients[6]);
            // r : Radius
            double r = modelCoefficients[3];

            // Connection vector from C to P
            Eigen::Vector3d helper_vectorCP = P - C;

            // Sample point projected on plane of circle
            Eigen::Vector3d P_proj = P + (-(helper_vectorCP.dot(N))) / N.dot(N) * N;

            // Connection vector from from C to P_proj
            Eigen::Vector3d helper_vectorCP_proj = P_proj - C;
            double distancePC                    = helper_vectorCP_proj.norm();

#if 0
            // K : Point on Circle
            Eigen::Vector3d K              = C + r * helper_vectorP_projC.normalized();
            Eigen::Vector3d distanceVector = P - K;

            //--- if distance to circle is below threshold increase counter
            if (distanceVector.norm() < threshold)
                nr_p++;
#endif

            //--- if point lies in outer support region increase count
            if (tmpMinSupportRadius <= distancePC && distancePC <= tmpMaxSupportRadius)
                outerSupportRegionCount++;

            //--- if point lies in inner support region decrease count according to weight
            if (distancePC < tmpMinSupportRadius)
            {
                // amplitude of weight calculation
                const float WEIGHT_AMPL = 5.f;

                //--- calculate weight based on distance to center
                // float weight = (WEIGHT_AMPL * std::cos((M_PI / minSupportRadius_) * distancePC)) +
                //                (WEIGHT_AMPL + 1);
                float weight = WEIGHT_AMPL;

                //--- subtract weight from innerSupportRegionCount_
                innerSupportRegionCount -= weight;
            }
        }

        //--- truncate inlier count to 0
        return static_cast<std::size_t>(
          std::max(0.f, std::round(innerSupportRegionCount + outerSupportRegionCount)));
    }

    /** @brief Select all the points which respect the given model coefficients as inliers.
     *
     * @param[in] modelCoefficients the coefficients of a model that we need to compute distances to
     * @param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
     * @param[in] inliers
     * @return the resultant number of inliers
     */
    void selectWithinDistance(const Eigen::VectorXf& modelCoefficients,
                              const double threshold,
                              std::vector<int>& inliers) override
    {
        //--- Check if the model is valid given the user constraints
        if (!isModelValid(modelCoefficients))
        {
            inliers.clear();
            return;
        }

        inliers.resize(indices_->size());

        //--- set radius bounds of support region if uninitialized
        float tmpMaxSupportRadius = (maxSupportRadius_ == -1) ? radius_max_ : maxSupportRadius_;

        //--- initialize inlier counter
        float outerSupportRegionCount = 0;

        //--- Iterate through the 3d points and calculate the distances from them to the sphere
        for (std::size_t i = 0; i < indices_->size(); ++i)
        {
            // what i have:
            // P : Sample Point
            Eigen::Vector3d P(input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z);
            // C : Circle Center
            Eigen::Vector3d C(modelCoefficients[0], modelCoefficients[1], modelCoefficients[2]);
            // N : Circle (Plane) Normal
            Eigen::Vector3d N(modelCoefficients[4], modelCoefficients[5], modelCoefficients[6]);
            // r : Radius
            double r = modelCoefficients[3];

            // Connection vector from C to P
            Eigen::Vector3d helper_vectorCP = P - C;

            // Sample point projected on plane of circle
            Eigen::Vector3d P_proj = P + (-(helper_vectorCP.dot(N))) / N.dot(N) * N;

            // Connection vector from from C to P_proj
            Eigen::Vector3d helper_vectorCP_proj = P_proj - C;
            double distancePC                    = helper_vectorCP_proj.norm();

            //--- if point lies in outer support region increase count
            if (distancePC <= tmpMaxSupportRadius)
            {
                inliers[outerSupportRegionCount] = (*indices_)[i];
                outerSupportRegionCount++;
            }
        }

        inliers.resize(outerSupportRegionCount);
    }

    /**
     * @brief Get score for a given amount of inliers.
     *
     * The score is the ratio between the given number of inliers and the expected number of inliers
     * based on size of the support region, defined by the maximum support radius
     */
    float getScore(const int& iNumInliers) const
    {
        //--- check if point density is initialized
        if (expectedPointDensity_ <= 0)
            return 0.f;

        //--- set radius bounds of support region if uninitialized
        float tmpMaxSupportRadius = (maxSupportRadius_ == -1) ? radius_max_ : maxSupportRadius_;

        //--- calculate score
        float score = static_cast<float>(iNumInliers) /
                      (expectedPointDensity_ * calculateCircleArea(tmpMaxSupportRadius));

        //--- truncate score to interval [0,1]
        return std::max(0.f, std::min(score, 1.f));
    }

  protected:
    /** @brief Check whether a model is valid given the user constraints.
     * @param[in] modelCoefficients the set of model coefficients
     */
    bool isModelValid(const Eigen::VectorXf& modelCoefficients) const override
    {
        //--- call parent method
        //--- in this, it is checked whether the computed radius is in between
        //--- min_radius_ and max_radius_
        if (!pcl::SampleConsensusModelCircle3D<PointT>::isModelValid(modelCoefficients))
            return (false);

        // estimated center point
        Eigen::Vector3f estCenter = modelCoefficients.block<3, 1>(0, 0);

        // estimated radius
        float estRadius = modelCoefficients[3];

        // estimated normal vector
        Eigen::Vector3f estNormalVector = modelCoefficients.block<3, 1>(4, 0);

        //--- Check against orientation template, if given
        if (epsAngleNormal_ > 0.0 && !normalAxis_.isZero())
        {

            double angle_diff = std::abs(pcl::getAngle3D(normalAxis_, estNormalVector, true));
            angle_diff        = (std::min)(angle_diff, 180.0 - angle_diff);

            //--- Check whether the current plane model satisfies our angle threshold criterion with
            //--- respect to the given axis
            if (angle_diff > epsAngleNormal_)
                return (false);
        }

        //--- check against invalid parameterization, if given
        if (!invalidParameterization_.empty())
        {
            for (Eigen::VectorXf invalidParam : invalidParameterization_)
            {
                //--- if distance between new center and invalid center is smaller than new radius +
                //--- invalid radius, return false

                Eigen::Vector3f invalidCenter = invalidParam.block<3, 1>(0, 0);
                float invalidRadius           = invalidParam[3];

                if ((invalidCenter - estCenter).norm() < (estRadius + invalidRadius))
                    return false;
            }
        }

        return (true);
    }

    /**
     * @brief Compute area of a circle with given radius
     */
    inline float calculateCircleArea(const float& iRadius) const
    {
        return (M_PI * iRadius * iRadius);
    }

    //--- MEMBER DECLARATION ---//

  protected:
    using pcl::SampleConsensusModel<PointT>::sample_size_;
    using pcl::SampleConsensusModel<PointT>::model_size_;

  private:
    /// The axis along which we need to search for a circle normal.
    Eigen::Vector3f normalAxis_;

    /// The maximum allowed difference (in degrees) between the circle normal and the given axis.
    double epsAngleNormal_;

    /// List of parameterization (center and radius in the same order as stored in the model
    /// coefficients) which are not allowed.
    std::vector<Eigen::VectorXf> invalidParameterization_;

    /// Point density that is to be expected of the point cloud in which the empty circle is tto be
    /// detected.
    float expectedPointDensity_;

    /// Minimum radius of support region. This will be set to radius_min_ if uninitialized.
    float minSupportRadius_;

    /// Maximum radius of support region. This will be set to radius_max_ if uninitialized.
    float maxSupportRadius_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_EMPTYCIRCLE3DSACMODEL_H