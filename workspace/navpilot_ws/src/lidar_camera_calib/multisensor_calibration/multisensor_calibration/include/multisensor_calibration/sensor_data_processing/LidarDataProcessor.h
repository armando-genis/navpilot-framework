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

#ifndef MULTISENSORCALIBRATION_LIDARDATAPROCESSOR_H
#define MULTISENSORCALIBRATION_LIDARDATAPROCESSOR_H

// ROS
#include <rclcpp/rclcpp.hpp>

// PCL
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// multisensor_calibration
#include "../common/common.h"
#include "../config/LidarTargetDetectionParameters.hpp"
#include "CalibrationTargetSacModel.h"
#include "DataProcessor3d.h"
#include <multisensor_calibration/common/lib3D/core/extrinsics.hpp>

namespace multisensor_calibration
{

/**
 * @ingroup  target_detection
 * @brief Class to processing the point cloud data from the LiDAR sensor and to
 * detect the calibration target within this data.
 */
class LidarDataProcessor : public DataProcessor3d
{

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Default constructor is deleted.
     *
     */
    LidarDataProcessor() = delete;

    /**
     * @brief Initialization constructor, providing the node name, the sensor name as well
     * as the path to the calibration target configuration file.
     */
    LidarDataProcessor(const std::string& iLoggerName,
                       const std::string& iSensorName,
                       const fs::path& iCalibTargetFilePath);

    /**
     * @brief Destructor
     */
    virtual ~LidarDataProcessor();

    /**
     * @brief Callback method to process the point cloud data.
     *
     * In this, first, the point cloud is transformed into a reference coordinate system if
     * according transformation is set with setDataTransform(). After that a preprocessing filter
     * is applied if it has been set using setPreprocFilter(). After the point cloud has been
     * transformed and filtered, the point cloud normals are estimated and the region growing is
     * applied. If the processing level is set to PREVIEW, only the cluster candidates are computed
     * and published. If the processing level is set to TARGET_DETECTION, the target pose is
     * detected and fit into the cloud using RANSAC as described above.
     *
     * @param iPointCloud Point cloud as pcl::PointCloud
     * @param[in] iProcLevel Level at which the data is to be processed, i.e. PREVIEW or
     * TARGET_DETECTION
     * @return Result of the processing.
     */
    EProcessingResult processData(const pcl::PointCloud<InputPointType>& iPointCloud,
                                  const EProcessingLevel& iProcLevel) override;

    /**
     * @brief Method to set pointer to filter used for pre-processing.
     *
     * @param[in] pFilter Pointer to filter object that is to be set. If null, the preprocessing
     * filter is deactivated.
     */
    void setPreprocFilter(const pcl::Filter<InputPointType>::Ptr& pFilter);

    /**
     * @brief Set parameters for target detection in lidar data.
     */
    void setParameters(const LidarTargetDetectionParameters& iParams);

  private:
    /**
     * @brief Method to detect calibration target in cluster cloud using ransac with a poseriori icp
     * optimization
     *
     * @param[in] ipClusterCloud Cluster cloud in which to detect the calibration target.
     * @param[out] oModelCoefficients Estimated model coefficients of the target
     * (nx, ny, nz, d, ux, uy, uz, cx, cy, cz, W, H)
     * @param[out] oRansacInlierCnt Number of inliers after ransac detection
     * @param[out] oIcpInlierCnt Number of inliers after a posteriori icp optimization
     * @return True, if successful. False, otherwise (e.g. if the sac model could not be detected)
     */

    bool detectCalibrationTargetInCluster(const pcl::PointCloud<InputPointType>::Ptr& ipClusterCloud,
                                          Eigen::VectorXf& oModelCoefficients,
                                          std::size_t& oRansacInlierCnt,
                                          std::size_t& oIcpInlierCnt) const;
    /**
     * @brief Method to do a region growing to cluster the point cloud into locally planar regions
     *
     * @param[in] ipInputCloud Point cloud for which normal vectors are to be estimated
     * @param[in] ipNormalCloud Cloud holding normal vectors.
     * @param[in, out] iopSearchTree Search tree used for neighbor search. If not yet initialized,
     * it will be initialized within the normal estimation.
     * @param[out] oClusterIndices List of groups holding point indices of separate clusters.
     */
    void doRegionGrowing(const pcl::PointCloud<InputPointType>::Ptr& ipInputCloud,
                         const pcl::PointCloud<pcl::Normal>::Ptr& ipNormalCloud,
                         pcl::search::Search<InputPointType>::Ptr& iopSearchTree,
                         std::vector<pcl::PointIndices>& oClusterIndices) const;

    /**
     * @brief Method to estimate normal vectors for given input cloud.
     *
     * @param[in] ipInputCloud Point cloud for which normal vectors are to be estimated
     * @param[in, out] iopSearchTree Search tree used for neighbor search. If not yet initialized,
     * it will be initialized within the normal estimation.
     * @param[out] opNormalCloud Cloud holding normal vectors.
     */
    void estimateCloudNormals(const pcl::PointCloud<InputPointType>::Ptr& ipInputCloud,
                              pcl::search::Search<InputPointType>::Ptr& iopSearchTree,
                              pcl::PointCloud<pcl::Normal>::Ptr& opNormalCloud) const;

    /**
     * @brief Method to project cluster cloud onto a planar model which is estimated using RANSAC.
     *
     * @param[in] ipClusterCloud Cluster cloud to project.
     * @param[out] opPlanarClusterCloud Projected cloud.
     * @param[out] oPlaneParameters Estimated 4D plane parameters (nx, ny, nz, d)
     * @return True if successful. False, otherwise (e.g. if plane model could not be estimated)
     */
    bool projectClusterToPlanarModel(const pcl::PointCloud<InputPointType>::Ptr& ipClusterCloud,
                                     pcl::PointCloud<InputPointType>::Ptr& opPlanarClusterCloud,
                                     Eigen::VectorXf& oPlaneParameters) const;

    /**
     * @brief Test cluster based on its size.
     *
     * @param[in] ipCluster Pointer to cluster cloud
     * @param[out] oBboxTransform Transformation of the bounding box to fit the input point cloud. This is
     * equivalent to a transformation from a local to a reference coordinate system. Meaning, that the
     * translational part is equivalent to the center of the bounding box as seen from the reference
     * system and, that the column vectors of the rotation matrix represent the orientation of the
     * axes of the local coordinate system as seen from the reference system. The rotation matrix
     * is set in such a way, that the z-axis is pointing towards the center of the sensor.
     * @return True, if the size of the cluster is in the margin of the calibration target size.
     * False otherwise.
     */
    bool testClusterSize(const pcl::PointCloud<InputPointType>::Ptr& ipCluster,
                         Eigen::Matrix4f& oBboxTransform) const;

    //--- STATIC FUNCTION DECLARATION ---//

  public:
    /**
     * @brief Method to extract a planar point cloud from input cloud.
     *
     * This uses the RANSAC based segmentation of the PCL to find a plane which has a normal vector
     * that parallel to given vector within a given tolerance.
     *
     * @param[in] ipInputCloud Pointer to input cloud.
     * @param[in] iNormalVec Desired normal direction of the plane which is to be found.
     * @param[in] iAngleTolerance Angular tolerance between given and predicted normal vector
     * in degrees.
     * @param[out] opOutputCloud Pointer to output cloud. If the pointer is a nullptr, a new
     * pointcloud will be instantiated.
     */
    static bool extractPlaneFromPointCloud(const pcl::PointCloud<InputPointType>::Ptr& ipInputCloud,
                                           const Eigen::Vector3f& iNormalVec,
                                           const double& iAngleTolerance,
                                           pcl::PointCloud<InputPointType>::Ptr& opOutputCloud);

    /**
     * @brief Overloaded function.
     */
    static bool extractPlaneFromPointCloud(const pcl::PointCloud<InputPointType>::Ptr& ipInputCloud,
                                           const tf2::Vector3& iNormalVec,
                                           const double& iAngleTolerance,
                                           pcl::PointCloud<InputPointType>::Ptr& opOutputCloud);

    //--- MEMBER DECLARATION ---//

  private:
    /// Object holding parameters for the target detection
    LidarTargetDetectionParameters targetDetectionParams_;

    /// Pointer to sample consensus model of calibration target
    CalibrationTargetSacModel<InputPointType>::Ptr pCalibTargetSacModel_;

    /// Pointer to preprocessing filter that is to be applied at the beginning of data processing.
    pcl::Filter<InputPointType>::Ptr pPreprocFilter_;
};

} // namespace multisensor_calibration

#endif