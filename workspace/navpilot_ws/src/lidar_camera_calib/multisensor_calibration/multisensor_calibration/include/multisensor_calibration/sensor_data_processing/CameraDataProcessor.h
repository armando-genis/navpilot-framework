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

#ifndef MULTISENSORCALIBRATION_CAMERADATAPROCESSOR_H
#define MULTISENSORCALIBRATION_CAMERADATAPROCESSOR_H

// Std
#include <array>

// ROS
#include <image_transport/image_transport.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// multisensor_calibration
#include "../common/common.h"
#include "DataProcessor2d.h"
#include <multisensor_calibration/common/lib3D/core/extrinsics.hpp>
#include <multisensor_calibration/common/lib3D/core/intrinsics.hpp>

namespace multisensor_calibration
{

/**
 * @ingroup  target_detection
 * @brief Class to processing the image data from the camera and to detect the
 * calibration target within this data.
 */
class CameraDataProcessor : public DataProcessor2d
{
    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Default constructor is deleted.
     *
     */
    CameraDataProcessor() = delete;

    /**
     * @brief Initialization constructor, providing the node name, the sensor name as well
     * as the path to the calibration target configuration file.
     */
    CameraDataProcessor(const std::string& iLoggerName,
                        const std::string& iSensorName,
                        const fs::path& iCalibTargetFilePath);

    /**
     * @brief Destructor
     */
    virtual ~CameraDataProcessor();

    /**
     * @brief Get constant reference to camera intrinsics object.
     */
    const lib3d::Intrinsics& cameraIntrinsics() const;

    /**
     * @brief Get Copy of camera intrinsics object.
     */
    lib3d::Intrinsics getCameraIntrinsics() const;

    /**
     * @brief Get state of the image data that is to be processed.
     */
    EImageState getImageState() const;

    /**
     * @brief Get flag indicating if the camera intrinsics object is set.
     */
    bool isCameraIntrinsicsSet() const;

    /**
     * @brief Callback method to process the image data.
     *
     * In this, first, the markers are detected. If the data is only to be processed as PREVIEW,
     * the border of the markers are drawn into the image and the method returns with success.
     * If the teh processing level is set to TARGET_DETECTION, the corners of the marker are
     * additionally annotated, the board pose is computed based on the detections, and the cloud
     * of the target is computed.
     *
     * @param[in] iCameraImage Image as cv::Mat
     * @param[in] iProcLevel Level at which the data is to be processed, i.e. PREVIEW or
     * TARGET_DETECTION
     * @return Result of the processing.
     */
    EProcessingResult processData(const cv::Mat& iCameraImage,
                                  const EProcessingLevel& iProcLevel) override;

    /**
     * @brief Set the camera intrinsics for the computation of the board pose.
     * @param[in] iIntrinsics Object of camera intrinsics to set.
     */
    void setCameraIntrinsics(const lib3d::Intrinsics& iIntrinsics);

    /**
     * @brief Set the state of the image data that is to be processed.
     */
    void setImageState(const EImageState& iState);

  private:
    /**
     * @brief Method to compute 3D point cloud given a camera image and a pose of the calibration
     * board
     *
     * @param[in] iCameraImage Camera image
     * @param[in] iBoardPose Extrinsic board pose with transformation direction LOCAL_2_REF
     * @param[out] opTargetCloud Pointer to computed point cloud of calibration target.
     * @return True, if successful, false otherwise (e.g. if camera image is empty).
     */
    bool computeTargetCloud(const cv::Mat& iCameraImage, const lib3d::Extrinsics& iBoardPose,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& opTargetCloud);
    /**
     * @brief Method to detect ArUco markers in given camera image. This will also draw the
     * detections into the camera image an provide an annotated copy in oAnnotatedCameraImage.
     *
     * @param[in] iCameraImage Camera image.
     * @param[out] oDetectedMarkerIds Detected marker IDs.
     * @param[out] oDetectedMarkerCorners Corners of the markers corresponding to the detection in
     * oDetectedMarkerIds.
     * @param[out] oAnnotatedCameraImage Copy of the camera image annotated with the detections.
     * @return True if successful, false otherwise (e.g. if camera image is empty). If no markers
     * are detected this will also return true with an empty lis of marker IDs.
     */
    bool detectMarkersInImage(const cv::Mat& iCameraImage,
                              std::vector<int>& oDetectedMarkerIds,
                              std::vector<std::array<cv::Point2f, 4>>& oDetectedMarkerCorners,
                              cv::Mat& oAnnotatedCameraImage) const;

    /**
     * @brief Method to draw given marker corners into image and colorize according to given
     * marker ids.
     *
     * @param[in] iMarkerIds Marker IDs.
     * @param[in] iMarkerCorners Corners of the markers corresponding to the detection in
     * iMarkerIds.
     * @param[in, out] ioImage Image into which to draw the detections.
     */
    void drawMarkerCornersIntoImage(const std::vector<int>& iMarkerIds,
                                    const std::vector<std::array<cv::Point2f, 4>>& iMarkerCorners,
                                    cv::Mat& ioImage) const;

    /**
     * @brief Method to compute board pose based on detected markers
     *
     * @param[in] iDetectedMarkerIds Detected marker IDs.
     * @param[in] iDetectedMarkerCorners Image points of marker corners corresponding to marker IDs.
     * @param[out] oBoardPose Estimated board pose with transformation direction LOCAL_2_REF
     * @return True if successful, False otherwise (i.e. detected markers are empty or
     * pose estimation failed).
     */
    bool estimateBoardPose(const std::vector<int>& iDetectedMarkerIds,
                           const std::vector<std::array<cv::Point2f, 4>>& iDetectedMarkerCorners,
                           lib3d::Extrinsics& oBoardPose) const;

    //--- MEMBER DECLARATION ---//

  private:
    /// Object holding camera intrinsics
    lib3d::Intrinsics cameraIntrinsics_;

    /// Flag indicating if the cameraIntrinsics_ object is set
    bool isCameraIntrinsicsSet_;

    /// List of colors associated with each marker ID
    cv::Mat markerIdColorLookup_;

    /// Pointer to ArUco detector parameters
    cv::Ptr<cv::aruco::DetectorParameters> pArucoDetectorParameters_;

    /// State of image data that is to be processed
    EImageState imageState_;
};

} // namespace multisensor_calibration

#endif