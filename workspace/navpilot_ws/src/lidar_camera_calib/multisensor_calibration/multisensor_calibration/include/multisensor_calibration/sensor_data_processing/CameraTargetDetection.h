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

#ifndef MULTISENSORCALIBRATION_CAMERATARGETDETECTION_H
#define MULTISENSORCALIBRATION_CAMERATARGETDETECTION_H

// Std
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

// ROS
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// multisensor_calibration
#include "../common/common.h"
#include "CameraDataProcessor.h"
#include <multisensor_calibration_interface/srv/camera_intrinsics.hpp>
#include <multisensor_calibration_interface/srv/capture_calib_target.hpp>
#include <multisensor_calibration_interface/srv/data_processor_state.hpp>

namespace fs     = std::filesystem;
namespace interf = multisensor_calibration_interface;

namespace multisensor_calibration
{

/**
 * @ingroup nodes
 * @ingroup  target_detection
 * @brief Node to run the processing of the camera data and, in turn, the
 * detection of the calibration target within the camera data isolated from the
 * rest.
 */
class CameraTargetDetection : public rclcpp::Node
{
    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Initialization Constructor
     */
    CameraTargetDetection(
      const std::string& nodeName,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Initialization constructor
     */
    CameraTargetDetection(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Default Destructor
     */
    virtual ~CameraTargetDetection();

  private:
    /**
     * @brief Method to initialize services.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool initializeServices(rclcpp::Node* ipNode);

    /**
     * @brief Method to initialize subscribers.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    bool initializeSubscribers(rclcpp::Node* ipNode);

    /**
     * @brief Callback function handling image messages.
     *
     * @param[in] ipImgMsg image message.
     */
    void onImageReceived(const InputImage_Message_T::ConstSharedPtr& ipImgMsg);

    /**
     * @brief Handle reception of camera info message of camera.
     */
    void onCameraInfoReceived(const sensor_msgs::msg::CameraInfo::SharedPtr pCamInfo);

    /**
     * @brief Service call to get camera intrinsics.
     *
     * @param[in] iReq Request, UNUSED.
     * @param[out] oRes Response.
     */
    bool onRequestCameraIntrinsics(
      const std::shared_ptr<interf::srv::CameraIntrinsics::Request> ipReq,
      std::shared_ptr<interf::srv::CameraIntrinsics::Response> opRes);

    /**
     * @brief Service call to request capturing of calibration target
     *
     * @param[in] ipReq Request, with flag to capture calibration target
     * @param[out] opRes Response, empty.
     */
    bool onRequestTargetCapture(
      const std::shared_ptr<interf::srv::CaptureCalibTarget::Request> ipReq,
      std::shared_ptr<interf::srv::CaptureCalibTarget::Response> opRes);

    /**
     * @brief Service call to request state of of the data processor
     *
     * @param[in] ipReq Request, UNUSED.
     * @param[out] opRes Response, holding the initialization state and the frame id of the data
     * processed.
     */
    bool onRequestState(
      const std::shared_ptr<interf::srv::DataProcessorState::Request> ipReq,
      std::shared_ptr<interf::srv::DataProcessorState::Response> opRes);

    /**
     * @brief Setup launch parameters.
     *
     * The implementation within this class hold launch parameters that are common to all
     * calibration nodes.
     *
     * @param[in] ipNode Pointer to node.
     */
    void setupLaunchParameters(rclcpp::Node* ipNode) const;

    /**
     * @brief Method to read launch parameters. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @param[in] ipNode Pointer to node.
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    bool readLaunchParameters(const rclcpp::Node* ipNode);

    //--- MEMBER DECLARATION ---//

  private:
    /// Flag indicating if node is initialized
    bool isInitialized_;

    /// Mutex guarding the image callback
    std::mutex imageCallbackMutex_;

    /// Pointer to service to get camera intrinsics
    rclcpp::Service<interf::srv::CameraIntrinsics>::SharedPtr pCameraIntrSrv_;

    /// Pointer to service to request capturing of calibration target.
    rclcpp::Service<interf::srv::CaptureCalibTarget>::SharedPtr pCaptureSrv_;

    /// Pointer to service to request the processor state
    rclcpp::Service<interf::srv::DataProcessorState>::SharedPtr pStateSrv_;

    /// Subscriber to image topic
    image_transport::Subscriber imageSubsc_;

    /// Subscriber to camera info topic
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr pCamInfoSubsc_;

    /// Path to target configuration file
    fs::path calibTargetFilePath_;

    /// Namespace of camera to which to subscribe
    std::string cameraNamespace_;

    /// Name of image to which to subscribe within cameraNamespace_
    std::string imageName_;

    /// Frame id of image received by #imageSubsc_
    std::string imageFrameId_;

    /// Pointer to object of camera data processor, responsible to detect calibration target
    /// in camera image data.
    std::shared_ptr<CameraDataProcessor> pCamDataProcessor_;

    /// State of image received on the subscribed topic
    EImageState imageState_;

    /// Flag to capture calibration target in next sensor data package.
    bool captureCalibrationTarget_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_CAMERATARGETDETECTION_H
