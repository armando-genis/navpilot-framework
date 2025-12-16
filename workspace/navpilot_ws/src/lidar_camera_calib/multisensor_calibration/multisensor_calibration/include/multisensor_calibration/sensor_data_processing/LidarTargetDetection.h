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

#ifndef MULTISENSORCALIBRATION_LIDARTARGETDETECTION_H
#define MULTISENSORCALIBRATION_LIDARTARGETDETECTION_H

// Std
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

// Eigen
#include <Eigen/Geometry>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv2/core.hpp>

// multisensor_calibration
#include "../common/common.h"
#include "../config/LidarTargetDetectionParameters.hpp"
#include "LidarDataProcessor.h"
#include <multisensor_calibration/common/lib3D/core/camera.hpp>
#include <multisensor_calibration_interface/srv/capture_calib_target.hpp>
#include <multisensor_calibration_interface/srv/data_processor_state.hpp>

namespace fs     = std::filesystem;
namespace interf = multisensor_calibration_interface;

namespace multisensor_calibration
{

/**
 * @brief Node to run the processing of the LiDAR data and, in turn, the
 * detection of the calibration target within the LiDAR point clouds isolated
 * from the rest.
 */
class LidarTargetDetection : public rclcpp::Node
{

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Initialization Constructor
     */
    LidarTargetDetection(
      const std::string& nodeName,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Initialization constructor
     */
    LidarTargetDetection(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Default Destructor
     */
    virtual ~LidarTargetDetection();

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
     * @brief Callback function handling point cloud messages.
     *
     * @param[in] ipCloudMsg cloud message.
     */
    void onCloudReceived(const InputCloud_Message_T::ConstSharedPtr& ipCloudMsg);

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
     * @brief Setup dynamic parameters.
     *
     * @param[in] ipNode Pointer to node.
     */
    void setupDynamicParameters(rclcpp::Node* ipNode) const;

    /**
     * @brief Method to read launch parameters. This overrides the method of the parent class.
     * In this, the parent method is also called.
     *
     * @param[in] ipNode Pointer to node.
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    bool readLaunchParameters(const rclcpp::Node* ipNode);

    /**
     * @brief Handle dynamic change of parameters. For example, through rqt_reconfigure.
     *
     * This loops through the list of parameters in iParameters and sets their value accordingly.
     *
     * @param[in] iParameters Parameters that have changed.
     */
    rcl_interfaces::msg::SetParametersResult handleDynamicParameterChange(
      const std::vector<rclcpp::Parameter>& iParameters);

    /**
     * @brief Virtual function to set dynamic parameter. This is called from
     * handleDynamicParameterChange for each parameter in the list that is to be changed.
     *
     * @param[in] iParameter Parameter that is to be changed.
     * @return True, if successful, i.e. if it has been changed. False, otherwise.
     */
    bool setDynamicParameter(const rclcpp::Parameter& iParameter);

    //--- MEMBER DECLARATION ---//

  private:
    /// Flag indicating if node is initialized
    bool isInitialized_;

    /// Mutex guarding the cloud callback
    std::mutex cloudCallbackMutex_;

    /// Object holding parameters for the target detection in LiDAR data
    LidarTargetDetectionParameters lidarTargetDetectionParams_;

    /// Callback handle to adjust parameters
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr pParameterCallbackHandle_;

    /// Pointer to service to request capturing of calibration target.
    rclcpp::Service<interf::srv::CaptureCalibTarget>::SharedPtr pCaptureSrv_;

    /// Pointer to service to request the processor state
    rclcpp::Service<interf::srv::DataProcessorState>::SharedPtr pStateSrv_;

    /// Subscriber to point cloud topic
    rclcpp::Subscription<InputCloud_Message_T>::SharedPtr pCloudSubsc_;

    /// Path to target configuration file
    fs::path calibTargetFilePath_;

    /// Name of clout to which to subscribe
    std::string cloudTopicName_;

    /// Frame id of cloud received by #cloudSubsc_
    std::string cloudFrameId_;

    /// Pointer to object of lidar data processor, responsible to detect calibration target
    /// in lidar cloud data.
    std::shared_ptr<LidarDataProcessor> pLidarDataProcessor_;

    /// Flag to capture calibration target in next sensor data package.
    bool captureCalibrationTarget_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_LIDARTARGETDETECTION_H
