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

#ifndef MULTISENSORCALIBRATION_GUIDANCEBASE_H
#define MULTISENSORCALIBRATION_GUIDANCEBASE_H

// Std
#include <rclcpp/executor.hpp>
#include <rclcpp/logger.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

// Eigen
#include <Eigen/Geometry>

// multisensor_calibration
#include "../common/common.h"
#include "multisensor_calibration/calibration_target/CalibrationTarget.hpp"
#include "multisensor_calibration_interface/srv/calibration_meta_data.hpp"
#include <multisensor_calibration/common/lib3D/core/extrinsics.hpp>
#include <multisensor_calibration_interface/srv/reset_calibration.hpp>
#include <multisensor_calibration_interface/srv/sensor_extrinsics.hpp>

namespace multisensor_calibration
{

/**
 * @ingroup guidance
 * @brief Base class for all classes that guide the user in the calibration process.
 *
 * @note The guidance feature of the calibration is currently still in development and not yet
 * functional.
 *
 */
class GuidanceBase
{

    //--- METHOD DECLARATION ---//
  public:
    /**
     * @brief Default constructor.
     */
    GuidanceBase(rclcpp::Node* pNode);

    /**
     * @brief Destructor
     */
    virtual ~GuidanceBase();

  protected:
    typedef multisensor_calibration_interface::srv::CalibrationMetaData CalibrationMetadataSrv;
    typedef multisensor_calibration_interface::srv::SensorExtrinsics SensorExtrinsicsSrv;
    /**
     * @brief Function to compute bound on axis along 'iVec' w.r.t to 'iPnt' by intersecting the axis
     * with the plane 'iPlane'.
     */
    float computeAxisBound(const Eigen::Vector3d& iPnt, const Eigen::Vector3d& iVec,
                           const Eigen::Vector4d& iPlane) const;

    /**
     * @brief Method to compute overlapping Fov between the two sensors based on the estimated extrinsic
     * pose.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    virtual void computeExtrinsicFovBoundingPlanes() = 0;

    /**
     * @brief Method to compute Fov of the two sensors based on the intrinsic parameters.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    virtual bool computeIntrinsicFovBoundingPlanes() = 0;

    /**
     * @brief Method to compute next target pose.
     *
     * @note This is a pure virtual method and needs to be implemented by the specific subclasses.
     */
    virtual void computeNextTargetPose() = 0;

    /**
     * @brief Method to initialize publishers
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    virtual bool initializePublishers() = 0;

    /**
     * @brief Method to initialize services
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    virtual bool initializeServices();

    /**
     * @brief Method to initialize subscribers.
     *
     * @return True, if all settings are valid. False, otherwise.
     */
    virtual bool initializeSubscribers();

    /**
     * @brief Method to initialize timers.
     *
     * @param[in, out] ioNh Reference to node handle
     * @return True, if successful. False otherwise.
     */
    virtual bool initializeTimers();

    /**
     * @brief Function to check if pose of the calibration target is within the bounding planes.
     *
     * @return True, if target with given pose is within bounding planes. False, otherwise.
     */
    bool isTargetPoseWithinBoundingPlanes(const lib3d::Extrinsics& iPose) const;

    /**
     * @brief Callback function handling calibration results messages.
     *
     * @param[in] ipResultMsg Calibration result message.
     */
    virtual void onCalibrationResultReceived(const CalibrationResult_Message_T::ConstSharedPtr& ipResultMsg);

    /**
     * @brief Callback function handling target pose messages.
     *
     * @param[in] ipPoseMsg Target pose message.
     */
    virtual void onTargetPoseReceived(const TargetBoardPose_Message_T::ConstSharedPtr& ipPoseMsg);

    /**
     * @brief Method to read launch parameters
     *
     * @param[in] iNh Object of node handle
     * @return True if successful. False, otherwise (e.g. if sanity check fails)
     */
    virtual bool readLaunchParameters();

    /**
     * @brief Method to reset nextTargetPose_
     */
    virtual void resetNextTargetPose() = 0;

  private:
    /**
     * @brief Method to get calibration meta data. This is connected to the calibMetaDataTimer_.
     */
    void getCalibrationMetaData();

    /**
     * @brief Method to get initial sensor pose from the calibration object.
     */
    bool getInitialSensorPose();

    /**
     * @brief Service call to request reset of calibration
     *
     * @param[in] iReq Request, empty
     * @param[out] oRes Response.
     */
    bool onReset(multisensor_calibration_interface::srv::ResetCalibration::Request::SharedPtr iReq,
                 multisensor_calibration_interface::srv::ResetCalibration::Response::SharedPtr oRes);

    //--- MEMBER DECLARATION ---//

  protected:
    /// Flag indicating if node is initialized.
    bool isInitialized_;

    /// Flag indicating if the initialPose has been received
    bool initialPoseReceived_;

    /// Name of the app.
    std::string appTitle_;

    /// Global node handler.
    rclcpp::Node* pNode_;

    /// ROS2 Executor
    rclcpp::Executor* pExecutor_;

    /// Name of the calibrator node
    std::string calibratorNodeName_;

    /// Timer object to trigger service call to get calibration meta data.
    rclcpp::TimerBase::SharedPtr pCalibMetaDataTimer_;

    /// Subscriber to calibration result messages.
    rclcpp::Subscription<CalibrationResult_Message_T>::SharedPtr pCalibResultSubsc_;

    /// Subscriber to target pose messages.
    rclcpp::Subscription<TargetBoardPose_Message_T>::SharedPtr pTargetPoseSubsc_;

    /// Server to provide service to reset calibration
    rclcpp::Service<multisensor_calibration_interface::srv::ResetCalibration>::SharedPtr pResetSrv_;

    /// Member variable holding calibration meta data.
    CalibrationMetadataSrv::Response::SharedPtr pCalibrationMetaData_;

    /// Extrinsic pose between the sensors to calibrate
    lib3d::Extrinsics extrinsicSensorPose_;

    /// List of planes encapsulating the overlapping FoV between the sensors.
    std::vector<Eigen::Vector4d> fovBoundingPlanes_;

    /// Object of the calibration target used.
    CalibrationTarget calibrationTarget_;

    /// List of target poses detected during calibration
    std::vector<lib3d::Extrinsics> detectedTargetPoses_;

    /// Pose of the calibration target that is to be placed next.
    lib3d::Extrinsics nextTargetPose_;

    /// Axes to be covered by the calibration
    std::array<Eigen::Vector3d, 3> axes_;

    /// Services
    rclcpp::Client<CalibrationMetadataSrv>::SharedPtr pMetaDataClient_;
    rclcpp::Client<SensorExtrinsicsSrv>::SharedPtr extrinsicsClient_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_GUIDANCEBASE_H