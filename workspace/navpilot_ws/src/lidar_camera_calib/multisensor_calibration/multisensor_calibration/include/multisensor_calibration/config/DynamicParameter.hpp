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

#ifndef MULTISENSORCALIBRATION_DYNAMICPARAMETER_HPP
#define MULTISENSORCALIBRATION_DYNAMICPARAMETER_HPP

// Std
#include <type_traits>

// ROS
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter.hpp>

// multisensor_calibration
#include "../common/common.h"

/// Macro to declare given parameter for given node pointer
#define DECLARE_PARAMETER(param, node_ptr) \
    node_ptr->declare_parameter(STRINGIFY(param), param.value, param.desc);

/// Macro to set given parameter from given rclcpp::Parameter
#define SET_PARAMETER_FROM_RCLCPP_PARAM(param, rclpp_param) \
    if (rclpp_param.get_name() == STRINGIFY(param))         \
        return param.setValue(rclpp_param);

namespace multisensor_calibration
{

template <typename T>
struct DynamicParameter
{
    T value;

    rcl_interfaces::msg::ParameterDescriptor desc;

    /**
     * @brief Initialization constructor
     *
     * @param[in] defaultValue Default value to be set for parameter.
     * @param[in] descriptionText Description to be set.
     */
    DynamicParameter(const T& defaultValue,
                     const std::string& descriptionText) :
      value(defaultValue),
      desc({})
    {
        desc.description = descriptionText;
    };

    /**
     * @brief Initialization constructor for bounded integer parameters
     *
     * @param[in] defaultValue Default value to be set for parameter.
     * @param[in] descriptionText Description to be set.
     * @param[in] min Minimum value (inclusive)
     * @param[in] max Maximum value (inclusive)
     * @param[in] step Step between min and max. If left with 0, a continuous range is assumed.
     */
    DynamicParameter(const int& defaultValue,
                     const std::string& descriptionText,
                     const int& min, const int& max, const int& step = 0) :
      DynamicParameter(defaultValue, descriptionText)
    {
        if (std::is_same<int, T>::value)
        {
            rcl_interfaces::msg::IntegerRange intRange;
            intRange.from_value = min;
            intRange.to_value   = max,
            intRange.step       = step;

            desc.description   = descriptionText;
            desc.integer_range = {intRange};
        }
    };

    /**
     * @brief Initialization constructor for bounded floating point parameters
     *
     * @param[in] defaultValue Default value to be set for parameter.
     * @param[in] descriptionText Description to be set.
     * @param[in] min Minimum value (inclusive)
     * @param[in] max Maximum value (inclusive)
     * @param[in] step Step between min and max. If left with 0, a continuous range is assumed.
     */
    DynamicParameter(const double& defaultValue,
                     const std::string& descriptionText,
                     const double& min, const double& max, const double& step = 0.0) :
      DynamicParameter(defaultValue, descriptionText)
    {
        if (std::is_same<double, T>::value)
        {
            rcl_interfaces::msg::FloatingPointRange fpRange;
            fpRange.from_value = min;
            fpRange.to_value   = max,
            fpRange.step       = step;

            desc.description          = descriptionText;
            desc.floating_point_range = {fpRange};
        }
    };

    /**
     * @brief Set value from rclcpp::Parameter.
     *
     * @return True, if successful. False, otherwise.
     */
    bool setValue(const rclcpp::Parameter& iParameter)
    {
        this->value = iParameter.get_value<T>();

        return true;
    }
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_DYNAMICPARAMETER_HPP