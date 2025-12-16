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

#ifndef MULTISENSORCALIBRATION_CIRCULARCUTOUT_HPP
#define MULTISENSORCALIBRATION_CIRCULARCUTOUT_HPP

// multisensor_calibration
#include "Cutout.h"

namespace multisensor_calibration
{

/**
 * @ingroup  target_detection
 * @brief Class implementing a circular cutout geometry from the calibration board.
 * Inheriting #Cutout
 *
 * Geometry ID: 1
 * Number of coefficients: 3
 * Coefficients: {Center.X, Center.Y, Radius}
 */
class CircularCutout : public Cutout
{
    //--- METHOD DECLaRATION ---//

  public:
    /**
     * @brief Empty Constructor
     */
    CircularCutout();

    /**
     * @brief Constructor initializing the geometry coefficients.
     *
     * @param[in] iCoefficients coefficients to initialize.
     */
    CircularCutout(const std::vector<float> iCoefficients);

    /**
     * @brief Destructor
     */
    virtual ~CircularCutout();

    /**
     * @overload
     * @brief Get radius of cutout. If cutout is not circular, this should be the biggest radius
     * which fully fits into the cutout.
     */
    float getRadius() const override;

    /**
     * @overload
     * @brief Get X coordinate of center point.
     */
    float getCenterX() const override;

    /**
     * @overload
     * @brief Get Y coordinate of center point.
     */
    float getCenterY() const override;

    /**
     * @overload
     * @brief Method to test whether a point lies inside the cutout.
     *
     * @param[in] x X-value of the point in the local coordinate system of the calibration target.
     * @param[in] y Y-value of the point in the local coordinate system of the calibration target.
     * @param[out] opDistance Pointer to return value providing the distance to the center of the
     * cutout. Set to nullptr if not used.
     * @param[out] opPenalty Pointer to return value providing a penalty based on the distance of
     * the point to the center of the cutout.
     * @return True, if point is inside the cutout. False, otherwise.
     */
    bool isPointInside(const float x, const float y,
                       float* opDistance = nullptr, float* opPenalty = nullptr) const override;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_CIRCULARCUTOUT_HPP